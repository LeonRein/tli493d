use core::fmt::Debug;
use core::marker::PhantomData;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c;

use crate::register::{
    REG_CACHE_LEN, REG_CONFIG, REG_CONFIG2, REG_MOD1, REG_MOD2, decode_data_frame,
    has_valid_bus_parity, set_bit, set_bits, set_config_parity, set_fuse_parity,
};
use crate::types::{
    AddressSlot, Diagnostics, Error, PowerMode, RawReading, TriggerMode,
    UpdateRate,
};
use crate::variant::{BxByBzTemp, MeasurementShape, SensorVariant, VariantSensitivity};

/// Asynchronous TLI493D driver.
///
/// `I2c` is any async I2C implementation accepted by `embedded-hal-async`
/// (for example a peripheral instance or a shared-bus proxy type). `V` is the
/// compile-time sensor variant marker. `M` is the compile-time measurement shape,
/// determining which fields are read and what type `read()` returns.
pub struct Tli493d<I2c, V, M = BxByBzTemp> {
    i2c: I2c,
    address: u8,
    power_mode: PowerMode,
    sensitivity_scale: f32,
    reg_cache: [u8; REG_CACHE_LEN],
    last_frame: Option<u8>,
    last_diag: Diagnostics,
    _variant: PhantomData<V>,
    _mode: PhantomData<M>,
}

impl<I2c, V> Tli493d<I2c, V, BxByBzTemp>
where
    I2c: i2c::I2c,
    <I2c as i2c::ErrorType>::Error: Debug,
    V: SensorVariant,
{
    /// Creates and initializes a sensor instance.
    ///
    /// The caller provides a valid sensor address slot and desired initial
    /// power mode. The function performs a reset sequence as specified in the
    /// datasheet, applies variant defaults, then writes initial mode settings.
    ///
    /// The returned driver defaults to [`BxByBzTemp`] measurement shape
    /// (X, Y, Z, temperature). Use [`into_measurement_mode`] to change it.
    ///
    /// # Notes
    ///
    /// The reset sequence includes a mandatory 30µs delay; the caller must
    /// provide a suitable delay implementation. The sensor is assumed to be
    /// powered and stable before calling this constructor.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] when initialization bus transactions fail.
    pub async fn new(
        i2c: I2c,
        delay: &mut impl DelayNs,
        slot: AddressSlot,
        power_mode: PowerMode,
    ) -> Result<Self, Error<<I2c as i2c::ErrorType>::Error>> {
        let address = slot.as_7bit();
        #[cfg(feature = "defmt")]
        defmt::trace!("Tli493d::new addr=0x{:02X} skip_reset={}", address, V::SKIP_RESET_SEQUENCE);

        let mut this = Self {
            i2c,
            address,
            power_mode,
            sensitivity_scale: 1.0,
            reg_cache: [0u8; REG_CACHE_LEN],
            last_frame: None,
            last_diag: Diagnostics::from_diag_byte(0),
            _variant: PhantomData,
            _mode: PhantomData,
        };

        // Reset sequence as specified in the user manual section 2.3:
        // Send 0xFF to recovery address twice, then 0x00 twice, followed by 30µs delay.
        // Use 1-byte dummy writes instead of empty writes: some I2C controller
        // implementations (e.g. embassy-rp) hang when transmitting 0 data bytes.
        //
        // A2B6 does support this I2C reset (and the manual recommends it after
        // each power-up to clear power-on spikes). It is skipped here because the
        // caller power-cycles each sensor individually via its own supply switch,
        // so each comes up cleanly at its default address.
        if !V::SKIP_RESET_SEQUENCE {
            #[cfg(feature = "defmt")]
            defmt::trace!("  running I2C reset sequence");
            let _ = this.i2c.write(0x7f, &[0x00]).await;
            let _ = this.i2c.write(0x7f, &[0x00]).await;
            let _ = this.i2c.write(0x00, &[0x00]).await;
            let _ = this.i2c.write(0x00, &[0x00]).await;
            delay.delay_us(30).await;
        }

        if V::SKIP_RESET_SEQUENCE {
            // Gen-2 sensors (A2B6): the register map has reserved bits — in
            // MOD2 (0x13, bits 6:0) and the reserved register 0x12 — that the
            // user manual requires be preserved ("Reserved: bits that must keep
            // the default values (read prior to write required)"). Writing them
            // to 0 corrupts the sensor; an incorrect fuse parity (which covers
            // MOD1 and MOD2 bit 7) puts it into an error state that only a power
            // cycle clears — the sensor stops acknowledging on the bus.
            //
            // Register reads only return real data once PR=1 (1-byte read mode)
            // is enabled, so the sequence is:
            //   1. write MOD1 with PR=1 (+ IICADR, CA=0, INT=1) — MOD1 has no
            //      reserved bits, and its fuse parity is valid because MOD2
            //      bit 7 (PRD) is 0 at reset.
            //   2. read the register map back so the cache holds the sensor's
            //      real reserved-bit values.
            //   3. apply remaining config from the real cache, preserving those
            //      reserved bits on every subsequent write.
            let slot_bits = match slot {
                AddressSlot::A0 => 0,
                AddressSlot::A1 => 1,
                AddressSlot::A2 => 2,
                AddressSlot::A3 => 3,
            };

            this.reg_cache[REG_MOD2] = V::RESET_MOD2;
            this.reg_cache[REG_MOD1] = set_bits(V::RESET_MOD1, 0x60, 5, slot_bits);
            this.reg_cache[REG_MOD1] = set_bit(this.reg_cache[REG_MOD1], 0x10, true);
            this.reg_cache[REG_MOD1] = set_bit(this.reg_cache[REG_MOD1], 0x08, false);
            this.reg_cache[REG_MOD1] = set_bit(this.reg_cache[REG_MOD1], 0x04, true);
            this.reg_cache[REG_MOD1] = set_fuse_parity(
                this.reg_cache[REG_MOD1],
                this.reg_cache[REG_MOD2],
                V::PRD_MASK,
            );

            this.i2c
                .write(this.address, &[REG_MOD1 as u8, this.reg_cache[REG_MOD1]])
                .await
                .map_err(Error::I2c)?;

            // Read the register map back (PR=1 is now active) so the cache holds
            // the sensor's actual contents, including reserved bits that must be
            // preserved on write. Reads through 0x13 (MOD2) suffice.
            let mut readback = [0u8; 0x14];
            this.i2c
                .read(this.address, &mut readback)
                .await
                .map_err(Error::I2c)?;
            this.reg_cache[..0x14].copy_from_slice(&readback);
            #[cfg(feature = "defmt")]
            defmt::trace!(
                "  gen-2 readback: CONFIG=0x{:02X} MOD1=0x{:02X} RSVD12=0x{:02X} MOD2=0x{:02X}",
                this.reg_cache[REG_CONFIG], this.reg_cache[REG_MOD1],
                this.reg_cache[0x12], this.reg_cache[REG_MOD2]
            );

            // Write CONFIG: CONFIG has no reserved bits, so start from the reset
            // defaults for the writable fields and fix up the configuration parity.
            this.reg_cache[REG_CONFIG] = set_config_parity(V::RESET_CONFIG, V::WAKEUP_CONFIG_PARITY);
            this.i2c
                .write(this.address, &[REG_CONFIG as u8, this.reg_cache[REG_CONFIG]])
                .await
                .map_err(Error::I2c)?;

            this.set_power_mode(power_mode).await?;
        } else {
            #[cfg(feature = "defmt")]
            defmt::trace!("  gen-3: applying reset defaults");
            this.apply_reset_defaults();
            this.set_power_mode(power_mode).await?;
        }

        #[cfg(feature = "defmt")]
        defmt::trace!("  init complete");
        Ok(this)
    }
}

impl<I2c, V, M> Tli493d<I2c, V, M>
where
    I2c: i2c::I2c,
    <I2c as i2c::ErrorType>::Error: Debug,
    V: SensorVariant,
    M: MeasurementShape,
{

    /// Returns the underlying I2C object.
    ///
    /// The driver is consumed and the stored I2C object is returned.
    pub fn into_i2c(self) -> I2c {
        self.i2c
    }

    /// Most recently decoded diagnostic flags.
    ///
    /// Diagnostics are updated by [`Self::read_raw`] and [`Self::read`].
    pub fn diagnostics(&self) -> Diagnostics {
        self.last_diag
    }

    /// Reads one raw measurement frame.
    ///
    /// Diagnostic and parity flags are validated as part of the read.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] when reading the frame fails.
    pub async fn read_raw(&mut self) -> Result<RawReading, Error<<I2c as i2c::ErrorType>::Error>> {
        let mut frame = [0u8; 7];
        self.i2c
            .read(self.address, &mut frame)
            .await
            .map_err(Error::I2c)?;

        self.reg_cache[..7].copy_from_slice(&frame);
        let (raw, diag) = decode_data_frame(&frame);
        self.validate_frame(&frame, diag)?;

        if let Some(last) = self.last_frame
            && last == diag.frame
        {
            return Err(Error::AdcLockup);
        }

        self.last_frame = Some(diag.frame);
        self.last_diag = diag;

        Ok(raw)
    }

    /// Reads one frame converted to engineering values.
    ///
    /// The return type depends on the measurement shape `M`:
    /// - `BxByBzTemp`: returns `Reading` (X, Y, Z, temperature)
    /// - `BxByBz`: returns `XyzReading` (X, Y, Z)
    /// - `BxBy`: returns `XyReading` (X, Y)
    ///
    /// # Errors
    ///
    /// Propagates errors from [`Self::read_raw`].
    pub async fn read(&mut self) -> Result<M::Output, Error<<I2c as i2c::ErrorType>::Error>> {
        let raw = self.read_raw().await?;
        Ok(M::decode(raw, self.sensitivity_scale))
    }

    /// Configures sensitivity.
    ///
    /// The argument type depends on the compile-time variant, so only valid
    /// sensitivity values are accepted at compile time.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] when writing configuration registers fails.
    pub async fn set_sensitivity(
        &mut self,
        sensitivity: V::Sensitivity,
    ) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        self.set_sensitivity_inner(sensitivity).await
    }

    /// Configures measurement mode and changes the measurement shape type.
    ///
    /// This method consumes the driver and returns a new instance with the
    /// measurement shape type parameter changed to `NewM`. This enables
    /// compile-time safety: the return type reflects what will be measured.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] when writing configuration registers fails.
    pub async fn into_measurement_mode<NewM: MeasurementShape>(
        mut self,
    ) -> Result<Tli493d<I2c, V, NewM>, Error<<I2c as i2c::ErrorType>::Error>> {
        self.reg_cache[REG_CONFIG] = set_bits(self.reg_cache[REG_CONFIG], 0x80, 7, NewM::DT);
        self.reg_cache[REG_CONFIG] = set_bits(self.reg_cache[REG_CONFIG], 0x40, 6, NewM::AM);
        self.reg_cache[REG_CONFIG] = set_config_parity(self.reg_cache[REG_CONFIG], V::WAKEUP_CONFIG_PARITY);
        self.write_config_registers().await?;

        Ok(Tli493d {
            i2c: self.i2c,
            address: self.address,
            power_mode: self.power_mode,
            sensitivity_scale: self.sensitivity_scale,
            reg_cache: self.reg_cache,
            last_frame: self.last_frame,
            last_diag: self.last_diag,
            _variant: self._variant,
            _mode: PhantomData,
        })
    }

    /// Starts a single ADC conversion immediately.
    ///
    /// Only meaningful in [`PowerMode::MasterControlled`], where nothing
    /// converts until asked. Use it to prime the pipeline before the first
    /// read: with [`TriggerMode::AfterReg05`] every read starts the *next*
    /// conversion, so the first read of all has nothing in flight to collect.
    ///
    /// User manual Table 6: in a write frame the byte after the sensor address
    /// carries the trigger bits in 7:5 and a register address in 4:0. Trigger
    /// bits `001B` mean "ADC trigger after write frame is finished" (Figure 4).
    /// Register `0x00` is read-only and no data bytes follow, so the frame
    /// triggers a conversion without writing anything. The write protocol is
    /// always the 2-byte form (§2.1), independent of the `PR` bit.
    ///
    /// Writes are never delayed by clock stretching (§2.2.2), so this returns
    /// promptly even with a conversion already in progress.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] when the trigger write fails.
    pub async fn trigger(&mut self) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        self.i2c
            .write(self.address, &[0x20])
            .await
            .map_err(Error::I2c)
    }

    /// Configures trigger mode (`TRIG`) in register `0x10`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] when writing configuration registers fails.
    pub async fn set_trigger_mode(
        &mut self,
        mode: TriggerMode,
    ) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        self.reg_cache[REG_CONFIG] = set_bits(self.reg_cache[REG_CONFIG], 0x30, 4, mode.bits());
        self.reg_cache[REG_CONFIG] = set_config_parity(self.reg_cache[REG_CONFIG], V::WAKEUP_CONFIG_PARITY);
        self.write_config_registers().await
    }

    /// Sets device I2C address slot by updating `IICADR` in `MOD1`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] when writing mode registers fails.
    pub async fn set_address_slot(
        &mut self,
        slot: AddressSlot,
    ) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        let slot_bits = match slot {
            AddressSlot::A0 => 0,
            AddressSlot::A1 => 1,
            AddressSlot::A2 => 2,
            AddressSlot::A3 => 3,
        };

        self.reg_cache[REG_MOD1] = set_bits(self.reg_cache[REG_MOD1], 0x60, 5, slot_bits);
        self.reg_cache[REG_MOD1] = set_fuse_parity(
            self.reg_cache[REG_MOD1],
            self.reg_cache[REG_MOD2],
            V::PRD_MASK,
        );

        // Write MOD1 only — after the IICADR change, the sensor moves to the
        // new address and won't ACK a MOD2 write at the old address.
        //
        // The sensor latches the new IICADR mid-transaction and immediately
        // stops acknowledging at the old address, so the controller reports
        // `NoAcknowledge` on the data byte even though the change took effect.
        // Tolerate that specific error here; the read at the new address below
        // is the real confirmation. Any other error is genuine and propagates.
        let mod1_payload = [REG_MOD1 as u8, self.reg_cache[REG_MOD1]];
        if let Err(e) = self.i2c.write(self.address, &mod1_payload).await {
            use i2c::Error as _;
            if !matches!(e.kind(), i2c::ErrorKind::NoAcknowledge(_)) {
                return Err(Error::I2c(e));
            }
        }

        self.address = slot.as_7bit();

        // Verify the sensor actually responds at the new address.
        let mut buf = [0u8; 1];
        self.i2c
            .read(self.address, &mut buf)
            .await
            .map_err(Error::I2c)?;

        #[cfg(feature = "defmt")]
        defmt::trace!("  set_addr: moved to slot, confirmed at 0x{:02X}", self.address);
        Ok(())
    }

    /// Configures power mode via `MODE` bits in `MOD1`.
    ///
    /// This updates cached configuration and writes `MOD1`/`MOD2` while keeping
    /// reserved register defaults untouched.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] when writing mode registers fails.
    pub async fn set_power_mode(
        &mut self,
        mode: PowerMode,
    ) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        self.reg_cache[REG_MOD1] = set_bits(self.reg_cache[REG_MOD1], 0x03, 0, mode.bits());
        self.reg_cache[REG_MOD1] = set_fuse_parity(
            self.reg_cache[REG_MOD1],
            self.reg_cache[REG_MOD2],
            V::PRD_MASK,
        );

        self.write_mode_registers().await?;
        self.power_mode = mode;
        Ok(())
    }

    /// Configures the fast/slow update bit (`PRD`).
    ///
    /// This maps to the `PRD` bit in `MOD2` for generation-2 variants.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] when writing update-rate registers fails.
    pub async fn set_update_rate(
        &mut self,
        rate: UpdateRate,
    ) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        let bits = V::update_rate_bits(rate).ok_or(Error::UnsupportedUpdateRate)?;
        self.reg_cache[REG_MOD2] = set_bits(self.reg_cache[REG_MOD2], V::PRD_MASK, V::PRD_SHIFT, bits);
        self.reg_cache[REG_MOD1] = set_fuse_parity(
            self.reg_cache[REG_MOD1],
            self.reg_cache[REG_MOD2],
            V::PRD_MASK,
        );

        self.write_mode_registers().await
    }

    fn apply_reset_defaults(&mut self) {
        self.reg_cache[REG_CONFIG] = V::RESET_CONFIG;
        // Force 1-byte read protocol (PR=1) so data reads can be performed as
        // a single plain I2C read transaction.
        self.reg_cache[REG_MOD1] = set_bit(V::RESET_MOD1, 0x10, true);
        self.reg_cache[REG_MOD2] = V::RESET_MOD2;
        self.reg_cache[REG_MOD1] = set_fuse_parity(
            self.reg_cache[REG_MOD1],
            self.reg_cache[REG_MOD2],
            V::PRD_MASK,
        );
        if V::HAS_X4 {
            self.reg_cache[REG_CONFIG2] = V::RESET_CONFIG2;
        }
    }

    async fn write_mode_registers(&mut self) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        // MOD1 (0x11) and MOD2 (0x13) are written as separate single-register
        // transactions: the sensor does not support repeated starts, and MOD2
        // carries reserved factory bits that must be preserved (the cache holds
        // the values read back at init).
        #[cfg(feature = "defmt")]
        defmt::trace!(
            "  write MOD1=0x{:02X} MOD2=0x{:02X} to 0x{:02X}",
            self.reg_cache[REG_MOD1], self.reg_cache[REG_MOD2], self.address
        );
        self.i2c
            .write(self.address, &[REG_MOD1 as u8, self.reg_cache[REG_MOD1]])
            .await
            .map_err(Error::I2c)?;
        self.i2c
            .write(self.address, &[REG_MOD2 as u8, self.reg_cache[REG_MOD2]])
            .await
            .map_err(Error::I2c)?;

        Ok(())
    }

    async fn write_config_registers(&mut self) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        // Write only CONFIG register (0x10) — matches the C++ library which
        // writes one register at a time.  Multi-byte writes would also touch
        // MOD1/MOD2 and corrupt the address change.
        self.i2c
            .write(self.address, &[REG_CONFIG as u8, self.reg_cache[REG_CONFIG]])
            .await
            .map_err(Error::I2c)?;

        if V::HAS_X4 {
            self.i2c
                .write(self.address, &[REG_CONFIG2 as u8, self.reg_cache[REG_CONFIG2]])
                .await
                .map_err(Error::I2c)
        } else {
            Ok(())
        }
    }

    async fn set_sensitivity_inner(
        &mut self,
        sensitivity: V::Sensitivity,
    ) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        let x2 = sensitivity.x2();
        let x4 = sensitivity.x4();

        self.reg_cache[REG_CONFIG] = set_bit(self.reg_cache[REG_CONFIG], 0x08, x2);
        self.reg_cache[REG_CONFIG] = set_config_parity(self.reg_cache[REG_CONFIG], V::WAKEUP_CONFIG_PARITY);
        if V::HAS_X4 {
            self.reg_cache[REG_CONFIG2] = set_bit(self.reg_cache[REG_CONFIG2], 0x01, x4);
        }

        self.write_config_registers().await?;
        self.sensitivity_scale = sensitivity.scale();
        Ok(())
    }

    fn validate_frame(
        &self,
        frame: &[u8; 7],
        diag: Diagnostics,
    ) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        if !diag.ff {
            return Err(Error::InvalidFuseParity);
        }
        if !diag.cf {
            return Err(Error::InvalidConfigurationParity);
        }
        if diag.t {
            return Err(Error::InvalidMeasurementData);
        }
        if !has_valid_bus_parity(frame, diag) {
            return Err(Error::InvalidBusParity);
        }

        // PD0 reports completion of the Bx conversion and applies to every
        // measurement shape. PD3 reports completion of the *temperature*
        // conversion (user manual §1.2.5), so it only carries data-ready
        // information when temperature is actually being measured. With DT=1
        // there is no temperature conversion for it to describe, and gating on
        // it would reject every frame.
        //
        // Fast mode is exempt from both: conversions run continuously, so the
        // flags are not a meaningful readiness signal there.
        if self.power_mode != PowerMode::Fast {
            let temperature_enabled = M::DT == 0;
            if !diag.pd0 || (temperature_enabled && !diag.pd3) {
                return Err(Error::DataNotReady);
            }
        }

        Ok(())
    }
}
