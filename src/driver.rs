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

        // Reset sequence as specified in datasheet section 2.3:
        // Send 0xFF to recovery address twice, then 0x00 twice, followed by 30µs delay.
        // Use 1-byte dummy writes instead of empty writes: some I2C controller
        // implementations (e.g. embassy-rp) hang when transmitting 0 data bytes.
        //
        // Gen-2 sensors (A2B6) do not support software reset; skip it and
        // rely on the power-on reset performed by the caller.
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
            // Gen-2 sensors: write MOD1 with PR=1 (enable 1-byte read mode),
            // then set CA=0 and INT=1 in MOD1. Skip register readback — the
            // C++ library reads registers but we don't need them, and if the
            // first MOD1 write doesn't take effect the readback returns a data
            // frame instead of register values.
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

            let mod1_payload = [REG_MOD1 as u8, this.reg_cache[REG_MOD1]];
            this.i2c
                .write(this.address, &mod1_payload)
                .await
                .map_err(Error::I2c)?;
            #[cfg(feature = "defmt")]
            defmt::trace!("  gen-2: wrote MOD1={}", this.reg_cache[REG_MOD1]);

            // Write CONFIG with reset defaults and correct parity handling.
            this.reg_cache[REG_CONFIG] = set_config_parity(V::RESET_CONFIG, V::WAKEUP_CONFIG_PARITY);
            let config_payload = [REG_CONFIG as u8, this.reg_cache[REG_CONFIG]];
            this.i2c
                .write(this.address, &config_payload)
                .await
                .map_err(Error::I2c)?;
            #[cfg(feature = "defmt")]
            defmt::trace!("  gen-2: wrote CONFIG=0x{:02X}", this.reg_cache[REG_CONFIG]);

            this.set_power_mode(power_mode).await?;
            #[cfg(feature = "defmt")]
            defmt::trace!("  gen-2 init OK");
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
        let mod1_payload = [REG_MOD1 as u8, self.reg_cache[REG_MOD1]];
        self.i2c
            .write(self.address, &mod1_payload)
            .await
            .map_err(Error::I2c)?;

        let new_addr = slot.as_7bit();
        self.address = new_addr;

        // Verify the sensor actually responds at the new address.
        let mut buf = [0u8; 1];
        self.i2c
            .read(self.address, &mut buf)
            .await
            .map_err(Error::I2c)?;

        #[cfg(feature = "defmt")]
        defmt::trace!("  set_addr: confirmed at 0x{:02X}", self.address);
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
        let mod1_payload = [REG_MOD1 as u8, self.reg_cache[REG_MOD1]];
        let mod2_payload = [REG_MOD2 as u8, self.reg_cache[REG_MOD2]];

        #[cfg(feature = "defmt")]
        defmt::trace!("    write MOD1: [0x{:02X}, 0x{:02X}] to 0x{:02X}", mod1_payload[0], mod1_payload[1], self.address);
        self.i2c
            .write(self.address, &mod1_payload)
            .await
            .map_err(Error::I2c)?;
        #[cfg(feature = "defmt")]
        defmt::trace!("    wrote MOD1 OK");

        #[cfg(feature = "defmt")]
        defmt::trace!("    write MOD2: [0x{:02X}, 0x{:02X}] to 0x{:02X}", mod2_payload[0], mod2_payload[1], self.address);
        self.i2c
            .write(self.address, &mod2_payload)
            .await
            .map_err(Error::I2c)?;
        #[cfg(feature = "defmt")]
        defmt::trace!("    wrote MOD2 OK");

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

        if self.power_mode != PowerMode::Fast && (!diag.pd3 || !diag.pd0) {
            return Err(Error::DataNotReady);
        }

        Ok(())
    }
}
