use core::fmt::Debug;
use core::marker::PhantomData;

use embedded_hal_async::i2c;

use crate::register::{
    REG_CACHE_LEN, REG_CONFIG, REG_CONFIG2, REG_MOD1, REG_MOD2, decode_data_frame, set_bit,
    set_bits, set_fuse_parity, temperature_to_c,
};
use crate::types::{Diagnostics, Error, PowerMode, RawReading, Reading, UpdateRate};
use crate::variant::{SensorVariant, VariantSensitivity};

const BASE_SENSITIVITY_MT_PER_LSB: f32 = 7.7;

/// Asynchronous TLI493D driver.
///
/// `I2c` is any async I2C implementation accepted by `embedded-hal-async`
/// (for example a peripheral instance or a shared-bus proxy type). `V` is the
/// compile-time sensor variant marker.
pub struct Tli493d<I2c, V> {
    i2c: I2c,
    address: u8,
    sensitivity_scale: f32,
    reg_cache: [u8; REG_CACHE_LEN],
    last_frame: Option<u8>,
    last_diag: Diagnostics,
    _variant: PhantomData<V>,
}

impl<I2c, V> Tli493d<I2c, V>
where
    I2c: i2c::I2c,
    <I2c as i2c::ErrorType>::Error: Debug,
    V: SensorVariant,
{
    /// Creates and initializes a sensor instance.
    ///
    /// The caller provides the target 7-bit I2C address and desired initial
    /// power mode. The function performs a best-effort reset sequence, reads an
    /// initial register window, then applies variant defaults.
    ///
    /// # Notes
    ///
    /// No startup delay is performed internally. Ensure the sensor is powered
    /// and stable before calling this constructor.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] when initialization bus transactions fail.
    pub async fn new(
        i2c: I2c,
        address: u8,
        power_mode: PowerMode,
    ) -> Result<Self, Error<<I2c as i2c::ErrorType>::Error>> {
        let mut this = Self {
            i2c,
            address,
            sensitivity_scale: 1.0,
            reg_cache: [0u8; REG_CACHE_LEN],
            last_frame: None,
            last_diag: Diagnostics::from_diag_byte(0),
            _variant: PhantomData,
        };

        // Reset writes are best-effort by design; some systems may already be in
        // a valid configured state before the driver is created.
        let _ = this.i2c.write(0x7f, &[]).await;
        let _ = this.i2c.write(0x00, &[0xff]).await;

        this.read_register_window().await?;
        this.apply_reset_defaults();
        this.set_power_mode(power_mode).await?;

        Ok(this)
    }

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
    /// Returns [`Error::AdcLockup`] if the frame counter did not change between
    /// consecutive reads.
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
    /// Magnetic field output is given in millitesla and temperature in degree
    /// Celsius.
    ///
    /// # Errors
    ///
    /// Propagates errors from [`Self::read_raw`].
    pub async fn read(&mut self) -> Result<Reading, Error<<I2c as i2c::ErrorType>::Error>> {
        let raw = self.read_raw().await?;
        let denom = BASE_SENSITIVITY_MT_PER_LSB * self.sensitivity_scale;

        Ok(Reading {
            x_mt: raw.x as f32 / denom,
            y_mt: raw.y as f32 / denom,
            z_mt: raw.z as f32 / denom,
            temp_c: temperature_to_c(raw.temp),
        })
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

    /// Configures power mode via `MODE` bits in `MOD1`.
    ///
    /// This updates cached configuration and writes the `MOD1` register triplet
    /// required by generation-2 devices.
    ///
    /// # Errors
    ///
    /// Returns [`Error::I2c`] when writing mode registers fails.
    pub async fn set_power_mode(
        &mut self,
        mode: PowerMode,
    ) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        self.reg_cache[REG_MOD1] = set_bits(self.reg_cache[REG_MOD1], 0x03, 0, mode.bits());
        self.reg_cache[REG_MOD1] = set_fuse_parity(self.reg_cache[REG_MOD1], self.reg_cache[REG_MOD2]);

        self.write_mod1_triplet().await
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
        self.reg_cache[REG_MOD2] = set_bit(self.reg_cache[REG_MOD2], 0x80, rate.bit());
        self.reg_cache[REG_MOD1] = set_fuse_parity(self.reg_cache[REG_MOD1], self.reg_cache[REG_MOD2]);

        self.write_mod1_triplet().await
    }

    async fn read_register_window(&mut self) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        self.i2c
            .read(self.address, &mut self.reg_cache)
            .await
            .map_err(Error::I2c)
    }

    fn apply_reset_defaults(&mut self) {
        self.reg_cache[REG_CONFIG] = V::RESET_CONFIG;
        self.reg_cache[REG_MOD1] = V::RESET_MOD1;
        self.reg_cache[REG_MOD2] = V::RESET_MOD2;
        if V::HAS_X4 {
            self.reg_cache[REG_CONFIG2] = V::RESET_CONFIG2;
        }
    }

    async fn write_mod1_triplet(&mut self) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        let payload = [
            REG_MOD1 as u8,
            self.reg_cache[REG_MOD1],
            self.reg_cache[REG_MOD1 + 1],
            self.reg_cache[REG_MOD2],
        ];

        self.i2c
            .write(self.address, &payload)
            .await
            .map_err(Error::I2c)
    }

    async fn write_config_registers(&mut self) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        let mut payload = [0u8; 6];
        payload[0] = REG_CONFIG as u8;
        payload[1] = self.reg_cache[REG_CONFIG];
        payload[2] = self.reg_cache[REG_CONFIG + 1];
        payload[3] = self.reg_cache[REG_CONFIG + 2];
        payload[4] = self.reg_cache[REG_CONFIG + 3];
        payload[5] = self.reg_cache[REG_CONFIG2];

        self.i2c
            .write(self.address, &payload)
            .await
            .map_err(Error::I2c)
    }

    async fn set_sensitivity_inner(
        &mut self,
        sensitivity: V::Sensitivity,
    ) -> Result<(), Error<<I2c as i2c::ErrorType>::Error>> {
        let x2 = sensitivity.x2();
        let x4 = sensitivity.x4();

        self.reg_cache[REG_CONFIG] = set_bit(self.reg_cache[REG_CONFIG], 0x08, x2);
        if V::HAS_X4 {
            self.reg_cache[REG_CONFIG2] = set_bit(self.reg_cache[REG_CONFIG2], 0x01, x4);
        }

        self.write_config_registers().await?;
        self.sensitivity_scale = sensitivity.scale();
        Ok(())
    }
}
