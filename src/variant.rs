use crate::types::{A2B6Sensitivity, Reading, RawReading, W2BWSensitivity, XyReading, XyzReading};

/// Measurement shape contract for compile-time configuration of sensor output.
///
/// This trait enables type-state programming: different struct instantiations
/// (`Tli493d<I2c, V, M>`) generate different return types from `read()`.
pub trait MeasurementShape: Copy {
    /// The output type returned by [`Tli493d::read`].
    type Output;

    /// Value for CONFIG register bit 7 (DT: Disable Temperature).
    /// `0` = temperature measurement enabled, `1` = disabled.
    const DT: u8;

    /// Value for CONFIG register bit 6 (AM: X/Y Angular Measurement).
    /// `0` = Z measurement enabled, `1` = Z measurement disabled (with DT=1).
    const AM: u8;

    /// Decode raw values into engineering format appropriate for this measurement shape.
    fn decode(raw: RawReading, scale: f32) -> Self::Output;
}

/// Measurement shape: X, Y, Z, and temperature (default).
#[derive(Copy, Clone)]
pub struct BxByBzTemp;

impl MeasurementShape for BxByBzTemp {
    type Output = Reading;
    const DT: u8 = 0;
    const AM: u8 = 0;

    fn decode(raw: RawReading, scale: f32) -> Reading {
        let denom = 7.7 * scale;
        Reading {
            x_mt: raw.x as f32 / denom,
            y_mt: raw.y as f32 / denom,
            z_mt: raw.z as f32 / denom,
            temp_c: crate::register::temperature_to_c(raw.temp),
        }
    }
}

/// Measurement shape: X, Y, Z only (temperature disabled).
#[derive(Copy, Clone)]
pub struct BxByBz;

impl MeasurementShape for BxByBz {
    type Output = XyzReading;
    const DT: u8 = 1;
    const AM: u8 = 0;

    fn decode(raw: RawReading, scale: f32) -> XyzReading {
        let denom = 7.7 * scale;
        XyzReading {
            x_mt: raw.x as f32 / denom,
            y_mt: raw.y as f32 / denom,
            z_mt: raw.z as f32 / denom,
        }
    }
}

/// Measurement shape: X and Y only (temperature and Z disabled).
#[derive(Copy, Clone)]
pub struct BxBy;

impl MeasurementShape for BxBy {
    type Output = XyReading;
    const DT: u8 = 1;
    const AM: u8 = 1;

    fn decode(raw: RawReading, scale: f32) -> XyReading {
        let denom = 7.7 * scale;
        XyReading {
            x_mt: raw.x as f32 / denom,
            y_mt: raw.y as f32 / denom,
        }
    }
}

/// Variant-specific sensitivity argument contract.
pub trait VariantSensitivity: Copy {
    /// X2 bit state for CONFIG register.
    fn x2(self) -> bool;
    /// X4 bit state for CONFIG2 register.
    fn x4(self) -> bool;
    /// Scale multiplier relative to full-range sensitivity.
    fn scale(self) -> f32;
}

impl VariantSensitivity for A2B6Sensitivity {
    fn x2(self) -> bool {
        matches!(self, Self::Short)
    }

    fn x4(self) -> bool {
        false
    }

    fn scale(self) -> f32 {
        match self {
            Self::Full => 1.0,
            Self::Short => 2.0,
        }
    }
}

impl VariantSensitivity for W2BWSensitivity {
    fn x2(self) -> bool {
        !matches!(self, Self::Full)
    }

    fn x4(self) -> bool {
        matches!(self, Self::ExtraShort)
    }

    fn scale(self) -> f32 {
        match self {
            Self::Full => 1.0,
            Self::Short => 2.0,
            Self::ExtraShort => 4.0,
        }
    }
}

/// Compile-time description for a concrete sensor variant.
pub trait SensorVariant {
    /// Variant-specific sensitivity argument type used by
    /// [`crate::Tli493d::set_sensitivity`].
    type Sensitivity: VariantSensitivity;

    /// Whether the variant supports the X4 range bit.
    const HAS_X4: bool;
    /// Whether the I2C reset sequence (0x7F/0x00 toggling) is required.
    /// Gen-2 sensors (A2B6) use power-on reset instead.
    const SKIP_RESET_SEQUENCE: bool;
    /// Default value for CONFIG register (`0x10`).
    const RESET_CONFIG: u8;
    /// Default value for MOD1 register (`0x11`).
    const RESET_MOD1: u8;
    /// Default value for MOD2 register (`0x13`).
    const RESET_MOD2: u8;
    /// Default value for CONFIG2 register (`0x14`).
    const RESET_CONFIG2: u8;
}

/// Marker type for the TLI493D-A2B6 variant.
pub struct A2B6;

impl SensorVariant for A2B6 {
    type Sensitivity = A2B6Sensitivity;

    const HAS_X4: bool = false;
    const SKIP_RESET_SEQUENCE: bool = true;
    const RESET_CONFIG: u8 = 0x00;
    const RESET_MOD1: u8 = 0x00;
    const RESET_MOD2: u8 = 0x00;
    const RESET_CONFIG2: u8 = 0x00;
}

/// Marker type for the TLI493D-W2BW variant.
pub struct W2BW;

impl SensorVariant for W2BW {
    type Sensitivity = W2BWSensitivity;

    const HAS_X4: bool = true;
    const SKIP_RESET_SEQUENCE: bool = false;
    const RESET_CONFIG: u8 = 0x01;
    const RESET_MOD1: u8 = 0x80;
    const RESET_MOD2: u8 = 0x00;
    const RESET_CONFIG2: u8 = 0x00;
}
