use crate::types::{A2B6Sensitivity, W2BWSensitivity};

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
    const RESET_CONFIG: u8 = 0x01;
    const RESET_MOD1: u8 = 0x80;
    const RESET_MOD2: u8 = 0x00;
    const RESET_CONFIG2: u8 = 0x00;
}
