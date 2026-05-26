use core::fmt::Debug;

/// Sensor address slot for generation-2 devices (7-bit I2C address).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum AddressSlot {
    /// Address slot A0 (`0x35` in 7-bit form).
    A0,
    /// Address slot A1 (`0x22` in 7-bit form).
    A1,
    /// Address slot A2 (`0x78` in 7-bit form).
    A2,
    /// Address slot A3 (`0x44` in 7-bit form).
    A3,
}

impl AddressSlot {
    /// Returns this slot as a 7-bit I2C address.
    pub const fn as_7bit(self) -> u8 {
        match self {
            Self::A0 => 0x35,
            Self::A1 => 0x22,
            Self::A2 => 0x78,
            Self::A3 => 0x44,
        }
    }
}

/// Power mode selection for the `MODE` field in register `0x11`.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PowerMode {
    /// Low power mode.
    LowPower,
    /// Master controlled mode.
    MasterControlled,
    /// Fast conversion mode.
    Fast,
}

impl PowerMode {
    pub(crate) const fn bits(self) -> u8 {
        match self {
            Self::LowPower => 0b00,
            Self::MasterControlled => 0b01,
            Self::Fast => 0b11,
        }
    }
}

/// Measurement mode selection for `DT`/`AM` in register `0x10`.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum MeasurementMode {
    /// Magnetic field X/Y/Z and temperature.
    BxByBzTemp,
    /// Magnetic field X/Y/Z only.
    BxByBz,
    /// Magnetic field X/Y only.
    BxBy,
}

impl MeasurementMode {
    pub(crate) const fn dt_am_bits(self) -> (u8, u8) {
        match self {
            Self::BxByBzTemp => (0, 0),
            Self::BxByBz => (1, 0),
            Self::BxBy => (1, 1),
        }
    }
}

/// Trigger mode selection for `TRIG` in register `0x10`.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum TriggerMode {
    /// No ADC trigger on read.
    None,
    /// ADC trigger before the first MSB read.
    BeforeFirstMsb,
    /// ADC trigger after register `0x05`.
    AfterReg05,
}

impl TriggerMode {
    pub(crate) const fn bits(self) -> u8 {
        match self {
            Self::None => 0,
            Self::BeforeFirstMsb => 1,
            Self::AfterReg05 => 2,
        }
    }
}

/// Update rate bit for gen-2 fast/slow setting (`PRD`, register `0x13`, bit 7).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum UpdateRate {
    /// Fast update rate.
    Fast,
    /// Slow update rate.
    Slow,
}

impl UpdateRate {
    pub(crate) const fn bit(self) -> bool {
        matches!(self, Self::Slow)
    }
}

/// Sensitivity options for TLI493D-A2B6.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum A2B6Sensitivity {
    /// Full magnetic range (base sensitivity).
    Full,
    /// Short range (2x sensitivity scale factor).
    Short,
}

/// Sensitivity options for TLI493D-W2BW.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum W2BWSensitivity {
    /// Full magnetic range (base sensitivity).
    Full,
    /// Short range (2x sensitivity scale factor).
    Short,
    /// Extra short range (4x sensitivity scale factor).
    ExtraShort,
}

/// Raw values read from one data frame.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct RawReading {
    /// Raw magnetic field sample for X axis.
    pub x: i16,
    /// Raw magnetic field sample for Y axis.
    pub y: i16,
    /// Raw magnetic field sample for Z axis.
    pub z: i16,
    /// Raw temperature sample.
    pub temp: i16,
}

/// Engineering values from a frame.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Reading {
    /// X-axis magnetic field in millitesla.
    pub x_mt: f32,
    /// Y-axis magnetic field in millitesla.
    pub y_mt: f32,
    /// Z-axis magnetic field in millitesla.
    pub z_mt: f32,
    /// Temperature in degree Celsius.
    pub temp_c: f32,
}

/// Diagnostic flags parsed from the `DIAG` register (`0x06`).
///
/// The values reflect the most recent successful read operation.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Diagnostics {
    /// Bus/data parity bit from DIAG register.
    pub p: bool,
    /// Fuse parity validity indicator.
    pub ff: bool,
    /// Configuration parity validity indicator.
    pub cf: bool,
    /// Temperature status bit.
    pub t: bool,
    /// Power-down status bit 3.
    pub pd3: bool,
    /// Power-down status bit 0.
    pub pd0: bool,
    /// Frame counter (`FRM`) value (`0..=3`).
    pub frame: u8,
}

impl Diagnostics {
    pub(crate) const fn from_diag_byte(diag: u8) -> Self {
        Self {
            p: (diag & 0x80) != 0,
            ff: (diag & 0x40) != 0,
            cf: (diag & 0x20) != 0,
            t: (diag & 0x10) != 0,
            pd3: (diag & 0x08) != 0,
            pd0: (diag & 0x04) != 0,
            frame: diag & 0x03,
        }
    }
}

/// Driver error.
#[derive(Debug)]
pub enum Error<E: Debug> {
    /// I2C transaction error from the underlying bus implementation.
    I2c(E),
    /// Frame counter did not advance between consecutive samples.
    ///
    /// This indicates stalled conversion output and matches the driver's lockup
    /// detection strategy.
    AdcLockup,
    /// Bus parity bit did not match the received frame payload.
    InvalidBusParity,
    /// Fuse parity status bit indicates an invalid configuration state.
    InvalidFuseParity,
    /// Configuration parity status bit indicates invalid configuration.
    InvalidConfigurationParity,
    /// Temperature validity bit indicates invalid temperature data.
    InvalidTemperature,
    /// Data-ready indicator bits show no fresh sample.
    DataNotReady,
}
