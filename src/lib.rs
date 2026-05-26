#![no_std]

//! Async `no_std` driver for TLI493D generation-2 devices.
//!
//! The crate is built on `embedded-hal-async` and designed for Embassy-style
//! execution.
//!
//! The I2C object is provided to the driver constructor. This can be either:
//! - the I2C peripheral itself, or
//! - a shared-bus device/proxy wrapper.
//!
//! Sensor startup timing is handled externally; no internal startup delay is
//! inserted by the driver.
//!
//! # Variants
//!
//! This crate exposes compile-time variant markers and type aliases:
//! - [`Tli493dA2b6`] for TLI493D-A2B6
//! - [`Tli493dW2BW`] for TLI493D-W2BW

mod driver;
mod register;
mod types;
mod variant;

pub use driver::Tli493d;
/// Marker type for TLI493D-A2B6.
pub use variant::A2B6;
/// Marker type for TLI493D-W2BW.
pub use variant::W2BW;
pub use types::{
    A2B6Sensitivity, AddressSlot, Diagnostics, Error, PowerMode, RawReading, Reading,
    UpdateRate, W2BWSensitivity,
};

/// Type alias for a TLI493D-A2B6 device.
pub type Tli493dA2b6<I2c> = Tli493d<I2c, A2B6>;

/// Type alias for a TLI493D-W2BW device.
pub type Tli493dW2BW<I2c> = Tli493d<I2c, W2BW>;
