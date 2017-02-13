//! # mio-serial - Serial port I/O for mio
//!
//! This crate provides a serial port implementation compatable with mio.
//!
//! **At this time this crate ONLY provides a unix implementation**
//!
//! ## Links
//!   - repo:  https://github.com/berkowski/mio-serial
//!   - docs:  https://docs.rs/mio-serial
#![deny(missing_docs)]

extern crate serialport;
extern crate mio;
extern crate libc;

#[cfg(unix)]
extern crate termios;

// Enums, Structs, and Traits from the serialport crate
pub use serialport::{// Traits
                     SerialPort,

                     // Structs
                     SerialPortInfo,
                     SerialPortSettings,

                     // Enums
                     DataBits,
                     StopBits,
                     Parity,
                     BaudRate,
                     FlowControl};

// The serialport Result type, used in SerialPort trait.
pub use serialport::Result as SerialResult;

// Some enumeration functions from the serialport crate
pub use serialport::{available_baud_rates, available_ports};

#[cfg(unix)]
mod unix;

#[cfg(windows)]
mod windows;

#[cfg(unix)]
pub use unix::Serial;

#[cfg(windows)]
pub use windows::Serial;
