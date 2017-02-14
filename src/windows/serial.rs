extern crate lazycell;
extern crate core;

use std::os::windows::prelude::*;
use std::io;
use std::path::Path;
use std::convert::AsRef;
use std::time::Duration;
use std::thread;
use std::ptr;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicUsize, Ordering};

use mio::{Evented, PollOpt, Token, Poll, Ready, Registration, SetReadiness};

use serialport;
use serialport::windows::COMPort;
use serialport::prelude::*;
use self::lazycell::{LazyCell, AtomicLazyCell};

use super::ffi::*;

struct SerialCtl {
    registration: LazyCell<Registration>,
    set_readiness: AtomicLazyCell<SetReadiness>,
    pending: AtomicUsize,
}

/// Windows serial port
pub struct Serial {
    inner: Arc<Mutex<serialport::windows::COMPort>>,
    ctl: Arc<SerialCtl>,
}

unsafe impl core::marker::Send for Serial { }

impl Serial {
    /// Open a nonblocking serial port from the provided path.
    pub fn from_path<T: AsRef<Path>>(path: T, settings: &SerialPortSettings) -> io::Result<Self> {
        let port = COMPort::open(path.as_ref(), settings)?;
        Serial::from_serial(port)
    }

    /// Convert an existing `serialport::windows::COMPort` struct.
    pub fn from_serial(port: COMPort) -> io::Result<Self> {
        let ctl = SerialCtl {
            registration: LazyCell::new(),
            set_readiness: AtomicLazyCell::new(),
            pending: AtomicUsize::new(0),
        };

        let serial = Serial {
            inner: Arc::new(Mutex::new(port)),
            ctl: Arc::new(ctl),
        };

        {
            let serial = serial.clone();
            thread::spawn(move || {
                let evt = &mut EV_RXCHAR;
                loop {
                    let raw_handle = {
                        let inner = serial.inner.lock().unwrap();
                        inner.as_raw_handle()
                    };
                    let res = unsafe { WaitCommEvent(raw_handle, evt, ptr::null_mut()) };
                    match res as u32 {
                        EV_RXCHAR => serial.ctl.make_readable().unwrap(),
                        _ => unreachable!()
                    }
                }
            });
        }

        Ok(serial)
    }
}

impl SerialCtl {
    fn make_readable(&self) -> io::Result<()> {
        let cnt = self.pending.fetch_add(1, Ordering::Acquire);

        if cnt == 0 {
            if let Some(set_readiness) = self.set_readiness.borrow() {
                try!(set_readiness.set_readiness(Ready::readable()));
            }
        }

        Ok(())
    }
}

impl Clone for Serial {
    fn clone(&self) -> Self {
        Serial {
            inner: self.inner.clone(),
            ctl: self.ctl.clone()
        }
    }
}

impl SerialPort for Serial {
    /// Returns a struct with the current port settings
    fn settings(&self) -> SerialPortSettings {
        self.inner.lock().unwrap().settings()
    }

    /// Return the name associated with the serial port, if known.
    fn port_name(&self) -> Option<String> {
        self.inner.lock().unwrap().port_name()
    }

    /// Returns the current baud rate.
    ///
    /// This function returns `None` if the baud rate could not be determined. This may occur if
    /// the hardware is in an uninitialized state. Setting a baud rate with `set_baud_rate()`
    /// should initialize the baud rate to a supported value.
    fn baud_rate(&self) -> Option<::BaudRate> {
        self.inner.lock().unwrap().baud_rate()
    }

    /// Returns the character size.
    ///
    /// This function returns `None` if the character size could not be determined. This may occur
    /// if the hardware is in an uninitialized state or is using a non-standard character size.
    /// Setting a baud rate with `set_char_size()` should initialize the character size to a
    /// supported value.
    fn data_bits(&self) -> Option<::DataBits> {
        self.inner.lock().unwrap().data_bits()
    }

    /// Returns the flow control mode.
    ///
    /// This function returns `None` if the flow control mode could not be determined. This may
    /// occur if the hardware is in an uninitialized state or is using an unsupported flow control
    /// mode. Setting a flow control mode with `set_flow_control()` should initialize the flow
    /// control mode to a supported value.
    fn flow_control(&self) -> Option<::FlowControl> {
        self.inner.lock().unwrap().flow_control()
    }

    /// Returns the parity-checking mode.
    ///
    /// This function returns `None` if the parity mode could not be determined. This may occur if
    /// the hardware is in an uninitialized state or is using a non-standard parity mode. Setting
    /// a parity mode with `set_parity()` should initialize the parity mode to a supported value.
    fn parity(&self) -> Option<::Parity> {
        self.inner.lock().unwrap().parity()
    }

    /// Returns the number of stop bits.
    ///
    /// This function returns `None` if the number of stop bits could not be determined. This may
    /// occur if the hardware is in an uninitialized state or is using an unsupported stop bit
    /// configuration. Setting the number of stop bits with `set_stop-bits()` should initialize the
    /// stop bits to a supported value.
    fn stop_bits(&self) -> Option<::StopBits> {
        self.inner.lock().unwrap().stop_bits()
    }

    /// Returns the current timeout.
    fn timeout(&self) -> Duration {
        Duration::from_secs(0)
    }

    // Port settings setters

    /// Applies all settings for a struct. This isn't guaranteed to involve only
    /// a single call into the driver, though that may be done on some
    /// platforms.
    fn set_all(&mut self, settings: &SerialPortSettings) -> serialport::Result<()> {
        self.inner.lock().unwrap().set_all(settings)
    }

    /// Sets the baud rate.
    ///
    /// ## Errors
    ///
    /// If the implementation does not support the requested baud rate, this function may return an
    /// `InvalidInput` error. Even if the baud rate is accepted by `set_baud_rate()`, it may not be
    /// supported by the underlying hardware.
    fn set_baud_rate(&mut self, baud_rate: ::BaudRate) -> serialport::Result<()> {
        self.inner.lock().unwrap().set_baud_rate(baud_rate)
    }

    /// Sets the character size.
    fn set_data_bits(&mut self, data_bits: ::DataBits) -> serialport::Result<()> {
        self.inner.lock().unwrap().set_data_bits(data_bits)
    }

    /// Sets the flow control mode.
    fn set_flow_control(&mut self, flow_control: ::FlowControl) -> serialport::Result<()> {
        self.inner.lock().unwrap().set_flow_control(flow_control)
    }

    /// Sets the parity-checking mode.
    fn set_parity(&mut self, parity: ::Parity) -> serialport::Result<()> {
        self.inner.lock().unwrap().set_parity(parity)
    }

    /// Sets the number of stop bits.
    fn set_stop_bits(&mut self, stop_bits: ::StopBits) -> serialport::Result<()> {
        self.inner.lock().unwrap().set_stop_bits(stop_bits)
    }

    /// Sets the timeout for future I/O operations.  This parameter is ignored but
    /// required for trait completeness.
    fn set_timeout(&mut self, _: Duration) -> serialport::Result<()> {
        Ok(())
    }

    // Functions for setting non-data control signal pins

    /// Sets the state of the RTS (Request To Send) control signal.
    ///
    /// Setting a value of `true` asserts the RTS control signal. `false` clears the signal.
    ///
    /// ## Errors
    ///
    /// This function returns an error if the RTS control signal could not be set to the desired
    /// state on the underlying hardware:
    ///
    /// * `NoDevice` if the device was disconnected.
    /// * `Io` for any other type of I/O error.
    fn write_request_to_send(&mut self, level: bool) -> serialport::Result<()> {
        self.inner.lock().unwrap().write_request_to_send(level)
    }

    /// Writes to the Data Terminal Ready pin
    ///
    /// Setting a value of `true` asserts the DTR control signal. `false` clears the signal.
    ///
    /// ## Errors
    ///
    /// This function returns an error if the DTR control signal could not be set to the desired
    /// state on the underlying hardware:
    ///
    /// * `NoDevice` if the device was disconnected.
    /// * `Io` for any other type of I/O error.
    fn write_data_terminal_ready(&mut self, level: bool) -> serialport::Result<()> {
        self.inner.lock().unwrap().write_data_terminal_ready(level)
    }

    // Functions for reading additional pins

    /// Reads the state of the CTS (Clear To Send) control signal.
    ///
    /// This function returns a boolean that indicates whether the CTS control signal is asserted.
    ///
    /// ## Errors
    ///
    /// This function returns an error if the state of the CTS control signal could not be read
    /// from the underlying hardware:
    ///
    /// * `NoDevice` if the device was disconnected.
    /// * `Io` for any other type of I/O error.
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        self.inner.lock().unwrap().read_clear_to_send()
    }

    /// Reads the state of the Data Set Ready control signal.
    ///
    /// This function returns a boolean that indicates whether the DSR control signal is asserted.
    ///
    /// ## Errors
    ///
    /// This function returns an error if the state of the DSR control signal could not be read
    /// from the underlying hardware:
    ///
    /// * `NoDevice` if the device was disconnected.
    /// * `Io` for any other type of I/O error.
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        self.inner.lock().unwrap().read_data_set_ready()
    }

    /// Reads the state of the Ring Indicator control signal.
    ///
    /// This function returns a boolean that indicates whether the RI control signal is asserted.
    ///
    /// ## Errors
    ///
    /// This function returns an error if the state of the RI control signal could not be read from
    /// the underlying hardware:
    ///
    /// * `NoDevice` if the device was disconnected.
    /// * `Io` for any other type of I/O error.
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        self.inner.lock().unwrap().read_ring_indicator()
    }

    /// Reads the state of the Carrier Detect control signal.
    ///
    /// This function returns a boolean that indicates whether the CD control signal is asserted.
    ///
    /// ## Errors
    ///
    /// This function returns an error if the state of the CD control signal could not be read from
    /// the underlying hardware:
    ///
    /// * `NoDevice` if the device was disconnected.
    /// * `Io` for any other type of I/O error.
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        self.inner.lock().unwrap().read_carrier_detect()
    }
}

impl io::Read for Serial {
    fn read(&mut self, bytes: &mut [u8]) -> io::Result<usize> {
        let res = self.inner.lock().unwrap().read(bytes);
        if let Some(set_readiness) = self.ctl.set_readiness.borrow() {
            try!(set_readiness.set_readiness(Ready::none()));
        }
        res
    }
}

impl io::Write for Serial {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        self.inner.lock().unwrap().write(bytes)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.inner.lock().unwrap().flush()
    }
}

impl AsRawHandle for Serial {
    fn as_raw_handle(&self) -> RawHandle {
        self.inner.lock().unwrap().as_raw_handle()
    }
}

impl Evented for Serial {
    fn register(&self,
                poll: &Poll,
                token: Token,
                interest: Ready,
                opts: PollOpt)
                -> io::Result<()> {
        self.ctl.register(poll, token, interest, opts)
    }

    fn reregister(&self,
                  poll: &Poll,
                  token: Token,
                  interest: Ready,
                  opts: PollOpt)
                  -> io::Result<()> {
        self.ctl.reregister(poll, token, interest, opts)
    }

    fn deregister(&self, poll: &Poll) -> io::Result<()> {
        self.ctl.deregister(poll)
    }
}

impl Evented for SerialCtl {
    fn register(&self, poll: &Poll, token: Token, interest: Ready, opts: PollOpt) -> io::Result<()> {
        if self.registration.borrow().is_some() {
            return Err(io::Error::new(io::ErrorKind::Other, "receiver already registered"));
        }

        let (registration, set_readiness) = Registration::new(poll, token, interest, opts);

        if self.pending.load(Ordering::Relaxed) > 0 {
            // It's already readable
            let _ = set_readiness.set_readiness(Ready::readable());
        }

        self.registration.fill(registration).ok().expect("unexpected state encountered");
        self.set_readiness.fill(set_readiness).ok().expect("unexpected state encountered");

        Ok(())
    }

    fn reregister(&self, poll: &Poll, token: Token, interest: Ready, opts: PollOpt) -> io::Result<()> {
        match self.registration.borrow() {
            Some(registration) => registration.update(poll, token, interest, opts),
            None => Err(io::Error::new(io::ErrorKind::Other, "receiver not registered")),
        }
    }

    fn deregister(&self, poll: &Poll) -> io::Result<()> {
        match self.registration.borrow() {
            Some(registration) => registration.deregister(poll),
            None => Err(io::Error::new(io::ErrorKind::Other, "receiver not registered")),
        }
    }
}
