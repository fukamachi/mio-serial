//! Simple example that echos recevied serial traffic to stdout
extern crate mio;
extern crate mio_serial;

use mio::{Poll, PollOpt, Events, Token, Ready};
use std::time::Duration;
use std::str;
use std::io::Read;
use std::env;

const SERIAL_TOKEN: Token = Token(0);

pub fn main() {

    let mut args = env::args();
    let tty_path = args.nth(1).unwrap_or("/dev/ttyUSB0".into());

    let poll = Poll::new().unwrap();
    let mut events = Events::with_capacity(1024);

    // Create the listener
    let settings = mio_serial::SerialPortSettings::default();

    println!("Opening {} at 9600,8N1", tty_path);
    let mut rx = mio_serial::Serial::from_path(&tty_path, &settings).unwrap();

    // Disable exclusive mode
    rx.set_exclusive(false).expect("Unable to set serial port into non-exclusive mode.");

    poll.register(&rx, SERIAL_TOKEN, Ready::readable(), PollOpt::level()).unwrap();

    let mut rx_buf = [0u8; 1024];

    loop {
        poll.poll(&mut events, Some(Duration::from_secs(1))).unwrap();

        if events.len() == 0 {
            println!("Read timed out!");
            continue;
        }

        for event in events.iter() {
            let bytes_read = match event.token() {
                SERIAL_TOKEN => rx.read(&mut rx_buf),
                _ => unreachable!(),
            };

            match bytes_read {
                Ok(b) => {
                    match b {
                        b if b > 0 => println!("{:?}", String::from_utf8_lossy(&rx_buf[..b])),
                        _ => println!("Read would have blocked."),
                    }
                }
                Err(e) => println!("Error:  {}", e),
            }
        }
    }
}
