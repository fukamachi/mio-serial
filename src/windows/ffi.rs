#![allow(non_snake_case)]
#![allow(dead_code)]
extern crate libc;

use std::os;

pub type BOOL = libc::c_int;
pub type DWORD = libc::c_ulong;
pub type LPDWORD = *mut DWORD;
pub type LPVOID = *mut libc::c_void;

pub type HANDLE = *mut LPVOID;

pub const EV_RXCHAR: DWORD = 0x0001;
pub const EV_RXFLAG: DWORD = 0x0002;
pub const EV_TXEMPTY: DWORD = 0x0004;
pub const EV_CTS: DWORD = 0x0008;
pub const EV_DSR: DWORD = 0x0010;
pub const EV_RLSD: DWORD = 0x0020;
pub const EV_BREAK: DWORD = 0x0040;
pub const EV_ERR: DWORD = 0x0080;
pub const EV_RING: DWORD = 0x0100;

#[repr(C)]
pub struct OVERLAPPED {
    pub Internal: *mut libc::c_ulong,
    pub InternalHigh: *mut libc::c_ulong,
    pub Offset: DWORD,
    pub OffsetHigh: DWORD,
    pub hEvent: HANDLE,
}

pub type LPOVERLAPPED = *mut OVERLAPPED;

extern "system" {
    pub fn WaitCommEvent(hfile: HANDLE,
                         lpEvtMask: LPDWORD,
                         lpOverlapped: LPOVERLAPPED) -> BOOL;
}
