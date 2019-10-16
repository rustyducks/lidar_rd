#[macro_use]
extern crate lazy_static;

pub mod lidar;
pub mod xv11;

pub use crate::lidar::{Sample, Lidar};

pub use crate::xv11::XV11;