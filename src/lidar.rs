use std::fmt;
use std::error::Error;

#[derive(Copy, Clone)]
pub struct Sample {
    pub angle: f64,
    pub distance: u16,
    pub quality: u16,
}

impl fmt::Display for Sample {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{},{}", self.angle, self.distance, self.quality)
    }
}

pub trait Lidar {
    fn get_scan(&self) -> Option<Vec<Option<Sample>>>;
    fn start(&mut self) -> Result<(), Box<dyn Error>>;
    fn stop(&mut self);
}
