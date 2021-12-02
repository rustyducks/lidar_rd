use std::fmt;
use std::error::Error;

#[derive(Copy, Clone)]
pub struct Sample {
    pub angle: f64,
    pub distance: u16,
    pub quality: u16,
}

pub struct Turn {
    pub samples: Vec<Option<Sample>>
}

impl fmt::Display for Sample {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.2}:{},{}", self.angle, self.distance, self.quality)
    }
}

impl fmt::Debug for Sample {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "angle:{:.2}, dist:{}, q:{}", self.angle, self.distance, self.quality)
    }
}

impl Turn {
    pub fn new() -> Turn{
        Turn {
            //samples: Vec::new(),
            samples: vec![],
        }
    }

    pub fn last_angle(&self) -> f64 {
        match self.samples.last() {
            Some(s) => s.unwrap().angle,
            None => 0.0,
        }
    }

    pub fn push(&mut self, s: Sample) {
        self.samples.push(Some(s));
    }
}

impl fmt::Display for Turn {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "turn")
    }
}



pub trait Lidar {
    fn get_scan(&self) -> Option<Vec<Option<Sample>>>;
    fn start(&mut self) -> Result<(), Box<dyn Error>>;
    fn stop(&mut self);
    fn is_running(&self) -> bool;
}


macro_rules! impl_iterator {
    ($t:ty) => (
        impl Iterator for $t {
            type Item = Vec<Option<Sample>>;

            fn next(&mut self) -> Option<Self::Item> {
                if self.is_running() {
                    loop {
                        if let Some(scan) = self.get_scan() {
                            return Some(scan);
                        }
                    }
                } else {
                    None
                }
            }
        }
    )
}


macro_rules! impl_drop {
    ($t:ty) => (
        impl Drop for $t {
            fn drop(&mut self) {
                self.stop();
            }
        }
    )
}


pub(crate) use impl_iterator;
pub(crate) use impl_drop;
