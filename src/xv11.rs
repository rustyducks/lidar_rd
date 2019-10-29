use serial::prelude::*;
use std::io;
use std::io::Read;
use std::mem;
use std::sync::mpsc::{self, TryRecvError};
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::RwLock;
use std::thread;
use std::time::Duration;
use std::error::Error;

use crate::lidar::{Lidar, Sample};

pub struct XV11Iter<'a> {
    inner: &'a XV11,
}

pub struct XV11 {
    inner: Arc<RwLock<XV11Inner>>,
    tx: Option<mpsc::Sender<()>>,
    started: bool,
    join_handle: Option<thread::JoinHandle<()>>,
}

struct XV11Inner {
    scan: Mutex<Box<Option<Vec<Option<Sample>>>>>,
    lidar_speed: Mutex<f64>,
    port_path: String,
}

#[derive(Debug)]
enum InitLevel {
    Idle,
    Started,
    Reading,
}

impl<'a> Iterator for XV11Iter<'a> {
    type Item = Vec<Option<Sample>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.inner.started {
            loop {
                if let Some(scan) = self.inner.get_scan() {
                    return Some(scan);
                }
            }
        } else {
            None
        }
    }
}

impl XV11Inner {
    fn get_turn(&self) -> Option<Vec<Option<Sample>>> {
        let mut boxed_samples = self.scan.lock().unwrap();
        let bb = mem::replace(&mut *boxed_samples, Box::new(None));
        *bb
    }

    fn set_turn(&self, b: Box<Option<Vec<Option<Sample>>>>) {
        let mut boxed_samples = self.scan.lock().unwrap();
        *boxed_samples = b;
    }

    fn get_lidar_speed(&self) -> f64 {
        *self.lidar_speed.lock().unwrap()
    }

    fn set_lidar_speed(&self, speed: f64) {
        *self.lidar_speed.lock().unwrap() = speed;
    }

    fn read_xv11(&self, f: &mut dyn Read, rx: mpsc::Receiver<()>) -> io::Result<()> {
        let mut buffer = [0; 22];
        let mut init_level = InitLevel::Idle;
        let mut angle_max: f64 = 0.0;
        let mut turn = vec![];

        loop {
            match init_level {
                InitLevel::Idle => {
                    f.read(&mut buffer[0..1])?;
                    if buffer[0] == 0xFA {
                        init_level = InitLevel::Started;
                    }
                }

                InitLevel::Started => {
                    f.read(&mut buffer[1..2])?;
                    if buffer[1] >= 0xA0 && buffer[1] <= 0xF9 {
                        init_level = InitLevel::Reading;
                    } else {
                        init_level = InitLevel::Idle;
                    }
                }

                InitLevel::Reading => {
                    f.read_exact(&mut buffer[2..])?;
                    init_level = InitLevel::Idle;
                    let (speed, mut samples) = decode_packet(buffer);

                    if let Some(speed) = speed {
                        self.set_lidar_speed(speed);
                    }

                    if let Some((a_min, a_max)) = get_min_max(&samples) {
                        if a_min < angle_max {
                            // new turn !
                            angle_max = a_max;
                            self.set_turn(Box::new(Some(turn)));
                            turn = vec![];
                        }
                        angle_max = angle_max.max(a_max);
                    }

                    turn.append(&mut samples);
                }
            }

            match rx.try_recv() {
                Ok(_) | Err(TryRecvError::Disconnected) => {
                    println!("Terminating.");
                    break Ok(());
                }
                Err(TryRecvError::Empty) => {}
            }
        }
    }
}

impl XV11 {
    pub fn new(port_path: &str) -> XV11 {
        XV11 {
            inner: Arc::new(RwLock::new(XV11Inner {
                port_path: port_path.to_string(),
                scan: Mutex::new(Box::new(None)),
                lidar_speed: Mutex::new(0.0),
            })),
            tx: None,
            started: false,
            join_handle: None,
        }
    }

    pub fn iter<'a>(&'a self) -> XV11Iter<'a> {
        XV11Iter { inner: &self }
    }

    pub fn get_lidar_speed(&self) -> f64 {
        self.inner.clone().read().unwrap().get_lidar_speed()
    }
}

impl Lidar for XV11 {
    fn get_scan(&self) -> Option<Vec<Option<Sample>>> {
        self.inner.clone().read().unwrap().get_turn()
    }

    fn start(&mut self) -> Result<(), Box<dyn Error>> {
        let local_self = self.inner.clone();

        let (tx, rx) = mpsc::channel();

        self.tx = Some(tx.clone());

        let mut port = serial::open(&local_self.read().unwrap().port_path).unwrap();

        port.reconfigure(&|settings| {
            settings.set_baud_rate(serial::Baud115200)?;
            settings.set_char_size(serial::Bits8);
            settings.set_parity(serial::ParityNone);
            settings.set_stop_bits(serial::Stop1);
            settings.set_flow_control(serial::FlowNone);
            Ok(())
        })
        .unwrap();

        port.set_timeout(Duration::from_millis(500)).unwrap();

        self.join_handle = Some(thread::spawn(move || {
            local_self
                .read()
                .unwrap()
                .read_xv11(&mut port, rx)
                .expect("XV11 reading failed !");
            println!("Stopped !");
        }));

        self.started = true;

        Ok(())
    }

    fn stop(&mut self) {
        if let Some(tx) = &self.tx {
            let _ = tx.send(());
        }
        if let Some(handle) = self.join_handle.take() {
            handle.join().expect("failed to join thread");
            self.join_handle = None;
        }
    }
}

fn get_min_max(samples: &Vec<Option<Sample>>) -> Option<(f64, f64)> {
    let mut mimax: Option<(f64, f64)> = None;
    for sample in samples {
        if let Some(sample) = sample {
            mimax = match mimax {
                Some((min, max)) => Some((min.min(sample.angle), max.max(sample.angle))),
                None => Some((sample.angle, sample.angle)),
            };
        }
    }
    mimax
}

fn decode_packet(buffer: [u8; 22]) -> (Option<f64>, Vec<Option<Sample>>) {
    let computed_chk = checksum(&buffer[0..20]);
    let read_chk = (buffer[20] as u16) | ((buffer[21] as u16) << 8);

    if computed_chk == read_chk {
        let index = buffer[1] - 0xA0;
        let speed_buf = &buffer[2..4];
        let speed = (speed_buf[0] as u16 | ((speed_buf[1] as u16) << 8)) as f64 / 64.0;

        let base_angle = index as usize * 4;

        let samples = (0..4)
            .map(|i| {
                let data = &buffer[4 * i..4 * (i + 1)];
                decode_data(base_angle + i, data)
            })
            .collect();

        (Some(speed), samples)
    } else {
        (None, vec![None, None, None, None])
    }
}

fn decode_data(angle: usize, data: &[u8]) -> Option<Sample> {
    let distance = (((0b00111111 & data[1]) as u16) << 8) | data[0] as u16;
    let invalid = (data[1] & 0b10000000) != 0;
    let warning = (data[1] & 0b01000000) != 0;
    let strength = ((data[3] as u16) << 8) | data[2] as u16;

    let angle = ((angle as f64) - 180.0).to_radians();

    if !invalid && !warning {
        Some(Sample {
            angle,
            distance,
            quality: strength,
        })
    } else {
        None
    }
}

fn checksum(buffer: &[u8]) -> u16 {
    let mut chk32: u32 = 0;
    for chunck in buffer.chunks(2) {
        let d: u32 = chunck[0] as u32 + ((chunck[1] as u32) << 8);
        chk32 = (chk32 << 1) + d;
    }

    let mut checksum: u32 = (chk32 & 0x7FFF) + (chk32 >> 15);
    checksum = checksum & 0x7FFF;

    checksum as u16
}
