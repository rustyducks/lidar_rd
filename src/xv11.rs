
use std::io;
use std::io::Read;
use std::sync::Mutex;
use std::sync::Arc;
use std::sync::RwLock;
use std::mem;
use std::thread;
use std::sync::mpsc::{self, TryRecvError};
use serial::prelude::*;
use std::time::Duration;

use crate::lidar::{Sample, Lidar};

pub struct XV11Iter<'a> {
    inner: &'a XV11
}

pub struct XV11 {
    inner: Arc<RwLock<XV11Inner>>,
    tx: Option<mpsc::Sender<()>>,
    }

struct XV11Inner {
    
    port_path: String,
}

#[derive(Debug)]
enum InitLevel {
    Idle,
    Started,
    Reading
}



lazy_static! {
    static ref TURN: Mutex<Box<Option<Vec<Sample>>>> = Mutex::new(Box::new(None));
}

lazy_static! {
    static ref LIDAR_SPEED: Mutex<f64> = Mutex::new(0.0);
}




impl<'a> Iterator for XV11Iter<'a> {

    type Item = Vec<Sample>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if let Some(scan) = self.inner.get_scan() {
                return Some(scan);
            }
        }

    }

}



impl XV11Inner {

    fn get_turn(&self) -> Option<Vec<Sample>> {
        let mut boxed_samples = TURN.lock().unwrap();
        let bb = mem::replace(&mut *boxed_samples, Box::new(None));
        *bb
    }

    fn set_turn(&self, b: Box<Option<Vec<Sample>>>) {
        let mut boxed_samples = TURN.lock().unwrap();
        *boxed_samples = b;
    }

    fn get_lidar_speed(&self) -> f64 {
        *LIDAR_SPEED.lock().unwrap()
    }

    fn set_lidar_speed(&self, speed: f64) {
        *LIDAR_SPEED.lock().unwrap() = speed;
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
                },

                InitLevel::Started => {
                    f.read(&mut buffer[1..2])?;
                    if buffer[1]>= 0xA0 && buffer[1] <= 0xF9 {
                        init_level = InitLevel::Reading;
                    } else {
                        init_level = InitLevel::Idle;
                    }
                },

                InitLevel::Reading => {
                    f.read_exact(&mut buffer[2..])?;
                    init_level = InitLevel::Idle;
                    let (speed, mut samples) = decode_packet(buffer);
                    let (a_min, a_max) = get_min_max(&samples);
                    
                    if let Some(speed) = speed {
                        self.set_lidar_speed(speed);
                    }
                    
                    if a_min < angle_max {  // new turn !
                        angle_max = a_max;
                        self.set_turn(Box::new(Some(turn)));
                        turn = vec![];
                        
                    }
                    
                    angle_max = angle_max.max(a_max);

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
            })),
            tx: None,
        }
    }

    pub fn iter<'a>(&'a self) -> XV11Iter<'a> {
        XV11Iter {
            inner: &self
        }
    }

    pub fn get_lidar_speed(&self) -> f64 {
        self.inner.clone().read().unwrap().get_lidar_speed()
    }

}

impl Lidar for XV11 {
    fn get_scan(&self) -> Option<Vec<Sample>> {
        //let local_self = ;
        self.inner.clone().read().unwrap().get_turn()
    }

    fn start(&mut self) 
    {
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
        }).unwrap();

        port.set_timeout(Duration::from_millis(500)).unwrap();

        thread::spawn(move || {
            local_self.read().unwrap().read_xv11(&mut port, rx).expect("XV11 reading failed !");
            //read_xv11(&mut f, rx).expect("XV11 reading failed !");
            println!("Stopped !");
        });
    }

    fn stop(&self) {
        if let Some(tx) = &self.tx {
            let _ = tx.send(());
        }
    }
}



fn get_min_max(samples: &Vec<Sample>) -> (f64, f64) {
    let (mut min, mut max): (f64, f64) = (360.0, 0.0);
    for sample in samples {
        min = min.min(sample.angle);
        max = min.max(sample.angle)
    }
    (min, max)
}


fn decode_packet(buffer: [u8; 22]) -> (Option<f64>, Vec<Sample>) {
    let computed_chk = checksum(&buffer[0..20]);
    let read_chk = (buffer[20] as u16) | ((buffer[21] as u16) << 8);

    if computed_chk == read_chk {
        let index = buffer[1] - 0xA0;
        let speed_buf = &buffer[2..4];
        let speed = ((speed_buf[0] as u16 | ((speed_buf[1] as u16) << 8))) as f64 / 64.0;

        let base_angle = index as usize * 4;

        let samples = (0..4).filter_map(|i| {
            let data = &buffer[4*i..4*(i+1)];
            decode_data(base_angle + i, data)
        }).collect();

        (Some(speed), samples)

    } else {
        (None, vec![])
    }

}

fn decode_data(angle: usize, data: &[u8]) -> Option<Sample>{
    let distance = (((0b00111111 & data[1]) as u16) << 8) | data[0] as u16;
    let invalid = (data[1] & 0b10000000) != 0;
    let warning = (data[1] & 0b01000000) != 0;
    let strength = ((data[3] as u16) << 8) | data[2] as u16;

    if !invalid && !warning {
        Some(Sample{
            angle: angle as f64,
            distance,
            quality: strength
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