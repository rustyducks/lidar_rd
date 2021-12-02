use std::time::Duration;
use std::sync::mpsc::{self, TryRecvError, Receiver};
use serialport::SerialPort;
use std::sync::Arc;
use std::sync::Mutex;
//use std::sync::RwLock;
use std::mem;
use crate::lidar::{Lidar, Sample, Turn, impl_iterator, impl_drop};
use std::thread;
use std::io;


const crcTable: [u8; 256] = 
[ 
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 
    0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 
    0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 
    0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 
    0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 
    0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 
    0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 
    0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 
    0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 
    0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 
    0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 
    0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 
    0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 
    0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 
    0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 
    0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 
    0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 
    0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 
    0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 
    0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 
    0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 
    0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8 
];


pub struct LD06 {
    // inner: Arc<RwLock<UST05LNInner>>,
    port: String,
    tx_cmd: Option<mpsc::Sender<()>>,
    join_handle: Option<thread::JoinHandle<()>>,
    data: Arc<Mutex<Box<Option<Turn>>>>,
}



impl Lidar for LD06 {
    fn get_scan(&self) -> Option<Vec<Option<Sample>>> {
        let mut boxed_turn = self.data.lock().unwrap();
        let bt = mem::replace(&mut *boxed_turn, Box::new(None));
        match *bt {
            Some(turn) => Some(turn.samples),
            None=> None
        }
    }

    fn start(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let (tx_cmd, rx_cmd) = mpsc::channel();
        self.tx_cmd = Some(tx_cmd.clone());

        let port = serialport::new(&self.port, 230_400)
            .timeout(Duration::from_millis(3))
            .open()?;

        let adata = self.data.clone();
        let th = thread::spawn(move || ld06_run(port, rx_cmd, adata));
        self.join_handle = Some(th);
            Ok(())
    }

    fn stop(&mut self) {
        if let Some(tx) = &self.tx_cmd {
            let _ = tx.send(());
        }

        if let Some(handle) = self.join_handle.take() {
            handle.join().expect("failed to join thread");
            self.join_handle = None;
        }
    }

    fn is_running(&self) -> bool {
        if let Some(_) = self.join_handle {
            true
        } else {
            false
        }
    }
}


impl LD06 {
    pub fn new(port: &str) -> LD06 {
        LD06 {
            port: port.into(),
            tx_cmd: None,
            join_handle: None,
            data: Arc::new(Mutex::new(Box::new(None)))
        }
    }
}

enum RcvState {
    WaitStart,
    WaitLen,
    GetPayload(usize),
    WaitChk,
}

struct LD06Header {
    data_len: usize,

}

struct LD06Transport {
    frame: Vec<u8>,
    buffer: Vec<u8>,
    rcv_state: RcvState,
    nb_points: usize,
}

fn u16le_from_slice(buffer: &[u8]) -> u16 {
    (buffer[1] as u16) << 8 | (buffer[0] as u16)
}

impl LD06Transport {

    fn new() -> LD06Transport {
        LD06Transport { frame: Vec::new(), buffer: Vec::new(), rcv_state: RcvState::WaitStart, nb_points: 0 }
    }

    fn checksum(&self)  -> u8 
    {
        let (_, f) = self.frame.split_last().unwrap();

        f.iter().fold(0, |crc, x| {
            let index = (crc ^ x) & 0xff;
            let crc = crcTable[index as usize];
            crc
        })
    }
    

    // TODO change return type to a "data type", and write parse code
    fn parse(&self) -> (f64, Vec<Sample>) {
        let speed = u16le_from_slice(&self.frame[2..4]) as f64 / 360.0;
        let start_angle = u16le_from_slice(&self.frame[4..6]) as f64 * 0.01;

        let end_angle = u16le_from_slice(&self.frame[6+self.nb_points*3..8+self.nb_points*3]) as f64 * 0.01;
        let timestamp = u16le_from_slice(&self.frame[8+self.nb_points*3..10+self.nb_points*3]);

        let step = if end_angle < start_angle {
            (end_angle + 360. - start_angle) / (self.nb_points - 1) as f64
        } else {
            (end_angle - start_angle) / (self.nb_points - 1) as f64
        };

        let samples = self.frame[6..6+self.nb_points*3]
            .chunks(3)
            .enumerate()
            .map(|(i, chuck)| {
                let angle = start_angle + step*(i as f64);
                let angle = if angle > 360. {angle - 360.} else {angle};
                let distance = u16le_from_slice(&chuck[0..2]);
                let confidence = chuck[2];
                Sample {
                    angle,
                    distance,
                    quality: confidence as u16
                }
        }).collect::<Vec<_>>();

        (speed, samples)
    }

    fn put(&mut self, c: u8) -> Option<(f64, Vec<Sample>)> {
        self.buffer.push(c);
        match self.rcv_state {
            RcvState::WaitStart => {
                self.buffer.clear();
                if c == 0x54 {
                    self.buffer.push(0x54);
                    self.rcv_state = RcvState::WaitLen;
                } else {
                    self.buffer.clear();
                }
            },
            RcvState::WaitLen => {
                self.nb_points = c as usize & 0x1F;
                let nb_bytes = 3 * self.nb_points + 8;
                self.rcv_state = RcvState::GetPayload(nb_bytes);
            },
            RcvState::GetPayload(mut n) => {
                n -= 1;
                self.rcv_state = match n {
                    0 => RcvState::WaitChk,
                    _ => RcvState::GetPayload(n)
                };
            },
            RcvState::WaitChk => {
                //if c == self.checksum() {
                //  TODO checksum code

                self.rcv_state = RcvState::WaitStart;

                // swap buffer and frame to analyse the frame, and clear the buffer.
                mem::swap(&mut self.buffer, &mut self.frame);
                self.buffer.clear();

                let calc_check = self.checksum();
                if c == calc_check {
                    //return None;
                    return Some(self.parse());
                } else {
                    println!("checksum failed: {} {}", c, calc_check);
                }
            },
        };
        None
    }
}

fn ld06_run(mut serial: Box<dyn SerialPort>, rx_cmd: Receiver<()>, data: Arc<Mutex<Box<Option<Turn>>>>) {
    let mut transport = LD06Transport::new();

    let mut turn = Turn::new();

    loop {
        thread::sleep(Duration::from_micros(10));

        let mut buffer: [u8; 47] = [0; 47];
        match serial.read(&mut buffer) {
            Ok(nb) => {
                for c in &buffer[0..nb] {
                    if let Some((speed, samples)) = transport.put(*c) {
                        for s in samples {
                            if s.angle < turn.last_angle() {
                                // a turn is complete, update LD06 last turn
                                let mut boxed_turn = data.lock().unwrap();
                                *boxed_turn = Box::new(Some(turn));

                                // create the new current turn
                                turn = Turn::new();
                            }

                            turn.push(s);
                        }
                    }
                }
;
            },
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
            Err(e) => eprintln!("{:?}", e),

        };

        match rx_cmd.try_recv() {
            Ok(_) | Err(TryRecvError::Disconnected) => {
                println!("Terminating.");
                break;
            }
            Err(TryRecvError::Empty) => {}
        }

    }
    
}

impl_iterator!(LD06);
impl_drop!(LD06);
