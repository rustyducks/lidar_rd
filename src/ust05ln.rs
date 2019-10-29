use bufstream::BufStream;
use regex::Regex;
use serial::prelude::*;
use std::io;
use std::io::BufRead;
use std::io::{Read, Write};
use std::mem;
use std::error::Error;
use std::sync::mpsc::{self, TryRecvError};
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::RwLock;
use std::thread;
use std::time::Duration;

use crate::lidar::{Lidar, Sample};

pub struct UST05LN {
    inner: Arc<RwLock<UST05LNInner>>,
    tx: Option<mpsc::Sender<()>>,
    started: bool,
    join_handle: Option<thread::JoinHandle<()>>,
}

struct UST05LNInner {
    scan: Mutex<Box<Option<Vec<Option<Sample>>>>>,
    timestamp: Mutex<u64>,
    port_path: String,
}

pub struct UST05LNIter<'a> {
    inner: &'a UST05LN,
}

impl Lidar for UST05LN {
    fn get_scan(&self) -> Option<Vec<Option<Sample>>> {
        self.inner.clone().read().unwrap().get_turn()
    }

    fn start(&mut self) -> Result<(), Box<dyn Error>> {
        let local_self = self.inner.clone();

        let (tx, rx) = mpsc::channel();

        self.tx = Some(tx.clone());

        let mut port = serial::open(&local_self.read().unwrap().port_path)?;

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
                .read_ust(&mut port, rx)
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

impl<'a> Iterator for UST05LNIter<'a> {
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

impl UST05LN {
    pub fn new(port_path: &str) -> UST05LN {
        UST05LN {
            inner: Arc::new(RwLock::new(UST05LNInner {
                port_path: port_path.to_string(),
                scan: Mutex::new(Box::new(None)),
                timestamp: Mutex::new(0),
            })),
            tx: None,
            started: false,
            join_handle: None,
        }
    }

    pub fn iter<'a>(&'a self) -> UST05LNIter<'a> {
        UST05LNIter { inner: &self }
    }

    pub fn get_timestamp(&self) -> u64 {
        self.inner.clone().read().unwrap().get_timestamp()
    }
}

impl UST05LNInner {
    pub fn get_turn(&self) -> Option<Vec<Option<Sample>>> {
        let mut boxed_samples = self.scan.lock().unwrap();
        let bb = mem::replace(&mut *boxed_samples, Box::new(None));
        *bb
    }

    fn set_turn(&self, b: Box<Option<Vec<Option<Sample>>>>) {
        let mut boxed_samples = self.scan.lock().unwrap();
        *boxed_samples = b;
    }

    fn get_timestamp(&self) -> u64 {
        *self.timestamp.lock().unwrap()
    }

    fn set_timestamp(&self, timestamp: u64) {
        *self.timestamp.lock().unwrap() = timestamp;
    }

    fn read_ust<T: Read + Write>(&self, port: &mut T, rx: mpsc::Receiver<()>) -> io::Result<()> {
        let mut buf = BufStream::new(port);

        self.start_ranging(&mut buf); //sends command over serial to ask the LIDAR to start ranging

        let scan_regex = Regex::new(r"#GT00:([0-9A-F]{12}):([0-9]{6}):([0-9A-F]{4332})").unwrap();
        let mes_regex = Regex::new(r"(.{4})(.{4})").unwrap();

        for line in buf.lines() {
            match line {
                Ok(line) => {
                    if let Some((timestamp, scan)) =
                        UST05LNInner::parse_scan(&line, &scan_regex, &mes_regex)
                    {
                        self.set_turn(Box::new(Some(scan)));
                        self.set_timestamp(timestamp);
                    }
                }
                Err(e) => {
                    println!("read error : {:?}", e);
                }
            }

            match rx.try_recv() {
                Ok(_) | Err(TryRecvError::Disconnected) => {
                    println!("Terminating.");
                    break;
                }
                Err(TryRecvError::Empty) => {}
            }
        }

        Ok(())
    }

    /*
    Sends "#GT15466\n" to the LIDAR to stop ranging. The LIDAR should reply with "#ST00A845\n".
    */
    fn start_ranging<S: Read + Write>(&self, buf: &mut bufstream::BufStream<S>) {
        while let Err(()) = self.stop_ranging(buf) {
            println!("try again!")
        }

        match buf.write(b"#GT15466\n") {
            Ok(_) => (),
            Err(e) => {
                println!("{:?}", e);
            }
        };
        buf.flush().unwrap();

        let mut ret: String = String::new();

        if let Ok(_nb) = buf.read_line(&mut ret) {
            if ret != "#ST00A845\n" {
                //println!("Unexpected answer. Expecting '#ST00A845\\n' but got {:?}", ret);
                println!("Unexpected answer to start_ranging.");
            } else {
                println!("success");
            }
        }
    }

    /*
    Sends "#ST5297\n" to the LIDAR to stop ranging.
    */
    fn stop_ranging<S: Read + Write>(&self, buf: &mut bufstream::BufStream<S>) -> Result<(), ()> {
        match buf.write(b"#ST5297\n") {
            Ok(_) => (),
            Err(e) => {
                println!("stop ranging : {:?}", e);
            }
        };
        buf.flush().unwrap();
        let mut ret: String = String::new();
        buf.read_line(&mut ret).unwrap();
        println!("stop ranging ret={}", ret);
        if ret == "#ST00A845\n" {
            Ok(())
        } else {
            Err(())
        }
    }

    /*
    Parse data from LIDAR
    */
    fn parse_scan(
        line: &String,
        scan_regex: &Regex,
        mes_regex: &Regex,
    ) -> Option<(u64, Vec<Option<Sample>>)> {
        let mut matches = scan_regex.captures_iter(line);

        if let Some(turn) = matches.next() {
            //Unwrap should not crash because it has been matched by the regex as an hex string
            let timestamp = u64::from_str_radix(turn.get(1).unwrap().as_str(), 16).unwrap();

            //scan.mysterious_field = turn.get(2).unwrap().as_str().to_string();

            let mut samples = vec![];

            for (i, c) in mes_regex
                .captures_iter(turn.get(3).unwrap().as_str())
                .enumerate()
            {
                let angle = ((i as f64) * 270.0 / 540.0 - (270.0 / 2.0)).to_radians();
                let distance = u16::from_str_radix(c.get(1).unwrap().as_str(), 16).unwrap();
                let quality = u16::from_str_radix(c.get(2).unwrap().as_str(), 16).unwrap();
                if quality > 0 && distance < 6000 {
                    samples.push(Some(Sample {
                        angle,
                        distance,
                        quality: quality,
                    }));
                } else {
                    samples.push(None);
                }
            }
            Some((timestamp, samples))
        } else {
            None
        }
    }
}
