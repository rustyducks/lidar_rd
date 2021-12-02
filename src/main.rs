use lidar_rd::{Lidar, Sample, XV11, LD06};
use std::error::Error;
use std::iter::Iterator;

fn main() -> Result<(), Box<dyn Error>> {
    //let mut l = XV11::new("/dev/ttyUSB0");
    let mut l = LD06::new("/dev/ttyUSB0");
    l.start()?;

    //let sd : &mut dyn Lidar = &mut l;
    //let q: Vec<_> = sd.collect();

    // iterator style, so class but be carefull : infinite iterator !
    // for scan in l.iter() {
    //     let txt = scan
    //         .iter()
    //         .filter_map(|x| x.as_ref())
    //         .map(|s| s.to_string())
    //         .collect::<Vec<_>>()
    //         .join("\n");
    //     println!("speed: {}\n{}\n\n", l.get_lidar_speed(), txt);
    // }

    // Ok(())

    let _ = l.map(|a|println!("{:?}", a)).collect::<Vec<_>>();


    // loop {
    //     if let Some(scan) = l.get_scan() {
    //         let txt = scan.iter()
    //             .filter_map(|x| x.as_ref())
    //             .map(|s| s.to_string()).collect::<Vec<_>>().join("\n");
    //         println!("{}\n\n", txt);
    //     }
    // }

    // l.stop();

    Ok(())
}
