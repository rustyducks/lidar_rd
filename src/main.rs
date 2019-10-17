use lidar_rd::{Lidar, Sample, XV11};

fn main() {
    let mut l = XV11::new("/dev/ttyUSB0");
    l.start();

    // iterator style, so class but be carefull : infinite iterator !
    for scan in l.iter() {
        let txt = scan
            .iter()
            .filter_map(|x| x.as_ref())
            .map(|s| s.to_string())
            .collect::<Vec<_>>()
            .join("\n");
        println!("speed: {}\n{}\n\n", l.get_lidar_speed(), txt);
    }

    // loop {
    //     if let Some(scan) = l.get_scan() {
    //         let txt = scan.iter()
    //             .filter_map(|x| x.as_ref())
    //             .map(|s| s.to_string()).collect::<Vec<_>>().join("\n");
    //         println!("speed: {}\n{}\n\n", l.get_lidar_speed(), txt);
    //     }
    // }

    // l.stop();
}
