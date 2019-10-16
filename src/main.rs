
use lidar_rd::{XV11, Lidar, Sample};


fn main() {

    let mut l = XV11::new("/dev/ttyUSB0");
    l.start();

    // iterator style, so class but be carefull : infinite iterator !
    for scan in l { 
        let txt = scan.iter().map(|s| s.to_string()).collect::<Vec<_>>().join("\n");
        println!("speed: {}\n{}\n\n", XV11::get_lidar_speed(), txt);
    }



    // loop {
    //     if let Some(samples) = XV11::get_scan() {
    //         let txt = samples.iter().map(|s| s.to_string()).collect::<Vec<_>>().join("\n");
    //         println!("speed: {}", XV11::get_lidar_speed());
    //         println!("{}\n\n", txt);
    //     }
    // }

    
    // l.stop();

}

