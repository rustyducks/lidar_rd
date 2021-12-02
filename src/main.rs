use lidar_rd::{Lidar, Sample, XV11, LD06};
use std::error::Error;
use std::iter::Iterator;
use std::time::Duration;


fn print_turn(turn: Vec<Option<Sample>>) {
    let txt = turn
            .iter()
            .filter_map(|x| x.as_ref())
            .map(|s| s.to_string())
            .collect::<Vec<_>>()
            .join("\n");
    println!("{}\n\n", txt);
}

fn main() -> Result<(), Box<dyn Error>> {
    //let mut l = XV11::new("/dev/ttyUSB0");
    let mut l = LD06::new("/dev/ttyUSB0");
    l.start()?;

    
    // // iter with a for loop
    // for turn in l {
    //     print_turn(turn);
    // }

    // // iter with a for map or anything else
    // let _ = l.map(|turn| {
    //     print_turn(turn);
    // }).collect::<Vec<_>>();


    // // iter with a loop loop. Allow to exit the loop.
    let start_time = std::time::SystemTime::now();
    loop {
        if let Some(turn) = l.get_scan() {
            print_turn(turn);
        }

        if start_time.elapsed()? > Duration::from_secs(2) {
            break;
        }
    }

    println!("Its been a while, let's stop now !");

    // l.stop();

    

    Ok(())
}
