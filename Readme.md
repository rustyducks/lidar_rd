# Lidar RD

This library contains trait for lidar handling, as well as implementation for the Neato XV11 and soon for the Hokuyo UST-05-LN.

**How to use it ?**

Add this crate as dependency to your Cargo.toml :

`lidar_rd = {git = "https://github.com/rustyducks/lidar_rd"}`


**Usage example :**

```rust
use lidar_rd::{XV11, Lidar, Sample};


fn main() {

    let mut l = XV11::new("/dev/ttyUSB0");
    l.start();

    for scan in l.iter() {
        let txt = scan.iter()
            .filter_map(|x| x.as_ref())
            .map(|s| s.to_string()).collect::<Vec<_>>().join("\n");
        println!("speed: {}\n{}\n\n", l.get_lidar_speed(), txt);
    }
    
    // l.stop();

}
```

**Cross-compile for Raspberry Pi:**

`cargo build --target armv7-unknown-linux-gnueabihf --release`

