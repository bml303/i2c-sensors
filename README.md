# i2c-sensors

This is a library to work with i2c sensors like BME280, ENS160, TMP117 and similar.

```rust
use i2c_sensors::ens160::{
    Ens160DeviceAddress,
    ENS160,
};
use std::path::Path;

fn main() {

    let bus_path = Path::new("/dev/i2c-0");
    let device_addr = Ens160DeviceAddress::Default;
    
    let mut ens160 = ENS160::new(bus_path, device_addr)
        .expect("Failed to initialize ENS160");    

    let ens160_part_id = ens160.get_part_id();
    println!("ENS160 device id is {ens160_part_id:#06x}");

    let ens160_aqi = ens160.get_air_quality_index()
        .expect("Failed to get air quality index from ENS160");
    println!("ENS160 Airt Quality Index: {ens160_aqi}");

    let ens160_validity = ens160.get_validity()
        .expect("Failed to get validity from ENS160");
    println!("ENS160 Validity: {ens160_validity}");

    let ens160_tvoc = ens160.get_total_volatile_organic_compounds()
        .expect("Failed to get total volatile organic compounds from ENS160");
    println!("ENS160 Total Volatile Organic Compounds Concentration (ppb): {ens160_tvoc}");

    let ens160_eco2 = ens160.get_equivalent_co2()
        .expect("Failed to get equivalent CO2 from ENS160");
    println!("ENS160 Equivalent CO2 Concentration (ppm): {ens160_eco2}");

}
```