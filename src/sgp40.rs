use i2c_linux::I2c;
#[allow(unused_imports)]
use log::{debug, error, log_enabled, info, warn, Level};
use std::fs::File;
use std::path::Path;

use crate::{i2cio, voc_algo::VocAlgorithmParams};

const DEVICE_ADDR_DEFAULT: u16 = 0x59;

// -- the soft reset time is actually up to or less than 0.6ms
const SGP40_SOFT_RESET_DELAY_MS: u32 = 1;
const SGP40_DATA_READY_DELAY_MS: u32 = 30;


pub struct SGP40 {
    // -- i2c bus
    i2c: I2c<File>,
    // -- device address
    device_addr: u16,
    // -- voc
    voc_algo: VocAlgorithmParams,
}

impl SGP40 {

    pub fn new(i2c_bus_path: &Path) -> Result<SGP40,std::io::Error> {
        // -- get the bus
        let i2c = i2cio::get_bus(i2c_bus_path)?;
        // -- create SGP40 object
        let mut sgp40 = SGP40 {
            i2c: i2c,
            device_addr: DEVICE_ADDR_DEFAULT,
            voc_algo: VocAlgorithmParams::new(),
        };
        // -- do a soft reset since it's in an unknown state
        debug!("Soft-resetting SGP40");
        sgp40.soft_reset()?;
        // -- ready to measure steady
        Ok(sgp40)
    }

    pub fn soft_reset(&mut self) -> Result<(), std::io::Error> {
        // -- see data sheet: subcommand 0x00 0x06 for soft reset
        let data: u8 = 0x06;
        debug!("Sending SGP40 data: {:#}", data);
        i2cio::write_byte_single(&mut self.i2c, data)?;
        // -- wait for the device to startup
        i2cio::delay(SGP40_SOFT_RESET_DELAY_MS);
        Ok(())
    }

    pub fn get_voc_data_no_compensation(&mut self) -> Result<u16, std::io::Error> {
        // -- see data sheet: subcommand 0x26 0x0f plus default compensation values with CRCs
        let data: [u8; 8] = [0x26, 0x0f, 0x80, 0x00, 0xa2, 0x66, 0x66, 0x93];
        debug!("Sending SGP40 data: {:#?}", data);
        i2cio::write_bytes(&mut self.i2c, self.device_addr, data)?;
        // -- wait for the sensor data
        i2cio::delay(SGP40_DATA_READY_DELAY_MS);
        // -- read response
        let mut read_buf: [u8; 3] = [0; 3];
        i2cio::read_bytes(&mut self.i2c, self.device_addr, &mut read_buf)?;
        let voc_raw_msb = read_buf[0];
        let voc_raw_lsb = read_buf[1];
        let voc_raw_crc = read_buf[2];
        let calc_crc = Self::calc_crc(&[voc_raw_msb, voc_raw_lsb]);
        let voc_raw = (voc_raw_msb as u16) << 8 | (voc_raw_lsb as u16);
        if voc_raw_crc != calc_crc {
            warn!("Expected CRC {:#04x}, received CRC {:#04x}", calc_crc, voc_raw_crc);
        }
        Ok(voc_raw)
    }

    pub fn get_voc_data_with_compensation(&mut self,
        humidity_raw: u16, temperature_raw: u16) -> Result<u16, std::io::Error> {
        let humidity_raw_msb: u8 = (humidity_raw >> 8) as u8;
        let humidity_raw_lsb: u8 = (humidity_raw & 0xff) as u8;
        let temperature_raw_msb: u8 = (temperature_raw >> 8) as u8;
        let temperature_raw_lsb: u8 = (temperature_raw & 0xff) as u8;
        // -- see data sheet: subcommand 0x26 0x0f plus compensation values with CRCs
        let mut data: [u8; 8] = [0x26, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        data[2] = humidity_raw_msb;
        data[3] = humidity_raw_lsb;
        data[4] = Self::calc_crc(&[humidity_raw_msb, humidity_raw_lsb]);
        data[5] = temperature_raw_msb;
        data[6] = temperature_raw_lsb;
        data[7] = Self::calc_crc(&[temperature_raw_msb, temperature_raw_lsb]);
        debug!("Sending SGP40 data: {:#?}", data);
        i2cio::write_bytes(&mut self.i2c, self.device_addr, data)?;
        // -- wait for the sensor data
        i2cio::delay(SGP40_DATA_READY_DELAY_MS);
        // -- read response
        let mut read_buf: [u8; 3] = [0; 3];
        i2cio::read_bytes(&mut self.i2c, self.device_addr, &mut read_buf)?;
        let voc_raw_msb = read_buf[0];
        let voc_raw_lsb = read_buf[1];
        let voc_raw_crc = read_buf[2];
        let calc_crc = Self::calc_crc(&[voc_raw_msb, voc_raw_lsb]);
        let voc_raw = (voc_raw_msb as u16) << 8 | (voc_raw_lsb as u16);
        if voc_raw_crc != calc_crc {
            warn!("Expected CRC {:#04x}, received CRC {:#04x}", calc_crc, voc_raw_crc);
        }
        Ok(voc_raw)
    }

    pub fn process_voc(&mut self, voc_raw: u16) -> f64 {
        self.voc_algo.process(voc_raw)
    }

    fn calc_crc<const LEN: usize>(data: &[u8; LEN]) -> u8 {
        let mut crc: u8 = 0xff;
        for i in 0..data.len() {
            crc ^= data[i];
            let mut b = 8;
            while b > 0 {
                println!("b: {}", b);
                if (crc & 0x80) > 0 {
                    crc = (crc << 1) ^ 0x31;
                } else {
                    crc = crc << 1;
                }
                b -= 1;
            }
        }
        crc
    }

}