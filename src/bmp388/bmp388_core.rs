use i2c_linux::I2c;
#[allow(unused_imports)]
use log::{debug, info};
use std::fs::File;
use std::path::Path;
use std::{thread, time};

use crate::i2cio;

use super::bmp388_enums::*;

// -- chip id
const BMP388_CHIP_ID: u8 = 0x50;

// -- length of multi-byte registers
const BMP388_LEN_TRIMMING_COEFFICIENTS: usize = 21;
const BMP388_LEN_PRESSURE_DATA: usize = 3;
const BMP388_LEN_TEMPERATURE_DATA: usize = 3;
#[allow(dead_code)]
const BMP388_LEN_SENSOR_TIME: usize = 3;
#[allow(dead_code)]
const BMP388_LEN_FIFO_LENGTH: usize = 2;
#[allow(dead_code)]
const BMP388_LEN_FIFO_WATERMARK: usize = 2;

// -- registers
const BMP388_REG_CHIP_ID: u8 = 0x00;
#[allow(dead_code)]
const BMP388_REG_ERRORS: u8 = 0x02;
const BMP388_REG_STATUS: u8 = 0x03;
const BMP388_REG_PRESSURE_DATA: u8 = 0x04;
const BMP388_REG_TEMPERATURE_DATA: u8 = 0x07;
#[allow(dead_code)]
const BMP388_REG_SENSOR_TIME: u8 = 0x0C;
#[allow(dead_code)]
const BMP388_REG_EVENT: u8 = 0x10;
#[allow(dead_code)]
const BMP388_REG_INT_STATUS: u8 = 0x11;
#[allow(dead_code)]
const BMP388_REG_FIFO_LENGTH: u8 = 0x12;
#[allow(dead_code)]
const BMP388_REG_FIFO_DATA: u8 = 0x14;
#[allow(dead_code)]
const BMP388_REG_FIFO_WATERMARK: u8 = 0x15;
#[allow(dead_code)]
const BMP388_REG_FIFO_CONFIG_1: u8 = 0x17;
#[allow(dead_code)]
const BMP388_REG_FIFO_CONFIG_2: u8 = 0x18;
#[allow(dead_code)]
const BMP388_REG_INT_CONTROL: u8 = 0x19;
#[allow(dead_code)]
const BMP388_REG_IF_CONF: u8 = 0x1a;
const BMP388_REG_POWER_CONTROL: u8 = 0x1b;
const BMP388_REG_OVERSAMPLING_RATE: u8 = 0x1c;
const BMP388_REG_OUTPUT_DATA_RATE: u8 = 0x1d;
const BMP388_REG_CONFIG: u8 = 0x1f;
const BMP388_REG_TRIMMING_COEFFICIENTS: u8 = 0x31;
const BMP388_REG_CMD: u8 = 0x7e;

// -- commands
#[allow(dead_code)]
const BMP388_CMD_FIFO_FLUSH: u8 = 0xb0;
const BMP388_CMD_SOFT_RESET: u8 = 0xb6;

// -- other constants
const BMP388_STARTUP_DELAY_MS: u64 = 2;

const BME280_PRESSURE_SENSOR_ENABLED_BIT: u8 = 0x1;
const BME280_TEMPERATURE_SENSOR_ENABLED_BIT: u8 = 0x2;
const BME280_POWER_MODE_LOW_BIT: u8 = 4;

const BME280_STATUS_CMD_READY_MASK: u8 = 0x10;
const BME280_STATUS_PRESSURE_DATA_READY_MASK: u8 = 0x20;
const BME280_STATUS_TEMPERATURE_DATA_READY_MASK: u8 = 0x40;

#[derive(Debug)]
struct CalibData
{
    // -- Calibration coefficients for the pressure sensor
    par_p1: f64,
    par_p2: f64,
    par_p3: f64,
    par_p4: f64,
    par_p5: f64,
    par_p6: f64,
    par_p7: f64,
    par_p8: f64,
    par_p9: f64,
    par_p10: f64,
    par_p11: f64,
    // -- Calibration coefficients for the temperature sensor
    par_t1: f64,
    par_t2: f64,
    par_t3: f64,    
}

#[derive(Debug, Default)]
pub struct RawData
{
    // -- Un-compensated pressure
    pub pressure: u32,
    // -- Un-compensated temperature
    pub temperature: u32,
}


pub struct BMP388 {
    // -- i2c bus
    i2c: I2c<File>,
    // -- device address.
    device_addr: BMP388DeviceAddress,
    // -- calibration data
    calib_data: CalibData,
    // -- uncompensated data
    raw_data: RawData,    
}

impl BMP388 {

    pub fn new(i2c_bus_path: &Path, device_addr: BMP388DeviceAddress, 
        osr_p: BMP388OverSamplingPr, osr_t: BMP388OverSamplingTp, 
        irr_filter: BMP388IrrFilter, odr: BMP388OutputDataRate) -> Result<BMP388, std::io::Error> {
        // -- get the bus
        let mut i2c = i2cio::get_bus(i2c_bus_path)?;
        // -- set device address
        i2cio::set_slave(&mut i2c, device_addr.value())?;
        // -- check if device is available by reading chip id
        let chip_id = i2cio::read_byte(&mut i2c, BMP388_REG_CHIP_ID)?;
        if chip_id != BMP388_CHIP_ID {
            let errmsg = format!("Found unknown chip id '{chip_id:#04x}', expected '{BMP388_CHIP_ID:#04x}'");
            return Err(std::io::Error::new(std::io::ErrorKind::Other, errmsg))
        }
        debug!("Got chip id: {chip_id:#x}");
        // -- do a soft reset since it's in an unknown state
        Self::soft_reset(&mut i2c)?;
        // -- get calibration data
        let calib_data = Self::get_calib_data(&mut i2c)?;
        // -- return initialized structure
        let mut bmp388 = BMP388 {
            i2c,
            device_addr,
            calib_data,
            raw_data: Default::default(),
        };
        bmp388.set_osr_pressure_temperature(osr_p, osr_t)?;
        bmp388.set_irr_filter(irr_filter)?;
        bmp388.set_output_data_rate(odr)?;
        Ok(bmp388)
    }

    #[allow(dead_code)]
    pub fn get_device_addr(&self) -> BMP388DeviceAddress {
        self.device_addr.clone()
    }

    fn soft_reset(i2c: &mut I2c<File>) -> Result<(), std::io::Error> {
        // -- initiate soft reset
        debug!("Initiating soft reset");
        i2cio::write_byte(i2c, BMP388_REG_CMD, BMP388_CMD_SOFT_RESET)?;
        // -- wait for the device to startup
        let startup_delay = time::Duration::from_millis(BMP388_STARTUP_DELAY_MS);
        thread::sleep(startup_delay);
        Ok(())
    }

    pub fn set_output_data_rate(&mut self, subdiv_factor: BMP388OutputDataRate) -> Result<(), std::io::Error> {
        let reg_val = subdiv_factor.value();
        debug!("Setting register BMP388_REG_OUTPUT_DATA_RATE {BMP388_REG_OUTPUT_DATA_RATE:#x} to value {reg_val:#010b}");
        // -- write it back
        i2cio::write_byte(&mut self.i2c, BMP388_REG_OUTPUT_DATA_RATE, reg_val)
    }

    pub fn set_irr_filter(&mut self, irr_filter: BMP388IrrFilter) -> Result<(), std::io::Error> {
        let reg_val = irr_filter.value();
        debug!("Setting register BMP388_REG_CONFIG {BMP388_REG_CONFIG:#x} to value {reg_val:#010b}");
        // -- write it back
        i2cio::write_byte(&mut self.i2c, BMP388_REG_CONFIG, reg_val)
    }

    pub fn set_sensor_mode(&mut self, pwr_mode : BMP388SensorPowerMode, 
        enable_pressure: BMP388StatusPressureSensor, enable_temperature: BMP388StatusTemperatureSensor) -> Result<(), std::io::Error> {
        let reg_val = pwr_mode.value() << BME280_POWER_MODE_LOW_BIT | enable_temperature.value() << 1 | enable_pressure.value();
        debug!("Setting register BMP388_REG_POWER_CONTROL {BMP388_REG_POWER_CONTROL:#x} to value {reg_val:#010b}");
        // -- write it back
        i2cio::write_byte(&mut self.i2c, BMP388_REG_POWER_CONTROL, reg_val)
    }

    pub fn get_sensor_mode(&mut self) -> Result<(BMP388SensorPowerMode, BMP388StatusPressureSensor, BMP388StatusTemperatureSensor), std::io::Error> {
        // -- read current value of BMP388_REG_POWER_CONTROL
        let reg_val = i2cio::read_byte(&mut self.i2c, BMP388_REG_POWER_CONTROL)?;
        debug!("Got register BMP388_REG_POWER_CONTROL {BMP388_REG_POWER_CONTROL:#x} value {reg_val:#010b}");
        let pressure_enabled = match (reg_val & BME280_PRESSURE_SENSOR_ENABLED_BIT) > 0 {
            false => BMP388StatusPressureSensor::Disabled,
            true => BMP388StatusPressureSensor::Enabled,
        };
        let temperature_enabled = match (reg_val & BME280_TEMPERATURE_SENSOR_ENABLED_BIT) > 0 {
            false => BMP388StatusTemperatureSensor::Disabled,
            true => BMP388StatusTemperatureSensor::Enabled,  
        };
        let sensor_mode = match reg_val >> BME280_POWER_MODE_LOW_BIT {
            0 => BMP388SensorPowerMode::Sleep,
            1..=2 => BMP388SensorPowerMode::Forced,
            _ => BMP388SensorPowerMode::Normal,
        };
        Ok((sensor_mode, pressure_enabled, temperature_enabled))
    }

    pub fn get_status(&mut self) 
        -> Result<(BMP388StatusCommandDecoder, BMP388StatusPressureData, BMP388StatusTemperatureData), std::io::Error> {
        // -- read current value of BMP388_REG_POWER_CONTROL
        let reg_val = i2cio::read_byte(&mut self.i2c, BMP388_REG_STATUS)?;
        let cmd_decoder_ready = match (reg_val & BME280_STATUS_CMD_READY_MASK) > 0 {
            false => BMP388StatusCommandDecoder::NotReady,
            true => BMP388StatusCommandDecoder::Ready,
        };
        let pressure_data_ready = match (reg_val & BME280_STATUS_PRESSURE_DATA_READY_MASK) > 0 {
            false => BMP388StatusPressureData::NotReady,
            true => BMP388StatusPressureData::Ready,
        };
        let temperature_data_ready = match (reg_val & BME280_STATUS_TEMPERATURE_DATA_READY_MASK) > 0 {
            false => BMP388StatusTemperatureData::NotReady,
            true => BMP388StatusTemperatureData::Ready,
        };
        Ok((cmd_decoder_ready, pressure_data_ready, temperature_data_ready))
    }

    fn concat_bytes(msb: u8, lsb: u8) -> u16 {
        ((msb as u16) << 8) | (lsb as u16)
    }

    fn get_calib_data(i2c: &mut I2c<File>) -> Result<CalibData, std::io::Error> {
        // -- get temperature and pressure calibration data
        let mut reg_data: [u8; BMP388_LEN_TRIMMING_COEFFICIENTS] = [0; BMP388_LEN_TRIMMING_COEFFICIENTS];
        let _bytes_read = i2c.i2c_read_block_data(BMP388_REG_TRIMMING_COEFFICIENTS, &mut reg_data)?;
        // -- temperature calibration coefficients
        let par_t1 = Self::concat_bytes(reg_data[1], reg_data[0]);
        // let par_t1 = par_t1 as f64 / 0.00390625;  
        let par_t1 = par_t1 as f64 * 256.0; // == 1 / 0.00390625;  
        let par_t2 = Self::concat_bytes(reg_data[3], reg_data[2]);
        // let par_t2 = par_t2 as f64 / 1073741824.0;
        let par_t2 = par_t2 as f64 * 0.000000000931323; // == 1 / 1073741824.0
        let par_t3 = reg_data[4] as i8;
        // let par_t3 = par_t3 as f64 / 281474976710656.0;
        let par_t3 = par_t3 as f64 * 0.000000000000004; // == 1 / 281474976710656.0

        // -- pressure calibration coefficients
        let par_p1 = Self::concat_bytes(reg_data[6], reg_data[5]) as i16;        
        //let par_p1 = (par_p1 - 16384) as f64 / 1048576.0;
        let par_p1 = (par_p1 - 16384) as f64 * 0.000000953674316;
        let par_p2 = Self::concat_bytes(reg_data[8], reg_data[7]) as i16;        
        //let par_p2 = (par_p2 - 16384) as f64 / 536870912.0;
        let par_p2 = (par_p2 - 16384) as f64 * 0.000000001862645;
        let par_p3 = reg_data[9] as i8;        
        //let par_p3 = par_p3 as f64 / 4294967296.0;
        let par_p3 = par_p3 as f64 * 0.000000000232831;
        let par_p4 = reg_data[10] as i8;        
        //let par_p4 = (par_p4 as f64) / 137438953472.0;
        let par_p4 = (par_p4 as f64) * 0.000000000007276;
        let par_p5 = Self::concat_bytes(reg_data[12], reg_data[11]);
        //let par_p5 = (par_p5 as f64) / 0.125;
        let par_p5 = (par_p5 as f64) * 8.0;
        let par_p6 = Self::concat_bytes(reg_data[14], reg_data[13]);        
        //let par_p6 = (par_p6 as f64) / 64.0;
        let par_p6 = (par_p6 as f64) * 0.015625;
        let par_p7 = reg_data[15] as i8;        
        //let par_p7 = (par_p7 as f64) / 256.0;
        let par_p7 = (par_p7 as f64) * 0.00390625;
        let par_p8 = reg_data[16] as i8;        
        //let par_p8 = (par_p8 as f64) / 32768.0;
        let par_p8 = (par_p8 as f64) * 0.000030517578125;
        let par_p9 = Self::concat_bytes(reg_data[18], reg_data[17]) as i16;        
        //let par_p9 = (par_p9 as f64) / 281474976710656.0;
        let par_p9 = (par_p9 as f64) * 0.000000000000004;
        let par_p10 = reg_data[19] as i8;
        //let par_p10 = (par_p10 as f64) / 281474976710656.0;
        let par_p10 = (par_p10 as f64) * 0.000000000000004;
        let par_p11 = reg_data[20] as i8;
        //let par_p11 = (par_p11 as f64) / 36893488147419103232.0;        
        let par_p11 = (par_p11 as f64) * 0.00000000000000000002710505431213761;

        // -- create calibration structure
        let calib_data = CalibData {
            par_t1, par_t2, par_t3,
            par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, 
            par_p7, par_p8, par_p9, par_p10, par_p11,            
        };
        debug!("Got calibration data: {calib_data:#?}");
        Ok(calib_data)

    }

    pub fn get_data_raw(&mut self) -> Result<(), std::io::Error> {
        // -- get temperature and pressure data
        const DATA_LEN: usize = BMP388_LEN_PRESSURE_DATA + BMP388_LEN_TEMPERATURE_DATA;
        let mut reg_data: [u8; DATA_LEN] = [0; DATA_LEN];
        let _bytes_read = self.i2c.i2c_read_block_data(BMP388_REG_PRESSURE_DATA, &mut reg_data)?;
        debug!("Got {_bytes_read} bytes of raw data");
        let data_xlsb = reg_data[0] as u32;
        let data_lsb = (reg_data[1] as u32) << 8;
        let data_msb = (reg_data[2] as u32) << 16;
        let pressure = data_msb | data_lsb | data_xlsb;
        let data_xlsb = reg_data[3] as u32;
        let data_lsb = (reg_data[4] as u32) << 8;
        let data_msb = (reg_data[5] as u32) << 16;
        let temperature = data_msb | data_lsb | data_xlsb;
        // -- create raw data structure
        let raw_data = RawData {
            pressure,
            temperature,
        };
        debug!("Got raw data: {raw_data:#?}");
        self.raw_data = raw_data;        
        Ok(())
    }

    pub fn get_pressure_raw(&mut self) -> Result<u32, std::io::Error> {
        // -- get temperature and pressure data
        let mut reg_data: [u8; BMP388_LEN_PRESSURE_DATA] = [0; BMP388_LEN_PRESSURE_DATA];
        let _bytes_read = self.i2c.i2c_read_block_data(BMP388_REG_PRESSURE_DATA, &mut reg_data)?;
        let pressure = (reg_data[2] as u32) << 16 | (reg_data[1] as u32) << 8 | (reg_data[0] as u32);
        debug!("Got raw pressure: {pressure}");
        Ok(pressure)
    }

    pub fn get_temperature_raw(&mut self) -> Result<u32, std::io::Error> {
        // -- get temperature and pressure data
        let mut reg_data: [u8; BMP388_LEN_TEMPERATURE_DATA] = [0; BMP388_LEN_TEMPERATURE_DATA];
        let _bytes_read = self.i2c.i2c_read_block_data(BMP388_REG_TEMPERATURE_DATA, &mut reg_data)?;
        let temperature = (reg_data[2] as u32) << 16 | (reg_data[1] as u32) << 8 | (reg_data[0] as u32);
        debug!("Got raw temperature: {temperature}");
        Ok(temperature)
    }

    pub fn set_osr_pressure_temperature(&mut self, osr_p: BMP388OverSamplingPr, osr_t : BMP388OverSamplingTp) -> Result<(), std::io::Error> {
        // -- write oversampling for pressure and temperature
        let reg_val = osr_t.value() << 3 | osr_p.value();
        debug!("Setting register BMP388_REG_OVERSAMPLING_RATE {BMP388_REG_OVERSAMPLING_RATE:#x} to value {reg_val:#010b} / {osr_p} for pressure, {osr_t} for temperature");
        i2cio::write_byte(&mut self.i2c, BMP388_REG_OVERSAMPLING_RATE, reg_val)
    }

    pub fn get_temperature(&self) -> f64 {    
        let temperature = self.raw_data.temperature as f64;
        let partial_data1 = temperature - self.calib_data.par_t1;
        let partial_data2 = partial_data1 * self.calib_data.par_t2;
        let t_lin = partial_data2 + ((partial_data1 * partial_data1) * self.calib_data.par_t3);
        t_lin
    }

    pub fn get_pressure(&self, temperature: f64) -> f64 {    
        let temperature_pow_2 = temperature.powi(2);
        let temperature_pow_3 = temperature.powi(3);
        let pressure_raw = self.raw_data.pressure as f64;
        let partial_data1 = self.calib_data.par_p6 * temperature;
        let partial_data2 = self.calib_data.par_p7 * temperature_pow_2;
        let partial_data3 = self.calib_data.par_p8 * temperature_pow_3;
        let partial_out1 = self.calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;
        let partial_data1 = self.calib_data.par_p2 * temperature;
        let partial_data2 = self.calib_data.par_p3 * temperature_pow_2;
        let partial_data3 = self.calib_data.par_p4 * temperature_pow_3;
        let partial_out2 = pressure_raw * (self.calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);
        let partial_data1 = pressure_raw.powi(2);
        let partial_data2 = self.calib_data.par_p9 + self.calib_data.par_p10 * temperature;
        let partial_data3 = partial_data1 * partial_data2;
        let partial_data4 = partial_data3 + pressure_raw.powi(3) * self.calib_data.par_p11;
        let pressure = partial_out1 + partial_out2 + partial_data4;
        pressure
    }

}