use i2c_linux::I2c;
#[allow(unused_imports)]
use log::{debug, info};
use std::fmt;
use std::fs::File;
use std::path::Path;
use std::{thread, time};

use crate::i2cio;

const BME280_CHIP_ID: u8 = 0x60;
const BME280_LEN_TEMP_PRESS_CALIB_DATA: usize = 26;
const BME280_LEN_HUMIDITY_CALIB_DATA: usize = 7;
const BME280_LEN_P_T_H_DATA: usize = 8;
const BME280_STARTUP_DELAY_MS: u64 = 2;
const BME280_SOFT_RESET_COMMAND: u8 = 0xb6;
const BME280_TEMPERATURE_MIN: f64 = -40.0;
const BME280_TEMPERATURE_MAX: f64 = 85.0;
const BME280_PRESSURE_MIN: f64 = 30000.0;
const BME280_PRESSURE_MAX: f64 = 110000.0;
const BME280_HUMIDITY_MIN: f64 = 0.0;
const BME280_HUMIDITY_MAX: f64 = 100.0;

// -- masks for ctrl_hum, ctrl_meas, and config registers
#[allow(dead_code)]
const BME280_CTRL_HUM_MSK: u8 = 0x07;
#[allow(dead_code)]
const BME280_CTRL_HUM_POS: u8 = 0x00;
const BME280_CTRL_MODE_MSK: u8 = 0x03;
const BME280_CTRL_PRESS_MSK: u8 = 0x1C;
const BME280_CTRL_PRESS_POS: u8 = 0x02;
const BME280_CTRL_TEMP_MSK: u8 = 0xE0;
const BME280_CTRL_TEMP_POS: u8 = 0x05;
#[allow(dead_code)]
const BME280_T_STANDBY_MSK: u8 = 0xE0;
const BME280_T_STANDBY_POS: u8 = 0x05;
#[allow(dead_code)]
const BME280_IRR_FILTER_MSK: u8 = 0x1C;
const BME280_IRR_FILTER_POS: u8 = 0x02;
#[allow(dead_code)]
const BME280_SPI3W_MAS: u8 = 0x01;

// -- masks for status
const BME280_STATUS_MEASURING: u8 = 0x08;

// -- registers
const BME280_REG_PART_ID: u8 = 0xd0;
const BME280_REG_RESET: u8 = 0xe0;
const BME280_REG_TEMP_PRESS_CALIB_DATA: u8 = 0x88;
const BME280_REG_HUMIDITY_CALIB_DATA: u8 = 0xe1;
const BME280_REG_CTRL_HUM: u8 = 0xf2;
const BME280_REG_STATUS: u8 = 0xf3;
#[allow(dead_code)]
const BME280_REG_PWR_CTRL: u8 = 0xf4;
const BME280_REG_CTRL_MEAS: u8 = 0xf4;
const BME280_REG_CONFIG: u8 = 0xf5;
const BME280_REG_DATA: u8 = 0xf7;

// -- shift values
const BME280_12_BIT_SHIFT: u8 = 12;
const BME280_8_BIT_SHIFT: u8 = 8;
const BME280_4_BIT_SHIFT: u8 = 4;

#[derive(Clone, Debug, PartialEq)]
pub enum Bme280DeviceAddress {
    Default,    
    Secondary,    
}

impl Default for Bme280DeviceAddress {
    fn default() -> Self {
        Bme280DeviceAddress::Default
    }
}

impl Bme280DeviceAddress {
    const BME280_DEV_ADDR_DEFAULT: u16 = 0x77;
    const BME280_DEV_ADDR_SECONDARY: u16 = 0x76;    

    fn value(&self) -> u16 {
        match *self {
            Self::Default => Self::BME280_DEV_ADDR_DEFAULT,
            Self::Secondary => Self::BME280_DEV_ADDR_SECONDARY,            
        }
    }
}

pub enum Bme280SensorMode {
    Bme280PowerModeSleep,
    Bme280PowerModeForced,
    Bme280PowerModeNormal
}

impl Bme280SensorMode {
    const BME280_POWERMODE_SLEEP: u8 = 0x00;
    const BME280_POWERMODE_FORCED: u8 = 0x01;
    const BME280_POWERMODE_NORMAL: u8 = 0x03;

    fn value(&self) -> u8 {
        match *self {
            Self::Bme280PowerModeSleep => Self::BME280_POWERMODE_SLEEP,
            Self::Bme280PowerModeForced => Self::BME280_POWERMODE_FORCED,
            Self::Bme280PowerModeNormal => Self::BME280_POWERMODE_NORMAL,
        }
    }
}

impl fmt::Display for Bme280SensorMode {
    // This trait requires `fmt` with this exact signature.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Bme280PowerModeSleep => write!(f, "Bme280PowerModeSleep/{:#04x}", self.value()),
            Self::Bme280PowerModeForced => write!(f, "Bme280PowerModeForced/{:#04x}", self.value()),
            Self::Bme280PowerModeNormal => write!(f, "Bme280PowerModeNormal/{:#04x}", self.value()),
        }
    }
}

pub enum Bme280OverSampling {
    NoOversampling,
    Oversampling1x,
    Oversampling2x,
    Oversampling4x,
    Oversampling8x,
    Oversampling16x,
    OversamplingMax,
}

impl Bme280OverSampling {
    const BME280_NO_OVERSAMPLING: u8 = 0x00;
    const BME280_OVERSAMPLING_1X: u8 = 0x01;
    const BME280_OVERSAMPLING_2X: u8 = 0x02;
    const BME280_OVERSAMPLING_4X: u8 = 0x03;
    const BME280_OVERSAMPLING_8X: u8 = 0x04;
    const BME280_OVERSAMPLING_16X: u8 = 0x05;
    const BME280_OVERSAMPLING_MAX: u8 = 0x06;

    fn value(&self) -> u8 {
        match *self {
            Self::NoOversampling => Self::BME280_NO_OVERSAMPLING,
            Self::Oversampling1x => Self::BME280_OVERSAMPLING_1X,
            Self::Oversampling2x => Self::BME280_OVERSAMPLING_2X,
            Self::Oversampling4x => Self::BME280_OVERSAMPLING_4X,
            Self::Oversampling8x => Self::BME280_OVERSAMPLING_8X,
            Self::Oversampling16x => Self::BME280_OVERSAMPLING_16X,
            Self::OversamplingMax => Self::BME280_OVERSAMPLING_MAX,
        }
    }
}

impl fmt::Display for Bme280OverSampling {
    // This trait requires `fmt` with this exact signature.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::NoOversampling => write!(f, "NoOversampling/{:#04x}", self.value()),
            Self::Oversampling1x => write!(f, "Oversampling1x/{:#04x}", self.value()),
            Self::Oversampling2x => write!(f, "Oversampling2x/{:#04x}", self.value()),
            Self::Oversampling4x => write!(f, "Oversampling4x/{:#04x}", self.value()),
            Self::Oversampling8x => write!(f, "Oversampling8x/{:#04x}", self.value()),
            Self::Oversampling16x => write!(f, "Oversampling16x/{:#04x}", self.value()),
            Self::OversamplingMax => write!(f, "OversamplingMax/{:#04x}", self.value()),
        }
    }
}

pub enum Bme280TimeStandby {
    Ms0_5,
    Ms10,
    Ms20,
    Ms62_5,
    Ms125,
    Ms250,
    Ms500,
    Ms1000,
}

impl Bme280TimeStandby {
    const BME280_STANDBY_TIME_0_5_MS: u8 = 0x00;
    const BME280_STANDBY_TIME_62_5_MS: u8 = 0x01;
    const BME280_STANDBY_TIME_125_MS: u8 = 0x02;
    const BME280_STANDBY_TIME_250_MS: u8 = 0x03;
    const BME280_STANDBY_TIME_500_MS: u8 = 0x04;
    const BME280_STANDBY_TIME_1000_MS: u8 = 0x05;
    const BME280_STANDBY_TIME_10_MS: u8 = 0x06;
    const BME280_STANDBY_TIME_20_MS: u8 = 0x07;

    fn value(&self) -> u8 {
        match *self {
            Self::Ms0_5 => Self::BME280_STANDBY_TIME_0_5_MS,
            Self::Ms10 => Self::BME280_STANDBY_TIME_10_MS,
            Self::Ms20 => Self::BME280_STANDBY_TIME_20_MS,
            Self::Ms62_5 => Self::BME280_STANDBY_TIME_62_5_MS,
            Self::Ms125 => Self::BME280_STANDBY_TIME_125_MS,
            Self::Ms250 => Self::BME280_STANDBY_TIME_250_MS,
            Self::Ms500 => Self::BME280_STANDBY_TIME_500_MS,
            Self::Ms1000 => Self::BME280_STANDBY_TIME_1000_MS,
        }
    }
}

impl fmt::Display for Bme280TimeStandby {
    // This trait requires `fmt` with this exact signature.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {            
            Self::Ms0_5 => write!(f, "Ms0_5/{:#04x}", self.value()),
            Self::Ms10 => write!(f, "Ms10/{:#04x}", self.value()),
            Self::Ms20 => write!(f, "Ms20/{:#04x}", self.value()),
            Self::Ms62_5 => write!(f, "Ms62_5/{:#04x}", self.value()),
            Self::Ms125 => write!(f, "Ms125/{:#04x}", self.value()),
            Self::Ms250 => write!(f, "Ms250/{:#04x}", self.value()),
            Self::Ms500 => write!(f, "Ms500/{:#04x}", self.value()),
            Self::Ms1000 => write!(f, "Ms1000/{:#04x}", self.value()),
        }
    }
}

#[allow(dead_code)]
pub enum Bme280IrrFilter {
    FilterOff,
    Filter2x,
    Filter4x,
    Filter8x,
    Filter16x,
}

impl Bme280IrrFilter {
    const BME280_FILTER_COEFF_OFF: u8 = 0x00;
    const BME280_FILTER_COEFF_2: u8 = 0x01;
    const BME280_FILTER_COEFF_4: u8 = 0x02;
    const BME280_FILTER_COEFF_8: u8 = 0x03;
    const BME280_FILTER_COEFF_16: u8 = 0x04;

    fn value(&self) -> u8 {
        match *self {
            Self::FilterOff => Self::BME280_FILTER_COEFF_OFF,
            Self::Filter2x => Self::BME280_FILTER_COEFF_2,
            Self::Filter4x => Self::BME280_FILTER_COEFF_4,
            Self::Filter8x => Self::BME280_FILTER_COEFF_8,
            Self::Filter16x => Self::BME280_FILTER_COEFF_16,
        }
    }
}

impl fmt::Display for Bme280IrrFilter {
    // This trait requires `fmt` with this exact signature.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {                        
            Self::FilterOff => write!(f, "FilterOff/{:#04x}", self.value()),
            Self::Filter2x => write!(f, "Filter2x/{:#04x}", self.value()),
            Self::Filter4x => write!(f, "Filter4x/{:#04x}", self.value()),
            Self::Filter8x => write!(f, "Filter8x/{:#04x}", self.value()),
            Self::Filter16x => write!(f, "Filter16x/{:#04x}", self.value()),
        }
    }
}

pub enum Bme280Spi3w {
    Disable,
    Enable,
}

impl Bme280Spi3w {
    const BME280_SPI3W_DISABLE: u8 = 0x00;
    const BME280_SPI3W_ENABLE: u8 = 0x01;

    fn value(&self) -> u8 {
        match *self {
            Self::Disable => Self::BME280_SPI3W_DISABLE,
            Self::Enable => Self::BME280_SPI3W_ENABLE,
        }
    }
}

impl fmt::Display for Bme280Spi3w {
    // This trait requires `fmt` with this exact signature.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {                                    
            Self::Disable => write!(f, "Disable/{:#04x}", self.value()),
            Self::Enable => write!(f, "Enable/{:#04x}", self.value()),
        }
    }
}

#[derive(Debug)]
struct CalibData
{
    // -- Calibration coefficients for the temperature sensor
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    // -- Calibration coefficients for the pressure sensor
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    // -- Calibration coefficients for the humidity sensor
    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
    // -- Variable to store the intermediate temperature coefficient
    t_fine_float: f64,
    t_fine_fixed: i32,
}

#[derive(Default)]
struct UncompData
{
    // -- Un-compensated pressure
    pressure: u32,
    // -- Un-compensated temperature
    temperature: u32,
    // -- Un-compensated humidity
    humidity: u32,
}


pub struct BME280 {
    // -- i2c bus
    i2c: I2c<File>,
    // -- device address.
    device_addr: Bme280DeviceAddress,
    // -- calibration data
    calib_data: CalibData,
    // -- uncompensated data
    uncomp_data: UncompData,
}

impl BME280 {

    pub fn new(i2c_bus_path: &Path, device_addr: Bme280DeviceAddress) -> Result<BME280, std::io::Error> {
        // -- get the bus
        let mut i2c = i2cio::get_bus(i2c_bus_path)?;
        // -- set device address
        i2cio::set_slave(&mut i2c, device_addr.value())?;  
        // -- check if device is available by reading chip id
        let chip_id = i2cio::read_byte(&mut i2c, BME280_REG_PART_ID)?;
        if chip_id != BME280_CHIP_ID {
            let errmsg = format!("Found unknown chip id '{chip_id:#04x}', expected '{BME280_CHIP_ID:#04x}'");
            return Err(std::io::Error::new(std::io::ErrorKind::Other, errmsg))
        }
        debug!("Got chip id: {chip_id:#x}");
        // -- do a soft reset since it's in an unknown state
        Self::soft_reset(&mut i2c)?;
        // -- get calibration data
        let calib_data = Self::get_calib_data(&mut i2c)?;
        // -- return initialized structure
        Ok(BME280 {
            i2c,
            device_addr,
            calib_data,
            uncomp_data: Default::default(),
        })
    }

    #[allow(dead_code)]
    pub fn get_device_addr(&self) -> Bme280DeviceAddress {
        self.device_addr.clone()
    }
    
    fn soft_reset(i2c: &mut I2c<File>) -> Result<(), std::io::Error> {
        // -- initiate soft reset
        debug!("Initiating soft reset");
        i2cio::write_byte(i2c, BME280_REG_RESET, BME280_SOFT_RESET_COMMAND)?;
        // -- wait for the device to startup
        let startup_delay = time::Duration::from_millis(BME280_STARTUP_DELAY_MS);
        thread::sleep(startup_delay);
        Ok(())
    }

    fn concat_bytes(msb: u8, lsb: u8) -> u16 {
        ((msb as u16) << 8) | (lsb as u16)
    }

    fn get_calib_data(i2c: &mut I2c<File>) -> Result<CalibData, std::io::Error> {
        // -- get temperature and pressure calibration data
        let mut reg_data: [u8; BME280_LEN_TEMP_PRESS_CALIB_DATA] = [0; BME280_LEN_TEMP_PRESS_CALIB_DATA];
        let _bytes_read = i2c.i2c_read_block_data(BME280_REG_TEMP_PRESS_CALIB_DATA, &mut reg_data)?;        
        let dig_t1 = Self::concat_bytes(reg_data[1], reg_data[0]);
        let dig_t2 = Self::concat_bytes(reg_data[3], reg_data[2]) as i16;
        let dig_t3 = Self::concat_bytes(reg_data[5], reg_data[4]) as i16;
        let dig_p1 = Self::concat_bytes(reg_data[7], reg_data[6]);
        let dig_p2 = Self::concat_bytes(reg_data[9], reg_data[8]) as i16;
        let dig_p3 = Self::concat_bytes(reg_data[11], reg_data[10]) as i16;
        let dig_p4 = Self::concat_bytes(reg_data[13], reg_data[12]) as i16;
        let dig_p5 = Self::concat_bytes(reg_data[15], reg_data[14]) as i16;
        let dig_p6 = Self::concat_bytes(reg_data[17], reg_data[16]) as i16;
        let dig_p7 = Self::concat_bytes(reg_data[19], reg_data[18]) as i16;
        let dig_p8 = Self::concat_bytes(reg_data[21], reg_data[20]) as i16;
        let dig_p9 = Self::concat_bytes(reg_data[23], reg_data[22]) as i16;
        let dig_h1 = reg_data[25];
        // -- get humidity calibration data
        let mut reg_data: [u8; BME280_LEN_HUMIDITY_CALIB_DATA] = [0; BME280_LEN_HUMIDITY_CALIB_DATA];
        let _bytes_read = i2c.i2c_read_block_data(BME280_REG_HUMIDITY_CALIB_DATA, &mut reg_data)?;
        let dig_h2 = Self::concat_bytes(reg_data[1], reg_data[0]) as i16;
        let dig_h3 = reg_data[2];
        let dig_h4_msb = ((reg_data[3] as i8) as i16) * 16;
        let dig_h4_lsb = (reg_data[4] & 0x0f) as i16;
        let dig_h4 = dig_h4_msb | dig_h4_lsb;
        let dig_h5_msb = ((reg_data[5] as i8) as i16) * 16;
        let dig_h5_lsb = (reg_data[4] >> 4) as i16;
        let dig_h5 = dig_h5_msb | dig_h5_lsb;
        let dig_h6 = reg_data[6] as i8;
        // -- create calibration structure
        let calib_data = CalibData {
            dig_t1, dig_t2, dig_t3,
            dig_p1, dig_p2, dig_p3, dig_p4, dig_p5, dig_p6, dig_p7, dig_p8, dig_p9,
            dig_h1, dig_h2, dig_h3, dig_h4, dig_h5, dig_h6,
            t_fine_float: 0.0, t_fine_fixed: 0,
        };
        debug!("Got calibration data: {calib_data:#?}");
        Ok(calib_data)

    }

    pub fn set_osr_humidity(&mut self, osr_h: Bme280OverSampling) -> Result<(), std::io::Error> {
        // -- write oversampling to ctr_hum
        let ctrl_hum = osr_h.value();
        debug!("Setting register BME280_REG_CTRL_HUM {BME280_REG_CTRL_HUM:#x} to value {ctrl_hum:#010b}");
        i2cio::write_byte(&mut self.i2c, BME280_REG_CTRL_HUM, ctrl_hum)?;
        // -- changes to ctrl_hum will be only effective after a write operation to ctrl_meas register
        // -- read current value of ctrl_meas...
        let ctrl_meas = i2cio::read_byte(&mut self.i2c, BME280_REG_CTRL_MEAS)?;
        // -- ...and write it back
        i2cio::write_byte(&mut self.i2c, BME280_REG_CTRL_MEAS, ctrl_meas)
    }

    pub fn set_osr_pressure_temperature(&mut self, osr_p : Bme280OverSampling, osr_t : Bme280OverSampling) -> Result<(), std::io::Error> {
        // -- read current value of ctrl_meas...
        let ctrl_meas = i2cio::read_byte(&mut self.i2c, BME280_REG_CTRL_MEAS)?;
        // -- ...keep mode bits, set pressure osr bits and temp osr bits...
        let ctrl_meas = (ctrl_meas & BME280_CTRL_MODE_MSK) | (osr_p.value() << BME280_CTRL_PRESS_POS) | (osr_t.value() << BME280_CTRL_TEMP_POS);
        debug!("Setting register BME280_REG_CTRL_MEAS {BME280_REG_CTRL_MEAS:#x} to value {ctrl_meas:#010b}");
        // -- ...and write it back
        i2cio::write_byte(&mut self.i2c, BME280_REG_CTRL_MEAS, ctrl_meas)
    }

    pub fn set_sensor_config(&mut self, t_standby: Bme280TimeStandby, irr_filter: Bme280IrrFilter, spi3w_en: Bme280Spi3w) -> Result<(), std::io::Error> {
        // -- set t_sb, filter bits, and spi3w_en bit
        let ctrl_config = (t_standby.value() << BME280_T_STANDBY_POS) | (irr_filter.value() << BME280_IRR_FILTER_POS) | spi3w_en.value();
        debug!("Setting register BME280_REG_CONFIG {BME280_REG_CONFIG:#x} to value {ctrl_config:#010b}");
        // -- write it back
        i2cio::write_byte(&mut self.i2c, BME280_REG_CONFIG, ctrl_config)
    }

    pub fn set_sensor_mode(&mut self, sensor_mode : Bme280SensorMode) -> Result<(), std::io::Error> {
        // -- read current value of ctrl_meas...
        let ctrl_meas = i2cio::read_byte(&mut self.i2c, BME280_REG_CTRL_MEAS)?;
        // -- ...keep pressure osr bits and temp osr bits, set mode bits...
        let ctrl_meas = (ctrl_meas & (BME280_CTRL_PRESS_MSK | BME280_CTRL_TEMP_MSK) ) | sensor_mode.value();
        debug!("Setting register BME280_REG_CTRL_MEAS {BME280_REG_CTRL_MEAS:#x} to value {ctrl_meas:#010b}");
        // -- ...and write it back
        i2cio::write_byte(&mut self.i2c, BME280_REG_CTRL_MEAS, ctrl_meas)
    }

    pub fn get_sensor_mode(&mut self) -> Result<Bme280SensorMode, std::io::Error> {
        // -- read current value of ctrl_meas...
        let ctrl_meas = i2cio::read_byte(&mut self.i2c, BME280_REG_CTRL_MEAS)?;
        let sensor_mode = match ctrl_meas & BME280_CTRL_MODE_MSK {
            0 => Bme280SensorMode::Bme280PowerModeSleep,
            1..=2 => Bme280SensorMode::Bme280PowerModeForced,
            _ => Bme280SensorMode::Bme280PowerModeNormal,
        };
        Ok(sensor_mode)
    }

    pub fn is_measuring(&mut self) -> Result<bool, std::io::Error> {
        // -- get temperature and pressure calibration data
        let status = i2cio::read_byte(&mut self.i2c, BME280_REG_STATUS)?;
        let is_measuring = (status & BME280_STATUS_MEASURING) > 0;
        Ok(is_measuring)
    }

    pub fn get_sensor_data(&mut self) -> Result<(), std::io::Error> {
        // -- get temperature and pressure calibration data
        let mut reg_data: [u8; BME280_LEN_P_T_H_DATA] = [0; BME280_LEN_P_T_H_DATA];
        let _bytes_read = self.i2c.i2c_read_block_data(BME280_REG_DATA, &mut reg_data)?;
        debug!("Read {_bytes_read} bytes sensor data");

        /* Store the parsed register values for pressure data */
        let data_msb: u32 = (reg_data[0] as u32) << BME280_12_BIT_SHIFT;
        let data_lsb: u32 = (reg_data[1] as u32) << BME280_4_BIT_SHIFT;
        let data_xlsb: u32 = (reg_data[2] as u32) >> BME280_4_BIT_SHIFT;
        self.uncomp_data.pressure = data_msb | data_lsb | data_xlsb;

        /* Store the parsed register values for temperature data */
        let data_msb: u32 = (reg_data[3] as u32) << BME280_12_BIT_SHIFT;
        let data_lsb: u32 = (reg_data[4] as u32) << BME280_4_BIT_SHIFT;
        let data_xlsb: u32 = (reg_data[5] as u32) >> BME280_4_BIT_SHIFT;
        self.uncomp_data.temperature = data_msb | data_lsb | data_xlsb;

        /* Store the parsed register values for humidity data */
        let data_msb: u32 = (reg_data[6] as u32) << BME280_8_BIT_SHIFT;
        let data_lsb: u32 = reg_data[7] as u32;
        self.uncomp_data.humidity = data_msb | data_lsb;

        Ok(())
    }

    pub fn compensate_temperature_float(&mut self) -> f64 {
        let var1: f64 = ((self.uncomp_data.temperature as f64) / 16384.0) - ((self.calib_data.dig_t1 as f64) / 1024.0);
        let var1: f64 = var1 * (self.calib_data.dig_t2 as f64);
        let var2: f64 = ((self.uncomp_data.temperature as f64) / 131072.0) - ((self.calib_data.dig_t1 as f64) / 8192.0);
        let var2: f64 = (var2 * var2) * (self.calib_data.dig_t3 as f64);
        self.calib_data.t_fine_float = var1 + var2;
        let temperature = (var1 + var2) / 5120.0;
        if temperature < BME280_TEMPERATURE_MIN {
            BME280_TEMPERATURE_MIN
        } else if temperature > BME280_TEMPERATURE_MAX {
            BME280_TEMPERATURE_MAX
        } else {
            temperature
        }
    }

    pub fn compensate_temperature_fixed(&mut self) -> f64 {
        let var1a: i32 = ((self.uncomp_data.temperature >> 3) as i32) - ((self.calib_data.dig_t1 as i32) << 1);
        let var1: i32 = (var1a * (self.calib_data.dig_t2 as i32)) >> 11;
        let var2a: i32 = ((self.uncomp_data.temperature >> 4) as i32) - (self.calib_data.dig_t1 as i32);
        let var2: i32 = (((var2a * var2a) >> 12) *  (self.calib_data.dig_t3 as i32)) >> 14;
        self.calib_data.t_fine_fixed = var1 + var2;
        let temperature = (self.calib_data.t_fine_fixed * 5 + 128) >> 8;
        let temperature = temperature as f64 / 100.0;
        if temperature < BME280_TEMPERATURE_MIN {
            BME280_TEMPERATURE_MIN
        } else if temperature > BME280_TEMPERATURE_MAX {
            BME280_TEMPERATURE_MAX
        } else {
            temperature
        }
    }

    #[allow(dead_code)]
    pub fn get_t_fine_float(&self) -> f64 {
        self.calib_data.t_fine_float
    }

    #[allow(dead_code)]
    pub fn get_t_fine_fixed(&self) -> i32 {
        self.calib_data.t_fine_fixed
    }

    pub fn compensate_pressure_float(&self) -> f64 {        
        let var1 = (self.calib_data.t_fine_float / 2.0) - 64000.0;
        let var2 = var1 * var1 * (self.calib_data.dig_p6 as f64) / 32768.0;
        let var2 = var2 + var1 * (self.calib_data.dig_p5 as f64) * 2.0;
        let var2 = (var2 / 4.0) + (self.calib_data.dig_p4 as f64) * 65536.0;
        let var3 = (self.calib_data.dig_p3 as f64) * var1 * var1 / 524288.0;
        let var1 = (var3 + (self.calib_data.dig_p2 as f64) * var1) / 524288.0;
        let var1 = (1.0 + var1 / 32768.0) * (self.calib_data.dig_p1 as f64);
        if var1 == 0.0 {
            // -- avoid exception caused by division by zero
            return BME280_PRESSURE_MIN
        } 
        let pressure = 1048576.0 - (self.uncomp_data.pressure as f64);
        let pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        let var1 = (self.calib_data.dig_p9 as f64) * pressure * pressure / 2147483648.0;
        let var2 = pressure * (self.calib_data.dig_p8 as f64) / 32768.0;
        let pressure = pressure + (var1 + var2 + (self.calib_data.dig_p7 as f64)) / 16.0;
        if pressure < BME280_PRESSURE_MIN {
            BME280_PRESSURE_MIN
        }
        else if pressure > BME280_PRESSURE_MAX {
            BME280_PRESSURE_MAX
        } else {
            pressure
        }
    }
    
    pub fn compensate_humidity_float(&self) -> f64 {        
         let var1 = self.calib_data.t_fine_float - 76800.0;
        let var2 = (self.calib_data.dig_h4 as f64) * 64.0 + ((self.calib_data.dig_h5 as f64) / 16384.0) * var1;
        let var3 = (self.uncomp_data.humidity as f64) - var2;
        let var4 = (self.calib_data.dig_h2 as f64) / 65536.0;
        let var5 = 1.0 + ((self.calib_data.dig_h3 as f64) / 67108864.0) * var1;
        let var6 = 1.0 + ((self.calib_data.dig_h6 as f64) / 67108864.0) * var1 * var5;
        let var6 = var3 * var4 * (var5 * var6);
        let humidity = var6 * (1.0 - (self.calib_data.dig_h1 as f64) * var6 / 524288.0); 
        if humidity > BME280_HUMIDITY_MAX {
            BME280_HUMIDITY_MAX
        }
        else if humidity < BME280_HUMIDITY_MIN {
            BME280_HUMIDITY_MIN
        } else {
            humidity
        }
    }

}