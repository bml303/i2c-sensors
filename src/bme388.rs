use i2c_linux::I2c;
#[allow(unused_imports)]
use log::{debug, info};
use std::fmt;
use std::fs::File;
use std::path::Path;
use std::{thread, time};

use crate::i2cio;

// -- chip id
const BME388_CHIP_ID: u8 = 0x50;

// -- length of multi-byte registers
const BME388_LEN_TRIMMING_COEFFICIENTS: usize = 21;
const BME388_LEN_PRESSURE_DATA: usize = 3;
const BME388_LEN_TEMPERATURE_DATA: usize = 3;
#[allow(dead_code)]
const BME388_LEN_SENSOR_TIME: usize = 3;
#[allow(dead_code)]
const BME388_LEN_FIFO_LENGTH: usize = 2;
#[allow(dead_code)]
const BME388_LEN_FIFO_WATERMARK: usize = 2;

// -- registers
const BME388_REG_CHIP_ID: u8 = 0x00;
#[allow(dead_code)]
const BME388_REG_ERRORS: u8 = 0x02;
const BME388_REG_STATUS: u8 = 0x03;
const BME388_REG_PRESSURE_DATA: u8 = 0x04;
const BME388_REG_TEMPERATURE_DATA: u8 = 0x07;
#[allow(dead_code)]
const BME388_REG_SENSOR_TIME: u8 = 0x0C;
#[allow(dead_code)]
const BME388_REG_EVENT: u8 = 0x10;
#[allow(dead_code)]
const BME388_REG_INT_STATUS: u8 = 0x11;
#[allow(dead_code)]
const BME388_REG_FIFO_LENGTH: u8 = 0x12;
#[allow(dead_code)]
const BME388_REG_FIFO_DATA: u8 = 0x14;
#[allow(dead_code)]
const BME388_REG_FIFO_WATERMARK: u8 = 0x15;
#[allow(dead_code)]
const BME388_REG_FIFO_CONFIG_1: u8 = 0x17;
#[allow(dead_code)]
const BME388_REG_FIFO_CONFIG_2: u8 = 0x18;
#[allow(dead_code)]
const BME388_REG_INT_CONTROL: u8 = 0x19;
#[allow(dead_code)]
const BME388_REG_IF_CONF: u8 = 0x1a;
const BME388_REG_POWER_CONTROL: u8 = 0x1b;
const BME388_REG_OVERSAMPLING_RATE: u8 = 0x1c;
const BME388_REG_OUTPUT_DATA_RATE: u8 = 0x1d;
const BME388_REG_CONFIG: u8 = 0x1f;
const BME388_REG_TRIMMING_COEFFICIENTS: u8 = 0x31;
const BME388_REG_CMD: u8 = 0x7e;

// -- commands
#[allow(dead_code)]
const BME388_CMD_FIFO_FLUSH: u8 = 0xb0;
const BME388_CMD_SOFT_RESET: u8 = 0xb6;

// -- other constants
const BME388_STARTUP_DELAY_MS: u64 = 2;

const BME280_PRESSURE_SENSOR_ENABLED_BIT: u8 = 0x1;
const BME280_TEMPERATURE_SENSOR_ENABLED_BIT: u8 = 0x2;
const BME280_POWER_MODE_LOW_BIT: u8 = 4;

const BME280_STATUS_CMD_READY_MASK: u8 = 0x10;
const BME280_STATUS_PRESSURE_DATA_READY_MASK: u8 = 0x20;
const BME280_STATUS_TEMPERATURE_DATA_READY_MASK: u8 = 0x40;

#[derive(Clone, Debug, PartialEq)]
pub enum BME388DeviceAddress {
    Default,
    Secondary,
}

impl Default for BME388DeviceAddress {
    fn default() -> Self {
        BME388DeviceAddress::Default
    }
}

impl BME388DeviceAddress {
    const ADDR_DEFAULT: u16 = 0x77;
    const ADDR_SECONDARY: u16 = 0x76;

    fn value(&self) -> u16 {
        match *self {
            Self::Default => Self::ADDR_DEFAULT,
            Self::Secondary => Self::ADDR_SECONDARY,
        }
    }
}

#[derive(PartialEq)]
pub enum BME388SensorPowerMode {
    Sleep,
    Forced,
    Normal
}

impl BME388SensorPowerMode {
    const POWERMODE_SLEEP: u8 = 0x00;
    const POWERMODE_FORCED: u8 = 0x01;
    const POWERMODE_NORMAL: u8 = 0x03;

    fn value(&self) -> u8 {
        match *self {
            Self::Sleep => Self::POWERMODE_SLEEP,
            Self::Forced => Self::POWERMODE_FORCED,
            Self::Normal => Self::POWERMODE_NORMAL,
        }
    }
}

impl fmt::Display for BME388SensorPowerMode {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Sleep => write!(f, "Sleep/{:#04x}", self.value()),
            Self::Forced => write!(f, "Forced/{:#04x}", self.value()),
            Self::Normal => write!(f, "Normal/{:#04x}", self.value()),
        }
    }
}

#[derive(PartialEq)]
pub enum BME388StatusPressureSensor {
    Disabled,
    Enabled,
}

impl BME388StatusPressureSensor {
    fn value(&self) -> u8 {
        match *self {
            Self::Disabled => 0,
            Self::Enabled => 1,
        }
    }
}

impl fmt::Display for BME388StatusPressureSensor {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Disabled => write!(f, "Disabled/{}", self.value()),
            Self::Enabled => write!(f, "Enabled/{}", self.value()),
        }
    }
}

#[derive(PartialEq)]
pub enum BME388StatusTemperatureSensor {
    Disabled,
    Enabled,
}

impl BME388StatusTemperatureSensor {
    fn value(&self) -> u8 {
        match *self {
            Self::Disabled => 0,
            Self::Enabled => 1,
        }
    }
}

impl fmt::Display for BME388StatusTemperatureSensor {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Disabled => write!(f, "Disabled/{}", self.value()),
            Self::Enabled => write!(f, "Enabled/{}", self.value()),
        }
    }
}

#[allow(dead_code)]
pub enum BME388OverSamplingPr {
    UltraLowX1, LowX2, StandardX4,
    HighX8, UltraHighX16, HighestX32,
}

impl BME388OverSamplingPr {
    const OSR_X1_ULTRA_LOW: u8 = 0x00;
    const OSR_X2_LOW: u8 = 0x01;
    const OSR_X4_STANDARD: u8 = 0x02;
    const OSR_X8_HIGH: u8 = 0x03;
    const OSR_X16_ULTRA_HIGH: u8 = 0x04;
    const OSR_X32_HIGHEST: u8 = 0x05;

    fn value(&self) -> u8 {
        match *self {
            Self::UltraLowX1 => Self::OSR_X1_ULTRA_LOW,
            Self::LowX2 => Self::OSR_X2_LOW,
            Self::StandardX4 => Self::OSR_X4_STANDARD,
            Self::HighX8 => Self::OSR_X8_HIGH,
            Self::UltraHighX16 => Self::OSR_X16_ULTRA_HIGH,
            Self::HighestX32 => Self::OSR_X32_HIGHEST,
        }
    }
}

impl fmt::Display for BME388OverSamplingPr {
    // This trait requires `fmt` with this exact signature.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::UltraLowX1 => write!(f, "UltraLow X1/{:#04x}", self.value()),
            Self::LowX2 => write!(f, "Low X2/{:#05x}", self.value()),
            Self::StandardX4 => write!(f, "Standard X4/{:#04x}", self.value()),
            Self::HighX8 => write!(f, "High X8/{:#05x}", self.value()),
            Self::UltraHighX16 => write!(f, "UltraHigh X16/{:#04x}", self.value()),
            Self::HighestX32 => write!(f, "Highest X32/{:#04x}", self.value()),
        }
    }
}

pub enum BME388OverSamplingTp {
    X1, X2, X4, X8, X16, X32,
}

impl BME388OverSamplingTp {
    const OSR_X1: u8 = 0x00;
    const OSR_X2: u8 = 0x01;
    const OSR_X4: u8 = 0x02;
    const OSR_X8: u8 = 0x03;
    const OSR_X16: u8 = 0x04;
    const OSR_X32: u8 = 0x05;

    fn value(&self) -> u8 {
        match *self {
            Self::X1 => Self::OSR_X1,
            Self::X2 => Self::OSR_X2,
            Self::X4 => Self::OSR_X4,
            Self::X8 => Self::OSR_X8,
            Self::X16 => Self::OSR_X16,
            Self::X32 => Self::OSR_X32,
        }
    }
}

impl fmt::Display for BME388OverSamplingTp {
    // This trait requires `fmt` with this exact signature.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::X1 => write!(f, "X1/{:#04x}", self.value()),
            Self::X2 => write!(f, "X2/{:#04x}", self.value()),
            Self::X4 => write!(f, "X4/{:#04x}", self.value()),
            Self::X8 => write!(f, "X8/{:#04x}", self.value()),
            Self::X16 => write!(f, "X16/{:#04x}", self.value()),
            Self::X32 => write!(f, "X32/{:#04x}", self.value()),            
        }
    }
}

pub enum BME388OutputDataRate {
    Odr200, Odr100, Odr50, Odr25, Odr12p5, 
    Odr6p25, Odr3p1, Odr1p5, Odr0p78, Odr0p39, 
    Odr0p2, Odr0p1, Odr0p05, Odr0p02, Odr0p01,
    Odr0p006, Odr0p003, Odr0p0015,
}

impl BME388OutputDataRate {
    fn value(&self) -> u8 {
        match *self {
            Self::Odr200 => 0x00,
            Self::Odr100 => 0x01,
            Self::Odr50 => 0x02,
            Self::Odr25 => 0x03,
            Self::Odr12p5 => 0x04,
            Self::Odr6p25 => 0x05,
            Self::Odr3p1 => 0x06,
            Self::Odr1p5 => 0x07,
            Self::Odr0p78 => 0x08,
            Self::Odr0p39 => 0x09,
            Self::Odr0p2 => 0x0a,
            Self::Odr0p1 => 0x0b,
            Self::Odr0p05 => 0x0c,
            Self::Odr0p02 => 0x0d,
            Self::Odr0p01 => 0x0e,
            Self::Odr0p006 => 0x0f,
            Self::Odr0p003 => 0x10,
            Self::Odr0p0015 => 0x11,
        }
    }
}

pub enum BME388IrrFilter {
    Off, Coef1, Coef3, Coef7, Coef15, Coef31, Coef63, Coef127,
}

impl BME388IrrFilter {
    const COEF_0: u8 = 0x00;
    const COEF_1: u8 = 0x01;
    const COEF_3: u8 = 0x02;
    const COEF_7: u8 = 0x03;
    const COEF_15: u8 = 0x04;
    const COEF_31: u8 = 0x05;
    const COEF_63: u8 = 0x06;
    const COEF_127: u8 = 0x07;

    fn value(&self) -> u8 {
        match *self {
            Self::Off => Self::COEF_0,
            Self::Coef1 => Self::COEF_1,
            Self::Coef3 => Self::COEF_3,
            Self::Coef7 => Self::COEF_7,
            Self::Coef15 => Self::COEF_15,
            Self::Coef31 => Self::COEF_31,
            Self::Coef63 => Self::COEF_63,
            Self::Coef127 => Self::COEF_127,
        }
    }
}

#[derive(PartialEq)]
pub enum BME388StatusCommandDecoder {
    NotReady,
    Ready
}

impl fmt::Display for BME388StatusCommandDecoder {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::NotReady => write!(f, "Command decoder not ready"),
            Self::Ready => write!(f, "Command decoder ready"),
        }
    }
}

#[derive(PartialEq)]
pub enum BME388StatusPressureData {
    NotReady,
    Ready
}

impl fmt::Display for BME388StatusPressureData {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::NotReady => write!(f, "Pressure data not ready"),
            Self::Ready => write!(f, "Pressure data ready"),
        }
    }
}

#[derive(PartialEq)]
pub enum BME388StatusTemperatureData {
    NotReady,
    Ready
}

impl fmt::Display for BME388StatusTemperatureData {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::NotReady => write!(f, "Temperature data not ready"),
            Self::Ready => write!(f, "Temperature data ready"),
        }
    }
}

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


pub struct BME388 {
    // -- i2c bus
    i2c: I2c<File>,
    // -- device address.
    device_addr: BME388DeviceAddress,
    // -- calibration data
    calib_data: CalibData,
    // -- uncompensated data
    raw_data: RawData,    
}

impl BME388 {

    pub fn new(i2c_bus_path: &Path, device_addr: BME388DeviceAddress, 
        osr_p: BME388OverSamplingPr, osr_t: BME388OverSamplingTp, 
        irr_filter: BME388IrrFilter, odr: BME388OutputDataRate) -> Result<BME388, std::io::Error> {
        // -- get the bus
        let mut i2c = i2cio::get_bus(i2c_bus_path)?;
        // -- set device address
        i2cio::set_slave(&mut i2c, device_addr.value())?;
        // -- check if device is available by reading chip id
        let chip_id = i2cio::read_byte(&mut i2c, BME388_REG_CHIP_ID)?;
        if chip_id != BME388_CHIP_ID {
            let errmsg = format!("Found unknown chip id '{chip_id:#04x}', expected '{BME388_CHIP_ID:#04x}'");
            return Err(std::io::Error::new(std::io::ErrorKind::Other, errmsg))
        }
        debug!("Got chip id: {chip_id:#x}");
        // -- do a soft reset since it's in an unknown state
        Self::soft_reset(&mut i2c)?;
        // -- get calibration data
        let calib_data = Self::get_calib_data(&mut i2c)?;
        // -- return initialized structure
        let mut bme388 = BME388 {
            i2c,
            device_addr,
            calib_data,
            raw_data: Default::default(),
        };
        bme388.set_osr_pressure_temperature(osr_p, osr_t)?;
        bme388.set_irr_filter(irr_filter)?;
        bme388.set_output_data_rate(odr)?;
        Ok(bme388)
    }

    #[allow(dead_code)]
    pub fn get_device_addr(&self) -> BME388DeviceAddress {
        self.device_addr.clone()
    }

    fn soft_reset(i2c: &mut I2c<File>) -> Result<(), std::io::Error> {
        // -- initiate soft reset
        debug!("Initiating soft reset");
        i2cio::write_byte(i2c, BME388_REG_CMD, BME388_CMD_SOFT_RESET)?;
        // -- wait for the device to startup
        let startup_delay = time::Duration::from_millis(BME388_STARTUP_DELAY_MS);
        thread::sleep(startup_delay);
        Ok(())
    }

    pub fn set_output_data_rate(&mut self, subdiv_factor: BME388OutputDataRate) -> Result<(), std::io::Error> {
        let reg_val = subdiv_factor.value();
        debug!("Setting register BME388_REG_OUTPUT_DATA_RATE {BME388_REG_OUTPUT_DATA_RATE:#x} to value {reg_val:#010b}");
        // -- write it back
        i2cio::write_byte(&mut self.i2c, BME388_REG_OUTPUT_DATA_RATE, reg_val)
    }

    pub fn set_irr_filter(&mut self, irr_filter: BME388IrrFilter) -> Result<(), std::io::Error> {
        let reg_val = irr_filter.value();
        debug!("Setting register BME388_REG_CONFIG {BME388_REG_CONFIG:#x} to value {reg_val:#010b}");
        // -- write it back
        i2cio::write_byte(&mut self.i2c, BME388_REG_CONFIG, reg_val)
    }

    pub fn set_sensor_mode(&mut self, pwr_mode : BME388SensorPowerMode, 
        enable_pressure: BME388StatusPressureSensor, enable_temperature: BME388StatusTemperatureSensor) -> Result<(), std::io::Error> {
        let reg_val = pwr_mode.value() << BME280_POWER_MODE_LOW_BIT | enable_temperature.value() << 1 | enable_pressure.value();
        debug!("Setting register BME388_REG_POWER_CONTROL {BME388_REG_POWER_CONTROL:#x} to value {reg_val:#010b}");
        // -- write it back
        i2cio::write_byte(&mut self.i2c, BME388_REG_POWER_CONTROL, reg_val)
    }

    pub fn get_sensor_mode(&mut self) -> Result<(BME388SensorPowerMode, BME388StatusPressureSensor, BME388StatusTemperatureSensor), std::io::Error> {
        // -- read current value of BME388_REG_POWER_CONTROL
        let reg_val = i2cio::read_byte(&mut self.i2c, BME388_REG_POWER_CONTROL)?;
        debug!("Got register BME388_REG_POWER_CONTROL {BME388_REG_POWER_CONTROL:#x} value {reg_val:#010b}");
        let pressure_enabled = match (reg_val & BME280_PRESSURE_SENSOR_ENABLED_BIT) > 0 {
            false => BME388StatusPressureSensor::Disabled,
            true => BME388StatusPressureSensor::Enabled,
        };
        let temperature_enabled = match (reg_val & BME280_TEMPERATURE_SENSOR_ENABLED_BIT) > 0 {
            false => BME388StatusTemperatureSensor::Disabled,
            true => BME388StatusTemperatureSensor::Enabled,  
        };
        let sensor_mode = match reg_val >> BME280_POWER_MODE_LOW_BIT {
            0 => BME388SensorPowerMode::Sleep,
            1..=2 => BME388SensorPowerMode::Forced,
            _ => BME388SensorPowerMode::Normal,
        };
        Ok((sensor_mode, pressure_enabled, temperature_enabled))
    }

    pub fn get_status(&mut self) 
        -> Result<(BME388StatusCommandDecoder, BME388StatusPressureData, BME388StatusTemperatureData), std::io::Error> {
        // -- read current value of BME388_REG_POWER_CONTROL
        let reg_val = i2cio::read_byte(&mut self.i2c, BME388_REG_STATUS)?;
        let cmd_decoder_ready = match (reg_val & BME280_STATUS_CMD_READY_MASK) > 0 {
            false => BME388StatusCommandDecoder::NotReady,
            true => BME388StatusCommandDecoder::Ready,
        };
        let pressure_data_ready = match (reg_val & BME280_STATUS_PRESSURE_DATA_READY_MASK) > 0 {
            false => BME388StatusPressureData::NotReady,
            true => BME388StatusPressureData::Ready,
        };
        let temperature_data_ready = match (reg_val & BME280_STATUS_TEMPERATURE_DATA_READY_MASK) > 0 {
            false => BME388StatusTemperatureData::NotReady,
            true => BME388StatusTemperatureData::Ready,
        };
        Ok((cmd_decoder_ready, pressure_data_ready, temperature_data_ready))
    }

    fn concat_bytes(msb: u8, lsb: u8) -> u16 {
        ((msb as u16) << 8) | (lsb as u16)
    }

    fn get_calib_data(i2c: &mut I2c<File>) -> Result<CalibData, std::io::Error> {
        // -- get temperature and pressure calibration data
        let mut reg_data: [u8; BME388_LEN_TRIMMING_COEFFICIENTS] = [0; BME388_LEN_TRIMMING_COEFFICIENTS];
        let _bytes_read = i2c.i2c_read_block_data(BME388_REG_TRIMMING_COEFFICIENTS, &mut reg_data)?;
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
        const DATA_LEN: usize = BME388_LEN_PRESSURE_DATA + BME388_LEN_TEMPERATURE_DATA;
        let mut reg_data: [u8; DATA_LEN] = [0; DATA_LEN];
        let _bytes_read = self.i2c.i2c_read_block_data(BME388_REG_PRESSURE_DATA, &mut reg_data)?;
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
        let mut reg_data: [u8; BME388_LEN_PRESSURE_DATA] = [0; BME388_LEN_PRESSURE_DATA];
        let _bytes_read = self.i2c.i2c_read_block_data(BME388_REG_PRESSURE_DATA, &mut reg_data)?;
        let pressure = (reg_data[2] as u32) << 16 | (reg_data[1] as u32) << 8 | (reg_data[0] as u32);
        debug!("Got raw pressure: {pressure}");
        Ok(pressure)
    }

    pub fn get_temperature_raw(&mut self) -> Result<u32, std::io::Error> {
        // -- get temperature and pressure data
        let mut reg_data: [u8; BME388_LEN_TEMPERATURE_DATA] = [0; BME388_LEN_TEMPERATURE_DATA];
        let _bytes_read = self.i2c.i2c_read_block_data(BME388_REG_TEMPERATURE_DATA, &mut reg_data)?;
        let temperature = (reg_data[2] as u32) << 16 | (reg_data[1] as u32) << 8 | (reg_data[0] as u32);
        debug!("Got raw temperature: {temperature}");
        Ok(temperature)
    }

    pub fn set_osr_pressure_temperature(&mut self, osr_p: BME388OverSamplingPr, osr_t : BME388OverSamplingTp) -> Result<(), std::io::Error> {
        // -- write oversampling for pressure and temperature
        let reg_val = osr_t.value() << 3 | osr_p.value();
        debug!("Setting register BME388_REG_OVERSAMPLING_RATE {BME388_REG_OVERSAMPLING_RATE:#x} to value {reg_val:#010b} / {osr_p} for pressure, {osr_t} for temperature");
        i2cio::write_byte(&mut self.i2c, BME388_REG_OVERSAMPLING_RATE, reg_val)
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