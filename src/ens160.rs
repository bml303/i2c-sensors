use i2c_linux::I2c;
#[allow(unused_imports)]
use log::{debug, error, log_enabled, info, Level};
use std::fmt;
use std::fs::File;

use crate::i2cio;

const ENS160_DEV_ADDR: u16 = 0x53;
const ENS160_REG_PART_ID: u8 = 0x00;
const ENS160_REG_OP_MODE: u8 = 0x10;
const ENS160_REG_TEMP_IN: u8 = 0x13;
const ENS160_REG_RH_IN: u8 = 0x15;
const ENS160_REG_DEVICE_STATUS: u8 = 0x20;
const ENS160_REG_DATA_AQI: u8 = 0x21;
const ENS160_REG_DATA_TVOC: u8 = 0x22;
const ENS160_REG_DATA_ECO2: u8 = 0x24;

#[allow(dead_code)]
const ENS160_OP_MODE_DEEP_SLEEP: u8 = 0x00;

#[allow(dead_code)]
const ENS160_OP_MODE_IDLE: u8 = 0x01;
const ENS160_OP_MODE_OPERATIONAL: u8 = 0x02;


pub enum Ens160Validity {
    OperatingOk,
    WarmUp,
    InitialStartUp,
    NoValidOutput,
}

impl Ens160Validity {
    const ENS160_VALIDITY_OPERATING_OK: u8 = 0x00;
    const ENS160_VALIDITY_WARM_UP: u8 = 0x01;
    const ENS160_VALIDITY_INITIAL_START_UP: u8 = 0x03;
    #[allow(dead_code)]
    const ENS160_VALIDITY_NO_VALID_OUTPUT: u8 = 0x04;

    fn from(val: u8) -> Self {
        match val {
            Self::ENS160_VALIDITY_OPERATING_OK => Self::OperatingOk,
            Self::ENS160_VALIDITY_WARM_UP => Self::WarmUp,
            Self::ENS160_VALIDITY_INITIAL_START_UP => Self::InitialStartUp,
            _ => Self::NoValidOutput,
        }
    }
}

impl fmt::Display for Ens160Validity {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::OperatingOk => write!(f, "OperatingOk"),
            Self::WarmUp => write!(f, "WarmUp"),
            Self::InitialStartUp => write!(f, "InitialStartUp"),
            Self::NoValidOutput => write!(f, "NoValidOutput"),
        }
    }
}

pub enum Ens160AirQualityIndex {
    Excellent,
    Good,
    FairModerate,
    Poor,
    BadUnhealthy,
}

impl Ens160AirQualityIndex {
    const ENS160_AQI_EXCELLENT: u8 = 1;
    const ENS160_AQI_GOOD: u8 = 2;
    const ENS160_AQI_FAIRMODERATE: u8 = 3;
    const ENS160_AQI_POOR: u8 = 4;
    #[allow(dead_code)]
    const ENS160_AQI_BADUNHEALTHY: u8 = 5;    

    fn from(val: u8) -> Self {
        match val {
            Self::ENS160_AQI_EXCELLENT => Self::Excellent,
            Self::ENS160_AQI_GOOD => Self::Good,
            Self::ENS160_AQI_FAIRMODERATE => Self::FairModerate,
            Self::ENS160_AQI_POOR => Self::Poor,
            _ => Self::BadUnhealthy,
        }
    }
}

impl fmt::Display for Ens160AirQualityIndex {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Excellent => write!(f, "Excellent"),
            Self::Good => write!(f, "Good"),
            Self::FairModerate => write!(f, "Fair/Moderate"),
            Self::Poor => write!(f, "Poor"),
            Self::BadUnhealthy => write!(f, "Bad/Unhealthy"),
        }
    }
}

pub enum Ens160EquivalentCO2 {
    Excellent,
    Good,
    FairModerate,
    Poor,
    BadUnhealthy,
}

impl Ens160EquivalentCO2 {    
    fn from(val: u16) -> Self {
        match val {
            0..=600 => Self::Excellent,
            601..=800 => Self::Good,
            801..=1000 => Self::FairModerate,
            1001..=1500 => Self::Poor,
            _ => Self::BadUnhealthy,
        }
    }
}

impl fmt::Display for Ens160EquivalentCO2 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Excellent => write!(f, "Excellent"),
            Self::Good => write!(f, "Good"),
            Self::FairModerate => write!(f, "Fair/Moderate"),
            Self::Poor => write!(f, "Poor"),
            Self::BadUnhealthy => write!(f, "Bad/Unhealthy"),
        }
    }
}

pub struct ENS160 {
    device_id: u16,
}

impl ENS160 {

    pub fn new(i2c: &mut I2c<File>) -> Result<ENS160, std::io::Error> {
        Self::set_slave(i2c)?;
        let device_id = Self::read_device_id(i2c)?;
        debug!("ENS160 device id: {device_id:#06x}");
        let op_mode = Self::read_op_mode(i2c)?;
        debug!("ENS160 op mode: {op_mode:#04x}");

        if op_mode != ENS160_OP_MODE_OPERATIONAL {
            debug!("Setting ENS160 op mode to operational");
            Self::set_op_mode_operational(i2c)?;
            let op_mode = Self::read_op_mode(i2c)?;
            debug!("ENS160 op mode: {op_mode:#04x}");
        }

        // -- return initialized structure
        Ok(ENS160 {
            device_id,
        })
    }

    pub fn set_slave(i2c: &mut I2c<File>) -> Result<(), std::io::Error> {
        i2cio::set_slave(i2c, ENS160_DEV_ADDR)
    }

    fn read_device_id(i2c: &mut I2c<File>) -> Result<u16, std::io::Error> {
        i2cio::read_word(i2c, ENS160_REG_PART_ID)
    }

    fn read_op_mode(i2c: &mut I2c<File>) -> Result<u8, std::io::Error> {
        i2cio::read_byte(i2c, ENS160_REG_OP_MODE)
    }

    fn set_op_mode_operational(i2c: &mut I2c<File>) -> Result<(), std::io::Error> {
        i2cio::write_byte(i2c, ENS160_REG_OP_MODE, ENS160_OP_MODE_OPERATIONAL)
    }

    pub fn get_device_id(&self) -> u16 {
        self.device_id
    }

    fn get_device_status(&self, i2c: &mut I2c<File>) -> Result<u8, std::io::Error> {
        let device_status = i2cio::read_byte(i2c, ENS160_REG_DEVICE_STATUS)?;
        debug!("ENS160 device status: {device_status:#010b}");
        Ok(device_status)
    }

    pub fn get_validity(&self, i2c: &mut I2c<File>) -> Result<Ens160Validity, std::io::Error> {
        let device_status = self.get_device_status(i2c)?;
        let validity_code = (device_status & 0b00001100) >> 2;
        let validity = Ens160Validity::from(validity_code);
        debug!("ENS160 validity: {validity_code:#04b} => {validity}");
        Ok(validity)
    }

    pub fn get_air_quality_index(&self, i2c: &mut I2c<File>) -> Result<Ens160AirQualityIndex, std::io::Error> {
        let aqi_code = i2cio::read_byte(i2c, ENS160_REG_DATA_AQI)?;
        let aqi = Ens160AirQualityIndex::from(aqi_code);
        debug!("END160 Air Quality Index: {aqi_code} => {aqi}");
        Ok(aqi)
    }

    pub fn get_total_volatile_organic_compounds(&self, i2c: &mut I2c<File>) -> Result<u16, std::io::Error> {
        let data_tvoc = i2cio::read_word(i2c, ENS160_REG_DATA_TVOC)?;
        debug!("END160 TVOC Concentration (ppb): {data_tvoc}");
        Ok(data_tvoc)
    }

    pub fn get_equivalent_co2(&self, i2c: &mut I2c<File>) -> Result<Ens160EquivalentCO2, std::io::Error> {
        let eco2_code = i2cio::read_word(i2c, ENS160_REG_DATA_ECO2)?;
        let eco2 = Ens160EquivalentCO2::from(eco2_code);
        debug!("END160 Equivalent CO2 Concentration (ppm): {eco2_code} => {eco2}");
        Ok(eco2)
    }
    
    pub fn get_relative_humidity(&self, i2c: &mut I2c<File>) -> Result<u16, std::io::Error> {
        let data_rh = i2cio::read_word(i2c, ENS160_REG_RH_IN)?;
        debug!("END160 Relative humidity: {data_rh:#04x}");
        Ok(data_rh)
    }

    pub fn set_relative_humidity(&self, i2c: &mut I2c<File>, rh: f64) -> Result<(), std::io::Error> {
        let rh_adj = (rh * 512.0) as u16;
        i2cio::write_word(i2c, ENS160_REG_RH_IN, rh_adj)
    }

    pub fn get_temperature(&self, i2c: &mut I2c<File>) -> Result<u16, std::io::Error> {
        let data_temperature = i2cio::read_word(i2c, ENS160_REG_TEMP_IN)?;
        debug!("END160 Temperature: {data_temperature:#04x}");
        Ok(data_temperature)
    }

    pub fn set_temperature(&self, i2c: &mut I2c<File>, temp: f64) -> Result<(), std::io::Error> {
        let temp_adj =  ((temp + 273.15) * 64.0) as u16;
        i2cio::write_word(i2c, ENS160_REG_TEMP_IN, temp_adj)
    }

}