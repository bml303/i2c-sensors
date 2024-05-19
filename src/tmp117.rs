use i2c_linux::I2c;
#[allow(unused_imports)]
use log::{debug, error, log_enabled, info, Level};
use std::fs::File;
use std::path::Path;
use std::{thread, time};

use crate::i2cio;

const TMP117_REG_TEMPERATURE: u8 = 0x00;
const TMP117_REG_CONFIGURATION: u8 = 0x01;
#[allow(dead_code)]
const TMP117_REG_HIGH_LIMIT: u8 = 0x02;
#[allow(dead_code)]
const TMP117_REG_LOW_LIMIT: u8 = 0x03;
#[allow(dead_code)]
const TMP117_REG_EEPROM_UNLOCK: u8 = 0x04;
#[allow(dead_code)]
const TMP117_REG_EEPROM1: u8 = 0x05;
#[allow(dead_code)]
const TMP117_REG_EEPROM2: u8 = 0x06;
const TMP117_REG_TEMPERATURE_OFFSET: u8 = 0x07;
#[allow(dead_code)]
const TMP117_REG_EEPROM3: u8 = 0x08;
const TMP117_REG_DEVICE_ID: u8 = 0x0f;

const TMP117_DEVICE_ID: u16 = 0x117;
const TMP117_DEVICE_ID_MASK: u16 = 0xfff;
const TMP117_REVISION_SHIFT_RIGHT: u8 = 12;
const TMP117_CONFIG_DATA_READY_BIT: u16 = 0x2000;
const TMP117_CONFIG_SOFT_RESET_BIT: u16 = 0x2;
#[allow(dead_code)]
const TMP117_CONFIG_MODE_MASK: u16 = 0x3ff;
const TMP117_CONFIG_MODE_CONV_AVG_MASK: u16 = 0x1f;
const TMP117_CONFIG_MODE_SHIFT_LEFT: u8 = 10;
const TMP117_CONFIG_CONVERSION_CYCLE_SHIFT_LEFT: u8 = 7;
const TMP117_CONFIG_AVERAGING_SHIFT_LEFT: u8 = 5;
#[allow(dead_code)]
const TMP117_STARTUP_DELAY_MS: u64 = 2;

const TMP117_TEMPERATURE_FACTOR: f64 = 0.0078125;

#[allow(dead_code)]
#[derive(Clone, Debug, PartialEq)]
pub enum Tmp117DeviceAddress {
    Default,    
    Alt1,    
    Alt2,    
    Alt3,
}

impl Default for Tmp117DeviceAddress {
    fn default() -> Self {
        Tmp117DeviceAddress::Default
    }
}

impl Tmp117DeviceAddress {
    const DEVICE_ADDR_DEFAULT: u16 = 0x48;
    const DEVICE_ADDR_ALT1: u16 = 0x49;
    const DEVICE_ADDR_ALT2: u16 = 0x4A;
    const DEVICE_ADDR_ALT3: u16 = 0x4B;

    fn value(&self) -> u16 {
        match *self {
            Self::Default => Self::DEVICE_ADDR_DEFAULT,
            Self::Alt1 => Self::DEVICE_ADDR_ALT1,
            Self::Alt2 => Self::DEVICE_ADDR_ALT2,
            Self::Alt3 => Self::DEVICE_ADDR_ALT3,
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, PartialEq)]
pub enum Tmp117SensorMode {
    ModeContinuousConversion,    
    ModeShutDown,
    ModeOneShot,
}

impl Default for Tmp117SensorMode {
    fn default() -> Self {
        Tmp117SensorMode::ModeContinuousConversion
    }
}

impl Tmp117SensorMode {
    pub(super) const TMP117_MODE_CONTINUOUS_CONVERSATION: u16 = 0x00;
    pub(super) const TMP117_MODE_SHUT_DOWN: u16 = 0x01;
    pub(super) const TMP117_MODE_ONE_SHOT: u16 = 0x3;

    fn value(&self) -> u16 {
        match *self {
            Self::ModeContinuousConversion => Self::TMP117_MODE_CONTINUOUS_CONVERSATION,
            Self::ModeShutDown => Self::TMP117_MODE_SHUT_DOWN,
            Self::ModeOneShot => Self::TMP117_MODE_ONE_SHOT,
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, PartialEq)]
pub enum Tmp117ConversionCycleTime {
    Shortest,
    Shorter,
    Short,
    Medium,
    Ms1000,
    Ms4000,
    Ms8000,
    Ms16000,
}

impl Tmp117ConversionCycleTime {
    // -- --------------------------------------------------------------
    // -- medium to shortest depend on averaging setting
    // -- --------------------------------------------------------------
    // -- CCT      | AVG None   | AVG 8 Cnv  | AVG 32 Cnv | AVG 64 Cnv |
    // -- Shortest | 15.5ms     | 125ms      | 500ms      | 1000ms     |
    // -- Shorter  | 125ms      | 125ms      | 500ms      | 1000ms     |
    // -- Short    | 250ms      | 250ms      | 500ms      | 1000ms     |
    // -- Medium   | 500ms      | 500ms      | 500ms      | 1000ms     |
    // -- --------------------------------------------------------------
    pub(super) const TMP117_CONVERSION_CYCLE_TIME_SHORTEST: u16 = 0x00;
    pub(super) const TMP117_CONVERSION_CYCLE_TIME_SHORTER: u16 = 0x01;
    pub(super) const TMP117_CONVERSION_CYCLE_TIME_SHORT: u16 = 0x02;
    pub(super) const TMP117_CONVERSION_CYCLE_TIME_MEDIUM: u16 = 0x03;
    pub(super) const TMP117_CONVERSION_CYCLE_TIME_MS_1000: u16 = 0x04;
    pub(super) const TMP117_CONVERSION_CYCLE_TIME_MS_4000: u16 = 0x05;
    pub(super) const TMP117_CONVERSION_CYCLE_TIME_MS_8000: u16 = 0x06;
    pub(super) const TMP117_CONVERSION_CYCLE_TIME_MS_16000: u16 = 0x07;

    fn value(&self) -> u16 {
        match *self {
            Self::Shortest => Self::TMP117_CONVERSION_CYCLE_TIME_SHORTEST,
            Self::Shorter => Self::TMP117_CONVERSION_CYCLE_TIME_SHORTER,
            Self::Short => Self::TMP117_CONVERSION_CYCLE_TIME_SHORT,
            Self::Medium => Self::TMP117_CONVERSION_CYCLE_TIME_MEDIUM,
            Self::Ms1000 => Self::TMP117_CONVERSION_CYCLE_TIME_MS_1000,
            Self::Ms4000 => Self::TMP117_CONVERSION_CYCLE_TIME_MS_4000,
            Self::Ms8000 => Self::TMP117_CONVERSION_CYCLE_TIME_MS_8000,
            Self::Ms16000 => Self::TMP117_CONVERSION_CYCLE_TIME_MS_16000,
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, PartialEq)]
pub enum Tmp117Averaging {
    NoAveraging,
    Averaging8Conversions,
    Averaging32Conversions,
    Averaging64Conversions,    
}


impl Tmp117Averaging {
    pub(super) const TMP117_AVERAGING_NONE: u16 = 0x00;
    pub(super) const TMP117_AVERAGING_8_CONVERSIONS: u16 = 0x01;
    pub(super) const TMP117_AVERAGING_32_CONVERSIONS: u16 = 0x02;
    pub(super) const TMP117_AVERAGING_64_CONVERSIONS: u16 = 0x03;

    fn value(&self) -> u16 {
        match *self {
            Self::NoAveraging => Self::TMP117_AVERAGING_NONE,
            Self::Averaging8Conversions => Self::TMP117_AVERAGING_8_CONVERSIONS,
            Self::Averaging32Conversions => Self::TMP117_AVERAGING_32_CONVERSIONS,
            Self::Averaging64Conversions => Self::TMP117_AVERAGING_64_CONVERSIONS,
        }
    }
}

#[allow(dead_code)]
pub struct TMP117 {
    // -- i2c bus
    i2c: I2c<File>,
    // -- device address.
    device_addr: Tmp117DeviceAddress,
    // -- device id
    device_id: u16,
    // -- device revision
    device_rev: u8,
}

impl TMP117
{
    pub fn new(i2c_bus_path: &Path, device_addr: Tmp117DeviceAddress, sensor_mode: &Tmp117SensorMode, 
        conversion_cycle: &Tmp117ConversionCycleTime, averaging: &Tmp117Averaging) -> Result<Self, std::io::Error> {
        // -- get the bus
        let mut i2c = i2cio::get_bus(i2c_bus_path)?;
        // -- set device address
        i2cio::set_slave(&mut i2c, device_addr.value())?;
        // -- check if device is available by reading id and revision
        let (device_id, device_rev) = Self::read_device_id_and_revision(&mut i2c)?;
        if device_id != TMP117_DEVICE_ID {
            let errmsg = format!("Found unknown device id '{device_id:#06x}', expected '{TMP117_DEVICE_ID:#06x}'");
            return Err(std::io::Error::new(std::io::ErrorKind::Other, errmsg))
        }
        // -- do a soft reset since it's in an unknown state
        Self::soft_reset(&mut i2c)?;
        // -- set the desired mode
        Self::set_sensor_mode_internal(&mut i2c, &sensor_mode, &conversion_cycle, &averaging)?;
        // -- ready to measure steady
        let tmp117 = TMP117 {
            i2c,
            device_addr,
            device_id,
            device_rev,
        };
        Ok(tmp117)
    }    

    fn read_device_id_and_revision(i2c: &mut I2c<File>) -> Result<(u16, u8), std::io::Error> {
        // let mut reg_val: [u8; 2] = [0, 0];
        // let _bytes_read = i2c.i2c_read_block_data(TMP117_REG_DEVICE_ID, &mut reg_val)?;
        // debug!("TMP117 device id register: {reg_val:#?}, byted read {_bytes_read}");
        // let reg_val = (reg_val[0] as u16) << 8 | (reg_val[1] as u16);
        // -- read the 16 bit (word) device_id register
        let reg_val = i2cio::read_word(i2c, TMP117_REG_DEVICE_ID)?;
        // -- TMP117 sends most significant byte first so a swap is required
        let reg_val = reg_val.swap_bytes();
        //let reg_val = reg_val >> 8 | ((reg_val & 0xf) << 8);
        debug!("TMP117 device id register: {reg_val:#018b}");
        let device_id = reg_val & TMP117_DEVICE_ID_MASK;
        let device_rev = (reg_val >> TMP117_REVISION_SHIFT_RIGHT) as u8;
        Ok((device_id, device_rev))
    }

    #[allow(dead_code)]
    pub fn get_device_addr(&self) -> Tmp117DeviceAddress {
        self.device_addr.clone()
    }

    pub fn get_device_id(&self) -> u16 {
        self.device_id
    }

    pub fn get_device_revision(&self) -> u8 {
        self.device_rev
    }    

    fn soft_reset(i2c: &mut I2c<File>) -> Result<(), std::io::Error> {
        let reg_val = TMP117_CONFIG_SOFT_RESET_BIT;
        // -- TMP117 expects most significant byte first so a swap is required
        let reg_val = reg_val.swap_bytes();
        i2cio::write_word(i2c, TMP117_REG_CONFIGURATION, reg_val)?;
        // -- wait for the device to startup
        let startup_delay = time::Duration::from_millis(TMP117_STARTUP_DELAY_MS);
        thread::sleep(startup_delay);
        Ok(())
    }

    fn set_sensor_mode_internal(i2c: &mut I2c<File>, sensor_mode: &Tmp117SensorMode, 
        conversion_cycle: &Tmp117ConversionCycleTime, averaging: &Tmp117Averaging) -> Result<(), std::io::Error> {
        // -- read the 16 bit (word) config register
        let reg_val = i2cio::read_word(i2c, TMP117_REG_CONFIGURATION)?;
        // -- TMP117 sends most significant byte first so a swap is required
        let reg_val = reg_val.swap_bytes();
        debug!("TMP117 config register: {reg_val:#018b}");
        // -- keep bit 0 - 4 as is
        let reg_val_masked = reg_val & TMP117_CONFIG_MODE_CONV_AVG_MASK;
        debug!("TMP117 reg value masked: {reg_val_masked:#018b}");
        // -- prepare mode bits
        let mode_bits = sensor_mode.value() << TMP117_CONFIG_MODE_SHIFT_LEFT;
        // -- prepare conversion cycle bits
        let conversion_cycle_bits = conversion_cycle.value() << TMP117_CONFIG_CONVERSION_CYCLE_SHIFT_LEFT;
        // -- prepare averaging bits
        let averaging_bits = averaging.value() << TMP117_CONFIG_AVERAGING_SHIFT_LEFT;
        debug!("TMP117 mode bits: {mode_bits:#018b}, conversion cycle bits: {conversion_cycle_bits:#018b}, averaging bits: {averaging_bits:#018b}");
        let reg_val = reg_val_masked | mode_bits | conversion_cycle_bits | averaging_bits;
        debug!("TMP117 change config register to: {reg_val:#018b}");
        // -- TMP117 expects most significant byte first so a swap is required
        let reg_val = reg_val.swap_bytes();
        i2cio::write_word(i2c, TMP117_REG_CONFIGURATION, reg_val)
    } 

    pub fn set_sensor_mode(&mut self, sensor_mode: &Tmp117SensorMode, 
        conversion_cycle: &Tmp117ConversionCycleTime, averaging: &Tmp117Averaging) -> Result<(), std::io::Error> {
        Self::set_sensor_mode_internal(&mut self.i2c, &sensor_mode, &conversion_cycle, &averaging)
    }

    // pub fn get_config(&mut self) -> Result<u16, std::io::Error> {
    //     // -- read the 16 bit (word) config register
    //     let reg_val = i2cio::read_word(&mut self.i2c, TMP117_REG_CONFIGURATION)?;
    //     // -- TMP117 sends most significant byte first so a swap is required
    //     let reg_val = reg_val.swap_bytes();
    //     debug!("TMP117 config register: {reg_val:#018b}");
    //     Ok(reg_val)
    // }

    pub fn is_data_ready(&mut self) -> Result<bool, std::io::Error> {
        // -- read the 16 bit (word) config register
        let reg_val = i2cio::read_word(&mut self.i2c, TMP117_REG_CONFIGURATION)?;
        // -- TMP117 sends most significant byte first so a swap is required
        let reg_val = reg_val.swap_bytes();
        debug!("TMP117 config register: {reg_val:#018b}");
        let is_data_ready = (reg_val & TMP117_CONFIG_DATA_READY_BIT) > 0;
        debug!("TMP117 is data ready: {is_data_ready}");
        Ok(is_data_ready)
    }

    pub fn get_temperature(&mut self) -> Result<f64, std::io::Error> {
        // -- read the 16 bit (word) config register
        let reg_val = i2cio::read_word(&mut self.i2c, TMP117_REG_TEMPERATURE)?;
        // -- TMP117 sends most significant byte first so a swap is required
        let reg_val = reg_val.swap_bytes();
        debug!("TMP117 temperature register: {reg_val:#018b}");
        let temp_celcius = ((reg_val as i16) as f64) * TMP117_TEMPERATURE_FACTOR;
        Ok(temp_celcius)
    }

    pub fn get_temperature_offset(&mut self) -> Result<f64, std::io::Error> {
        // -- read the 16 bit (word) config register
        let reg_val = i2cio::read_word(&mut self.i2c, TMP117_REG_TEMPERATURE_OFFSET)?;
        // -- TMP117 sends most significant byte first so a swap is required
        let reg_val = reg_val.swap_bytes();
        debug!("TMP117 temperature offset register: {reg_val:#06x}");
        let temp_offset = if reg_val != 0 {
            ((reg_val as i16) as f64) * TMP117_TEMPERATURE_FACTOR
        } else {
            0.0
        };
        Ok(temp_offset)
    }

    pub fn set_temperature_offset(&mut self, offset: f64) -> Result<(), std::io::Error> {
        // -- convert float to register value
        let reg_val = (offset / TMP117_TEMPERATURE_FACTOR) as i16;
        debug!("TMP117 writing temperature offset: {reg_val:#06x}");
        // -- TMP117 expects most significant byte first so a swap is required
        let reg_val = (reg_val.swap_bytes()) as u16;
        // -- read the 16 bit (word) config register
        i2cio::write_word(&mut self.i2c, TMP117_REG_TEMPERATURE_OFFSET, reg_val)
    }

}