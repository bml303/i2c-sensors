use i2c_linux::{
    I2c, Message, ReadFlags,
};
#[allow(unused_imports)]
use log::{debug, error, log_enabled, info, Level};
use std::fs::File;
use std::path::Path;
use std::{thread, time};

use crate::i2cio;

const SHT31_COMMAND_FETCH_DATA: u16 = 0xe000;
const SHT31_COMMAND_READ_STATUS: u16 = 0xf32d;
const SHT31_COMMAND_RESET_STATUS: u16 = 0x3041;
const SHT31_COMMAND_SOFT_RESET: u16 = 0x30a2;
const SHT31_COMMAND_STOP_CONTINUOUS_MODE: u16 = 0x3093;

// -- the soft reset time is actually 1.5ms
const SHT31_SOFT_RESET_DELAY_MS: u64 = 2;
const SHT31_NO_CLOCK_STRETCH_READ_DELAY_MS: u64 = 5;

#[allow(dead_code)]
#[derive(Clone, Debug, PartialEq)]
pub enum SHT31DeviceAddress {
    Default,
    Secondary,
}

impl Default for SHT31DeviceAddress {
    fn default() -> Self {
        SHT31DeviceAddress::Default
    }
}

impl SHT31DeviceAddress {
    const DEVICE_ADDR_DEFAULT: u16 = 0x44;
    const DEVICE_ADDR_SECONDARY: u16 = 0x45;

    fn value(&self) -> u16 {
        match *self {
            Self::Default => Self::DEVICE_ADDR_DEFAULT,
            Self::Secondary => Self::DEVICE_ADDR_SECONDARY,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum SHT31SingleShotAcquisition {
    RepeatabilityHigh,
    RepeatabilityMedium,
    RepeatabilityLow,
}

impl SHT31SingleShotAcquisition {
    const REPEATABILITY_HIGH_WITH_CLOCK_STRETCHING: u16 = 0x2c06;
    const REPEATABILITY_MEDIUM_WITH_CLOCK_STRETCHING: u16 = 0x2c0d;
    const REPEATABILITY_LOW_WITH_CLOCK_STRETCHING: u16 = 0x2c10;

    fn value(&self) -> u16 {
        match *self {
            Self::RepeatabilityHigh => Self::REPEATABILITY_HIGH_WITH_CLOCK_STRETCHING,
            Self::RepeatabilityMedium => Self::REPEATABILITY_MEDIUM_WITH_CLOCK_STRETCHING,
            Self::RepeatabilityLow => Self::REPEATABILITY_LOW_WITH_CLOCK_STRETCHING,
        }
    }
}


#[derive(Clone, Debug, PartialEq)]
pub enum SHT31SingleShotAcquisitionNoClockStretch {
    RepeatabilityHigh,
    RepeatabilityMedium,
    RepeatabilityLow,
}

impl SHT31SingleShotAcquisitionNoClockStretch {
    const REPEATABILITY_HIGH_NO_CLOCK_STRETCHING: u16 = 0x2400;
    const REPEATABILITY_MEDIUM_NO_CLOCK_STRETCHING: u16 = 0x240b;
    const REPEATABILITY_LOW_NO_CLOCK_STRETCHING: u16 = 0x2416;

    fn value(&self) -> u16 {
        match *self {
            Self::RepeatabilityHigh => Self::REPEATABILITY_HIGH_NO_CLOCK_STRETCHING,
            Self::RepeatabilityMedium => Self::REPEATABILITY_MEDIUM_NO_CLOCK_STRETCHING,
            Self::RepeatabilityLow => Self::REPEATABILITY_LOW_NO_CLOCK_STRETCHING,
        }
    }
}


pub enum SHT31ContinuousAcquisition {
    RepeatabilityHigh0_5Mps,
    RepeatabilityMedium0_5Mps,
    RepeatabilityLow0_5Mps,
    RepeatabilityHigh1Mps,
    RepeatabilityMedium1Mps,
    RepeatabilityLow1Mps,
    RepeatabilityHigh2Mps,
    RepeatabilityMedium2Mps,
    RepeatabilityLow2Mps,
    RepeatabilityHigh4Mps,
    RepeatabilityMedium4Mps,
    RepeatabilityLow4Mps,
    RepeatabilityHigh10Mps,
    RepeatabilityMedium10Mps,
    RepeatabilityLow10Mps,
}

impl SHT31ContinuousAcquisition {
    const REPEATABILITY_HIGH_0_5_MPS: u16 = 0x2032;
    const REPEATABILITY_MEDIUM_0_5_MPS: u16 = 0x2024;
    const REPEATABILITY_LOW_0_5_MPS: u16 = 0x202f;
    const REPEATABILITY_HIGH_1_MPS: u16 = 0x2130;
    const REPEATABILITY_MEDIUM_1_MPS: u16 = 0x2126;
    const REPEATABILITY_LOW_1_MPS: u16 = 0x212d;
    const REPEATABILITY_HIGH_2_MPS: u16 = 0x2236;
    const REPEATABILITY_MEDIUM_2_MPS: u16 = 0x2220;
    const REPEATABILITY_LOW_2_MPS: u16 = 0x222b;
    const REPEATABILITY_HIGH_4_MPS: u16 = 0x2234;
    const REPEATABILITY_MEDIUM_4_MPS: u16 = 0x2222;
    const REPEATABILITY_LOW_4_MPS: u16 = 0x2229;
    const REPEATABILITY_HIGH_10_MPS: u16 = 0x2737;
    const REPEATABILITY_MEDIUM_10_MPS: u16 = 0x2721;
    const REPEATABILITY_LOW_10_MPS: u16 = 0x272a;

    fn value(&self) -> u16 {
        match *self {
            Self::RepeatabilityHigh0_5Mps => Self::REPEATABILITY_HIGH_0_5_MPS,
            Self::RepeatabilityMedium0_5Mps => Self::REPEATABILITY_MEDIUM_0_5_MPS,
            Self::RepeatabilityLow0_5Mps => Self::REPEATABILITY_LOW_0_5_MPS,
            Self::RepeatabilityHigh1Mps => Self::REPEATABILITY_HIGH_1_MPS,
            Self::RepeatabilityMedium1Mps => Self::REPEATABILITY_MEDIUM_1_MPS,
            Self::RepeatabilityLow1Mps => Self::REPEATABILITY_LOW_1_MPS,
            Self::RepeatabilityHigh2Mps => Self::REPEATABILITY_HIGH_2_MPS,
            Self::RepeatabilityMedium2Mps => Self::REPEATABILITY_MEDIUM_2_MPS,
            Self::RepeatabilityLow2Mps => Self::REPEATABILITY_LOW_2_MPS,
            Self::RepeatabilityHigh4Mps => Self::REPEATABILITY_HIGH_4_MPS,
            Self::RepeatabilityMedium4Mps => Self::REPEATABILITY_MEDIUM_4_MPS,
            Self::RepeatabilityLow4Mps => Self::REPEATABILITY_LOW_4_MPS,
            Self::RepeatabilityHigh10Mps => Self::REPEATABILITY_HIGH_10_MPS,
            Self::RepeatabilityMedium10Mps => Self::REPEATABILITY_MEDIUM_10_MPS,
            Self::RepeatabilityLow10Mps => Self::REPEATABILITY_LOW_10_MPS,
        }
    }
}

pub struct SHT31 {
    // -- i2c bus
    i2c: I2c<File>,
    // -- device address
    device_addr: SHT31DeviceAddress,
    // -- measuring mode
}

impl SHT31 {

    pub fn new(i2c_bus_path: &Path, device_addr: SHT31DeviceAddress) -> Result<SHT31, std::io::Error> {
        // -- get the bus
        let mut i2c = i2cio::get_bus(i2c_bus_path)?;
        // -- set device address
        i2cio::set_slave(&mut i2c, device_addr.value())?;
        // -- create SHT31 object
        let mut sht31 = SHT31 {
            i2c,
            device_addr,
        };
        // -- read status register
        debug!("Reading SHT31 status register");
        let status_reg_val = sht31.get_status()?;
        debug!("SHT31 status register value: {status_reg_val:#010b}");
        if status_reg_val != 0 {
            // -- reset status register
            debug!("Resetting status register SHT31");
            sht31.reset_status()?;
        }
        // -- stop continuous mode
        debug!("Stopping SHT31 continuous mode");
        sht31.stop_continuous_mode()?;
        // -- do a soft reset since it's in an unknown state
        // debug!("Soft-resetting SHT31");
        // sht31.soft_reset()?;
        // -- ready to measure steady
        Ok(sht31)
    }

    pub fn get_status(&mut self) -> Result<u16, std::io::Error> {
        // -- SHT31 expects most significant byte first
        let cmd_msb: u8 = (SHT31_COMMAND_READ_STATUS >> 8) as u8;
        let cmd_lsb: u8 = (SHT31_COMMAND_READ_STATUS & 0xff) as u8;
        // -- send MSB as command and LSB as data
        debug!("Sending SHT31 command: {cmd_msb:#04x} {cmd_lsb:#04x}");
        i2cio::write_byte(&mut self.i2c, cmd_msb, cmd_lsb)?;
        // -- read response
        let mut read_buf: [u8; 3] = [0; 3];
        let read_message = Message::Read { address: self.device_addr.value(), data: &mut read_buf, flags: ReadFlags::empty() };
        let mut messages = [read_message];
        self.i2c.i2c_transfer(&mut messages)?;
        let reg_msb = read_buf[0] as u16;
        let reg_lsb = read_buf[1] as u16;
        let reg_val = reg_msb << 8 | reg_lsb;
        Ok(reg_val)
    }

    pub fn reset_status(&mut self) -> Result<(), std::io::Error> {
        // -- SHT31 expects most significant byte first
        let cmd_msb: u8 = (SHT31_COMMAND_RESET_STATUS >> 8) as u8;
        let cmd_lsb: u8 = (SHT31_COMMAND_RESET_STATUS & 0xff) as u8;
        // -- send MSB as command and LSB as data
        debug!("Sending SHT31 command: {cmd_msb:#04x} {cmd_lsb:#04x}");
        i2cio::write_byte(&mut self.i2c, cmd_msb, cmd_lsb)
    }

    pub fn soft_reset(&mut self) -> Result<(), std::io::Error> {
        // -- SHT31 expects most significant byte first
        let cmd_msb: u8 = (SHT31_COMMAND_SOFT_RESET >> 8) as u8;
        let cmd_lsb: u8 = (SHT31_COMMAND_SOFT_RESET & 0xff) as u8;
        // -- send MSB as command and LSB as data
        debug!("Sending SHT31 command: {cmd_msb:#04x} {cmd_lsb:#04x}");
        i2cio::write_byte(&mut self.i2c, cmd_msb, cmd_lsb)?;
        // -- wait for the device to startup
        let startup_delay = time::Duration::from_millis(SHT31_SOFT_RESET_DELAY_MS);
        thread::sleep(startup_delay);
        Ok(())
    }

    pub fn get_data_single(&mut self, acquisition_mode: SHT31SingleShotAcquisition) 
         -> Result<(u16, u16), std::io::Error> { 
            let acquisition_mode = acquisition_mode.value();
        // -- SHT31 expects most significant byte first
        let cmd_msb: u8 = (acquisition_mode >> 8) as u8;
        let cmd_lsb: u8 = (acquisition_mode & 0xff) as u8;
        // -- send MSB as command and LSB as data
        debug!("Sending SHT31 command: {cmd_msb:#04x} {cmd_lsb:#04x}");
        i2cio::write_byte(&mut self.i2c, cmd_msb, cmd_lsb)?;
        // -- read response
        let mut read_buf: [u8; 6] = [0; 6];
        let read_message = Message::Read { address: self.device_addr.value(), data: &mut read_buf, flags: ReadFlags::empty() };
        let mut messages = [read_message];
        self.i2c.i2c_transfer(&mut messages)?;
        let temperature_msb = read_buf[0] as u16;
        let temperature_lsb = read_buf[1] as u16;
        let temperature_raw = temperature_msb << 8 | temperature_lsb;
        let humidity_msb = read_buf[3] as u16;
        let humidity_lsb = read_buf[4] as u16;
        let humidity_raw = humidity_msb << 8 | humidity_lsb;
        Ok((temperature_raw, humidity_raw))
    }

    pub fn get_data_single_no_clock_stretch(&mut self, acquisition_mode: SHT31SingleShotAcquisitionNoClockStretch) 
        -> Result<(u16, u16), std::io::Error> { 
        let acquisition_mode = acquisition_mode.value();
        // -- SHT31 expects most significant byte first
        let cmd_msb: u8 = (acquisition_mode >> 8) as u8;
        let cmd_lsb: u8 = (acquisition_mode & 0xff) as u8;
        // -- send MSB as command and LSB as data
        debug!("Sending SHT31 command: {cmd_msb:#04x} {cmd_lsb:#04x}");
        i2cio::write_byte(&mut self.i2c, cmd_msb, cmd_lsb)?;
        // -- no clock stretch requires a delay before reading values 
        let startup_delay = time::Duration::from_millis(SHT31_NO_CLOCK_STRETCH_READ_DELAY_MS);
        thread::sleep(startup_delay);
        // -- read response
        let mut read_buf: [u8; 6] = [0; 6];
        let read_message = Message::Read { address: self.device_addr.value(), data: &mut read_buf, flags: ReadFlags::empty() };
        let mut messages = [read_message];
        self.i2c.i2c_transfer(&mut messages)?;
        let temperature_msb = read_buf[0] as u16;
        let temperature_lsb = read_buf[1] as u16;
        let temperature_raw = temperature_msb << 8 | temperature_lsb;
        let humidity_msb = read_buf[3] as u16;
        let humidity_lsb = read_buf[4] as u16;
        let humidity_raw = humidity_msb << 8 | humidity_lsb;
        Ok((temperature_raw, humidity_raw))
    }

    pub fn start_continuous_mode(&mut self, acquisition_mode: SHT31ContinuousAcquisition) -> Result<(), std::io::Error> {
        let acquisition_mode = acquisition_mode.value();
        // -- SHT31 expects most significant byte first
        let cmd_msb: u8 = (acquisition_mode >> 8) as u8;
        let cmd_lsb: u8 = (acquisition_mode & 0xff) as u8;
        // -- send MSB as command and LSB as data
        debug!("Sending SHT31 command: {cmd_msb:#04x} {cmd_lsb:#04x}");
        i2cio::write_byte(&mut self.i2c, cmd_msb, cmd_lsb)
    }

    pub fn stop_continuous_mode(&mut self) -> Result<(), std::io::Error> {
        // -- SHT31 expects most significant byte first
        let cmd_msb: u8 = (SHT31_COMMAND_STOP_CONTINUOUS_MODE >> 8) as u8;
        let cmd_lsb: u8 = (SHT31_COMMAND_STOP_CONTINUOUS_MODE & 0xff) as u8;
        // -- send MSB as command and LSB as data
        debug!("Sending SHT31 command: {cmd_msb:#04x} {cmd_lsb:#04x}");
        i2cio::write_byte(&mut self.i2c, cmd_msb, cmd_lsb)
    }

    pub fn get_data_continuous(&mut self) 
         -> Result<(u16, u16), std::io::Error> {
        // -- SHT31 expects most significant byte first
        let cmd_msb: u8 = (SHT31_COMMAND_FETCH_DATA >> 8) as u8;
        let cmd_lsb: u8 = (SHT31_COMMAND_FETCH_DATA & 0xff) as u8;
        // -- send MSB as command and LSB as data
        debug!("Sending SHT31 command: {cmd_msb:#04x} {cmd_lsb:#04x}");
        i2cio::write_byte(&mut self.i2c, cmd_msb, cmd_lsb)?;
        // -- read response
        let mut read_buf: [u8; 6] = [0; 6];
        let read_message = Message::Read { address: self.device_addr.value(), data: &mut read_buf, flags: ReadFlags::empty() };
        let mut messages = [read_message];
        self.i2c.i2c_transfer(&mut messages)?;
        let temperature_msb = read_buf[0] as u16;
        let temperature_lsb = read_buf[1] as u16;
        let temperature_raw = temperature_msb << 8 | temperature_lsb;
        let humidity_msb = read_buf[3] as u16;
        let humidity_lsb = read_buf[4] as u16;
        let humidity_raw = humidity_msb << 8 | humidity_lsb;
        Ok((temperature_raw, humidity_raw))
    }

    pub fn get_temperature_celcius(&self, temperature_raw: u16) -> f64 {
        -45.0 + (temperature_raw as f64 * 175.0) / 65535.0
    }

    pub fn get_temperature_fahrenheit(&self, temperature_raw: u16) -> f64 {
        -49.0 + (temperature_raw as f64 * 315.0) / 65535.0
    }

    pub fn get_humidity(&self, humidity_raw: u16) -> f64 {
        (humidity_raw as f64 * 100.0) / 65535.0
    }

}