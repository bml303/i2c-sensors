use i2c_linux::I2c;
#[allow(unused_imports)]
use log::{debug, error, log_enabled, info, Level};
use std::fs::File;
use std::path::Path;

use crate::i2cio;

// -- setup consts
const ALPHA_CMD_SYSTEM_SETUP: u8 = 0b00100000;
const ALPHA_SYSTEM_SETUP_ENABLE_CLOCK: u8 = 1;
const ALPHA_CMD_DISPLAY_SETUP: u8 = 0b10000000;
const ALPHA_CMD_DIMMING_SETUP: u8 = 0b11100000;

// -- segment consts
const SEGMENT_BIT_00_MASK: u16 = 0b00000000000001;
const SEGMENT_BIT_00_BYTE: usize = 15;
const SEGMENT_BIT_00_SHLF: usize = 0;
const SEGMENT_BIT_01_MASK: u16 = 0b00000000000010;
const SEGMENT_BIT_01_BYTE: usize = 1;
const SEGMENT_BIT_01_SHLF: usize = 0;
const SEGMENT_BIT_02_MASK: u16 = 0b00000000000100;
const SEGMENT_BIT_02_BYTE: usize = 3;
const SEGMENT_BIT_02_SHLF: usize = 0;
const SEGMENT_BIT_03_MASK: u16 = 0b00000000001000;
const SEGMENT_BIT_03_BYTE: usize = 5;
const SEGMENT_BIT_03_SHLF: usize = 0;
const SEGMENT_BIT_04_MASK: u16 = 0b00000000010000;
const SEGMENT_BIT_04_BYTE: usize = 7;
const SEGMENT_BIT_04_SHLF: usize = 0;
const SEGMENT_BIT_05_MASK: u16 = 0b00000000100000;
const SEGMENT_BIT_05_BYTE: usize = 9;
const SEGMENT_BIT_05_SHLF: usize = 0;
const SEGMENT_BIT_06_MASK: u16 = 0b00000001000000;
const SEGMENT_BIT_06_BYTE: usize = 11;
const SEGMENT_BIT_06_SHLF: usize = 0;
const SEGMENT_BIT_07_MASK: u16 = 0b00000010000000;
const SEGMENT_BIT_07_BYTE: usize = 1;
const SEGMENT_BIT_07_SHLF: usize = 4;
const SEGMENT_BIT_08_MASK: u16 = 0b00000100000000;
const SEGMENT_BIT_08_BYTE: usize = 15;
const SEGMENT_BIT_08_SHLF: usize = 4;
const SEGMENT_BIT_09_MASK: u16 = 0b00001000000000;
const SEGMENT_BIT_09_BYTE: usize = 3;
const SEGMENT_BIT_09_SHLF: usize = 4;
const SEGMENT_BIT_10_MASK: u16 = 0b00010000000000;
const SEGMENT_BIT_10_BYTE: usize = 5;
const SEGMENT_BIT_10_SHLF: usize = 4;
const SEGMENT_BIT_11_MASK: u16 = 0b00100000000000;
const SEGMENT_BIT_11_BYTE: usize = 7;
const SEGMENT_BIT_11_SHLF: usize = 4;
const SEGMENT_BIT_12_MASK: u16 = 0b01000000000000;
const SEGMENT_BIT_12_BYTE: usize = 9;
const SEGMENT_BIT_12_SHLF: usize = 4;
const SEGMENT_BIT_13_MASK: u16 = 0b10000000000000;
const SEGMENT_BIT_13_BYTE: usize = 11;
const SEGMENT_BIT_13_SHLF: usize = 4;

#[allow(dead_code)]
#[derive(Clone, Debug, PartialEq)]
pub enum HT16K33DeviceAddress {
    Default,
    Alt1,
    Alt2,
    Alt3,
}

impl Default for HT16K33DeviceAddress {
    fn default() -> Self {
        HT16K33DeviceAddress::Default
    }
}

impl HT16K33DeviceAddress {
    const DEVICE_ADDR_DEFAULT: u16 = 0x70;
    const DEVICE_ADDR_ALT1: u16 = 0x71;
    const DEVICE_ADDR_ALT2: u16 = 0x72;
    const DEVICE_ADDR_ALT3: u16 = 0x73;

    fn value(&self) -> u16 {
        match *self {
            Self::Default => Self::DEVICE_ADDR_DEFAULT,
            Self::Alt1 => Self::DEVICE_ADDR_ALT1,
            Self::Alt2 => Self::DEVICE_ADDR_ALT2,
            Self::Alt3 => Self::DEVICE_ADDR_ALT3,
        }
    }
}

pub enum HT16K33DimmingDuty {
    Duty0, Duty1, Duty2, Duty3, 
    Duty4, Duty5, Duty6, Duty7,
    Duty8, Duty9, Duty10, Duty11,
    Duty12, Duty13, Duty14, Duty15,
}

impl HT16K33DimmingDuty {
    fn value(&self) -> u8 {
        match *self {
            Self::Duty0 => 0,
            Self::Duty1 => 1,
            Self::Duty2 => 2,
            Self::Duty3 => 3,
            Self::Duty4 => 4,
            Self::Duty5 => 5,
            Self::Duty6 => 6,
            Self::Duty7 => 7,
            Self::Duty8 => 8,
            Self::Duty9 => 9,
            Self::Duty10 => 10,
            Self::Duty11 => 11,
            Self::Duty12 => 12,
            Self::Duty13 => 13,
            Self::Duty14 => 14,
            Self::Duty15 => 15,
        }
    }
}

pub enum HT16K33BlinkRate {
    NoBlink,    
    BlinkRate2Hz,
    BlinkRate1Hz,
    BlinkRate0_5Hz,
}

impl HT16K33BlinkRate {
    const ALPHA_BLINK_RATE_NOBLINK: u8 = 0b00;
    const ALPHA_BLINK_RATE_2HZ: u8 = 0b01;
    const ALPHA_BLINK_RATE_1HZ: u8 = 0b10;
    const ALPHA_BLINK_RATE_0_5HZ: u8 = 0b11;    

    fn value(&self) -> u8 {
        match *self {
            Self::NoBlink => Self::ALPHA_BLINK_RATE_NOBLINK,
            Self::BlinkRate2Hz => Self::ALPHA_BLINK_RATE_2HZ,
            Self::BlinkRate1Hz => Self::ALPHA_BLINK_RATE_1HZ,
            Self::BlinkRate0_5Hz => Self::ALPHA_BLINK_RATE_0_5HZ,
        }
    }
}

enum HT16K33DisplayPower {
    DisplayOff,
    DisplayOn,
}

impl HT16K33DisplayPower {
    const ALPHA_DISPLAY_SETUP_DISPLAY_OFF: u8 = 0;
    const ALPHA_DISPLAY_SETUP_DISPLAY_ON: u8 = 1;

    fn value(&self) -> u8 {
        match *self {
            Self::DisplayOff => Self::ALPHA_DISPLAY_SETUP_DISPLAY_OFF,
            Self::DisplayOn => Self::ALPHA_DISPLAY_SETUP_DISPLAY_ON,            
        }
    }
}

pub struct HT16K33 {
    // -- i2c bus
    i2c: I2c<File>,
    // -- device address.
    device_addr: HT16K33DeviceAddress,
    // -- blink rate
    blink_rate: HT16K33BlinkRate,
    // -- display RAM
    display_ram: [u8; 16],
}

impl HT16K33 {

    pub fn new(i2c_bus_path: &Path, device_addr: HT16K33DeviceAddress, 
        dimming: HT16K33DimmingDuty, blink_rate: HT16K33BlinkRate) -> Result<HT16K33, std::io::Error> {
        // -- get the bus
        let mut i2c = i2cio::get_bus(i2c_bus_path)?;
        // -- set device address
        i2cio::set_slave(&mut i2c, device_addr.value())?;
        // -- check if device is available by reading part id
        Self::enable_system_clock(&mut i2c)?;
        Self::set_brightness_internal(&mut i2c, dimming)?;
        Self::set_blinkrate_internal(&mut i2c, &blink_rate, HT16K33DisplayPower::DisplayOn)?;
        // -- ready to display steady
        Ok(HT16K33 {
            i2c,
            device_addr,
            blink_rate,
            display_ram: [0; 16],
        })
    }

    #[allow(dead_code)]
    pub fn get_device_addr(&self) -> HT16K33DeviceAddress {
        self.device_addr.clone()
    }
    
    fn enable_system_clock(i2c: &mut I2c<File>) -> Result<(), std::io::Error> {
        let command: u8 = ALPHA_CMD_SYSTEM_SETUP | ALPHA_SYSTEM_SETUP_ENABLE_CLOCK;
        i2cio::write_byte_single(i2c, command)
    }

    pub fn set_brightness(&mut self, dimming: HT16K33DimmingDuty) -> Result<(), std::io::Error> {
        Self::set_brightness_internal(&mut self.i2c, dimming)
    }

    fn set_brightness_internal(i2c: &mut I2c<File>, duty: HT16K33DimmingDuty) -> Result<(), std::io::Error> {
        let command: u8 = ALPHA_CMD_DIMMING_SETUP | duty.value();
        i2cio::write_byte_single(i2c, command)
    }

    pub fn set_blinkrate(&mut self, blink_rate: HT16K33BlinkRate) -> Result<(), std::io::Error> {
        let res = Self::set_blinkrate_internal(&mut self.i2c, &blink_rate, HT16K33DisplayPower::DisplayOn);
        if res.is_ok() {
            self.blink_rate = blink_rate;
        }
        res
    }

    fn set_blinkrate_internal(i2c: &mut I2c<File>, blink_rate: &HT16K33BlinkRate, display_pwr: HT16K33DisplayPower) -> Result<(), std::io::Error> {
        let command: u8 = ALPHA_CMD_DISPLAY_SETUP | (blink_rate.value() << 1) | display_pwr.value();
        i2cio::write_byte_single(i2c, command)
    }

    pub fn set_disply_off(&mut self) -> Result<(), std::io::Error> {
        Self::set_blinkrate_internal(&mut self.i2c, &self.blink_rate, HT16K33DisplayPower::DisplayOff)
    }

    pub fn set_disply_on(&mut self) -> Result<(), std::io::Error> {
        Self::set_blinkrate_internal(&mut self.i2c, &self.blink_rate, HT16K33DisplayPower::DisplayOn)
    }

    // pub fn update_diaplay(&mut self) -> Result<(), std::io::Error> {
    //     self.i2c.smbus_write_block_data(0, &self.display_ram)
    // }

    pub fn print(&mut self, msg: String, ) -> Result<(), std::io::Error> {
        let segments = Self::get_segments(msg);
        self.display_ram = [0; 16];
        self.illuminate_char(segments[0], 0);
        self.illuminate_char(segments[1], 1);
        self.illuminate_char(segments[2], 2);
        self.illuminate_char(segments[3], 3);        
        self.i2c.smbus_write_block_data(0, &self.display_ram)
    }    

    fn illuminate_char(&mut self, segs_turn_on: u16, digit: u8) {
        // -- digit cannot be bigger than 4
        let digit = digit % 4;
        // -- segment 0
        if (segs_turn_on & SEGMENT_BIT_00_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_00_BYTE] |= (1 << digit) << SEGMENT_BIT_00_SHLF;
        }
        // -- segment 1
        if (segs_turn_on & SEGMENT_BIT_01_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_01_BYTE] |= (1 << digit) << SEGMENT_BIT_01_SHLF;
        }
        // -- segment 2
        if (segs_turn_on & SEGMENT_BIT_02_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_02_BYTE] |= (1 << digit) << SEGMENT_BIT_02_SHLF;
        }
        // -- segment 3
        if (segs_turn_on & SEGMENT_BIT_03_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_03_BYTE] |= (1 << digit) << SEGMENT_BIT_03_SHLF;
        }
        // -- segment 4
        if (segs_turn_on & SEGMENT_BIT_04_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_04_BYTE] |= (1 << digit) << SEGMENT_BIT_04_SHLF;
        }
        // -- segment 5
        if (segs_turn_on & SEGMENT_BIT_05_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_05_BYTE] |= (1 << digit) << SEGMENT_BIT_05_SHLF;
        }
        // -- segment 6
        if (segs_turn_on & SEGMENT_BIT_06_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_06_BYTE] |= (1 << digit) << SEGMENT_BIT_06_SHLF;
        }
        // -- segment 7
        if (segs_turn_on & SEGMENT_BIT_07_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_07_BYTE] |= (1 << digit) << SEGMENT_BIT_07_SHLF;
        }
        // -- segment 8
        if (segs_turn_on & SEGMENT_BIT_08_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_08_BYTE] |= (1 << digit) << SEGMENT_BIT_08_SHLF;
        }
        // -- segment 9
        if (segs_turn_on & SEGMENT_BIT_09_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_09_BYTE] |= (1 << digit) << SEGMENT_BIT_09_SHLF;
        }
        // -- segment 10
        if (segs_turn_on & SEGMENT_BIT_10_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_10_BYTE] |= (1 << digit) << SEGMENT_BIT_10_SHLF;
        }
        // -- segment 11
        if (segs_turn_on & SEGMENT_BIT_11_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_11_BYTE] |= (1 << digit) << SEGMENT_BIT_11_SHLF;
        }
        // -- segment 12
        if (segs_turn_on & SEGMENT_BIT_12_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_12_BYTE] |= (1 << digit) << SEGMENT_BIT_12_SHLF;
        }
        // -- segment 13
        if (segs_turn_on & SEGMENT_BIT_13_MASK) > 0 {
            self.display_ram[SEGMENT_BIT_13_BYTE] |= (1 << digit) << SEGMENT_BIT_13_SHLF;
        }
    }

    fn get_segments(msg: String) -> [u16;4] {
        let mut segments: [u16;4] = [0;4];
        let mut chars = msg.chars();
        if let Some(digit0) = chars.next() {
            segments[0] = Self::get_segments_for_char(digit0);
        };
        if let Some(digit1) = chars.next() {
            segments[1] = Self::get_segments_for_char(digit1);
        };
        if let Some(digit2) = chars.next() {
            segments[2] = Self::get_segments_for_char(digit2);
        };
        if let Some(digit3) = chars.next() {
            segments[3] = Self::get_segments_for_char(digit3);
        };        
        segments
    }    

    fn get_segments_for_char(char: char) -> u16 {
        return match char {            
            '!' => 0b00001000001000,
            '"' => 0b00001000000010,
            '#' => 0b01001101001110,
            '$' => 0b01001101101101,
            '%' => 0b10010000100100,
            '&' => 0b00110011011001,
            '\'' => 0b00001000000000,
            '(' => 0b00000000111001,
            ')' => 0b00000000001111,
            '*' => 0b11111010000000,
            '+' => 0b01001101000000,
            ',' => 0b10000000000000,
            '-' => 0b00000101000000,
            '.' => 0b00000000000000,
            '/' => 0b10010000000000,
            '0' => 0b00000000111111,
            '1' => 0b00010000000110,
            '2' => 0b00000101011011,
            '3' => 0b00000101001111,
            '4' => 0b00000101100110,
            '5' => 0b00000101101101,
            '6' => 0b00000101111101,
            '7' => 0b01010000000001,
            '8' => 0b00000101111111,
            '9' => 0b00000101100111,
            ':' => 0b00000000000000,
            ';' => 0b10001000000000,
            '<' => 0b00110000000000,
            '=' => 0b00000101001000,
            '>' => 0b01000010000000,            
            '?' => 0b01000100000011,
            '@' => 0b00001100111011,
            'A' => 0b00000101110111,
            'B' => 0b01001100001111,
            'C' => 0b00000000111001,
            'D' => 0b01001000001111,
            'E' => 0b00000101111001,
            'F' => 0b00000101110001,
            'G' => 0b00000100111101,
            'H' => 0b00000101110110,
            'I' => 0b01001000001001,
            'J' => 0b00000000011110,
            'K' => 0b00110001110000,
            'L' => 0b00000000111000,
            'M' => 0b00010010110110,
            'N' => 0b00100010110110,
            'O' => 0b00000000111111,
            'P' => 0b00000101110011,
            'Q' => 0b00100000111111,
            'R' => 0b00100101110011,
            'S' => 0b00000110001101,
            'T' => 0b01001000000001,
            'U' => 0b00000000111110,
            'V' => 0b10010000110000,
            'W' => 0b10100000110110,
            'X' => 0b10110010000000,
            'Y' => 0b01010010000000,
            'Z' => 0b10010000001001,
            '[' => 0b00000000111001,
            '\\' => 0b00100010000000,
            ']' => 0b00000000001111,
            '^' => 0b10100000000000,
            '_' => 0b00000000001000,
            '`' => 0b00000010000000,
            'a' => 0b00000101011111,
            'b' => 0b00100001111000,
            'c' => 0b00000101011000,
            'd' => 0b10000100001110,
            'e' => 0b00000001111001, 
            'f' => 0b00000001110001,
            'g' => 0b00000110001111,
            'h' => 0b00000101110100,
            'i' => 0b01000000000000,
            'j' => 0b00000000001110,
            'k' => 0b01111000000000,
            'l' => 0b01001000000000,
            'm' => 0b01000101010100,
            'n' => 0b00100001010000,
            'o' => 0b00000101011100,
            'p' => 0b00010001110001,
            'q' => 0b00100101100011,
            'r' => 0b00000001010000,
            's' => 0b00000110001101,
            't' => 0b00000001111000,
            'u' => 0b00000000011100,
            'v' => 0b10000000010000,
            'w' => 0b10100000010100,
            'x' => 0b10110010000000,
            'y' => 0b00001100001110,
            'z' => 0b10010000001001,
            '{' => 0b10000011001001,
            '|' => 0b01001000000000,
            '}' => 0b00110100001001,
            '~' => 0b00000101010010,
            _ => 0b00000000000000, 
        };
    }
}