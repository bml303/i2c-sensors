use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub enum BMP388DeviceAddress {
    Default,
    Secondary,
}

impl Default for BMP388DeviceAddress {
    fn default() -> Self {
        BMP388DeviceAddress::Default
    }
}

impl BMP388DeviceAddress {
    const ADDR_DEFAULT: u16 = 0x77;
    const ADDR_SECONDARY: u16 = 0x76;

    pub fn value(&self) -> u16 {
        match *self {
            Self::Default => Self::ADDR_DEFAULT,
            Self::Secondary => Self::ADDR_SECONDARY,
        }
    }
}

#[derive(PartialEq)]
pub enum BMP388SensorPowerMode {
    Sleep,
    Forced,
    Normal
}

impl BMP388SensorPowerMode {
    const POWERMODE_SLEEP: u8 = 0x00;
    const POWERMODE_FORCED: u8 = 0x01;
    const POWERMODE_NORMAL: u8 = 0x03;

    pub fn value(&self) -> u8 {
        match *self {
            Self::Sleep => Self::POWERMODE_SLEEP,
            Self::Forced => Self::POWERMODE_FORCED,
            Self::Normal => Self::POWERMODE_NORMAL,
        }
    }
}

impl fmt::Display for BMP388SensorPowerMode {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Sleep => write!(f, "Sleep/{:#04x}", self.value()),
            Self::Forced => write!(f, "Forced/{:#04x}", self.value()),
            Self::Normal => write!(f, "Normal/{:#04x}", self.value()),
        }
    }
}

#[derive(PartialEq)]
pub enum BMP388StatusPressureSensor {
    Disabled,
    Enabled,
}

impl BMP388StatusPressureSensor {
    pub fn value(&self) -> u8 {
        match *self {
            Self::Disabled => 0,
            Self::Enabled => 1,
        }
    }
}

impl fmt::Display for BMP388StatusPressureSensor {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Disabled => write!(f, "Disabled/{}", self.value()),
            Self::Enabled => write!(f, "Enabled/{}", self.value()),
        }
    }
}

#[derive(PartialEq)]
pub enum BMP388StatusTemperatureSensor {
    Disabled,
    Enabled,
}

impl BMP388StatusTemperatureSensor {
    pub fn value(&self) -> u8 {
        match *self {
            Self::Disabled => 0,
            Self::Enabled => 1,
        }
    }
}

impl fmt::Display for BMP388StatusTemperatureSensor {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Disabled => write!(f, "Disabled/{}", self.value()),
            Self::Enabled => write!(f, "Enabled/{}", self.value()),
        }
    }
}

#[allow(dead_code)]
pub enum BMP388OverSamplingPr {
    UltraLowX1, LowX2, StandardX4,
    HighX8, UltraHighX16, HighestX32,
}

impl BMP388OverSamplingPr {
    const OSR_X1_ULTRA_LOW: u8 = 0x00;
    const OSR_X2_LOW: u8 = 0x01;
    const OSR_X4_STANDARD: u8 = 0x02;
    const OSR_X8_HIGH: u8 = 0x03;
    const OSR_X16_ULTRA_HIGH: u8 = 0x04;
    const OSR_X32_HIGHEST: u8 = 0x05;

    pub fn value(&self) -> u8 {
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

impl fmt::Display for BMP388OverSamplingPr {
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

pub enum BMP388OverSamplingTp {
    X1, X2, X4, X8, X16, X32,
}

impl BMP388OverSamplingTp {
    const OSR_X1: u8 = 0x00;
    const OSR_X2: u8 = 0x01;
    const OSR_X4: u8 = 0x02;
    const OSR_X8: u8 = 0x03;
    const OSR_X16: u8 = 0x04;
    const OSR_X32: u8 = 0x05;

    pub fn value(&self) -> u8 {
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

impl fmt::Display for BMP388OverSamplingTp {
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

pub enum BMP388OutputDataRate {
    Odr200, Odr100, Odr50, Odr25, Odr12p5, 
    Odr6p25, Odr3p1, Odr1p5, Odr0p78, Odr0p39, 
    Odr0p2, Odr0p1, Odr0p05, Odr0p02, Odr0p01,
    Odr0p006, Odr0p003, Odr0p0015,
}

impl BMP388OutputDataRate {
    pub fn value(&self) -> u8 {
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

pub enum BMP388IrrFilter {
    Off, Coef1, Coef3, Coef7, Coef15, Coef31, Coef63, Coef127,
}

impl BMP388IrrFilter {
    const COEF_0: u8 = 0x00;
    const COEF_1: u8 = 0x01;
    const COEF_3: u8 = 0x02;
    const COEF_7: u8 = 0x03;
    const COEF_15: u8 = 0x04;
    const COEF_31: u8 = 0x05;
    const COEF_63: u8 = 0x06;
    const COEF_127: u8 = 0x07;

    pub fn value(&self) -> u8 {
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
pub enum BMP388StatusCommandDecoder {
    NotReady,
    Ready
}

impl fmt::Display for BMP388StatusCommandDecoder {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::NotReady => write!(f, "Command decoder not ready"),
            Self::Ready => write!(f, "Command decoder ready"),
        }
    }
}

#[derive(PartialEq)]
pub enum BMP388StatusPressureData {
    NotReady,
    Ready
}

impl fmt::Display for BMP388StatusPressureData {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::NotReady => write!(f, "Pressure data not ready"),
            Self::Ready => write!(f, "Pressure data ready"),
        }
    }
}

#[derive(PartialEq)]
pub enum BMP388StatusTemperatureData {
    NotReady,
    Ready
}

impl fmt::Display for BMP388StatusTemperatureData {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::NotReady => write!(f, "Temperature data not ready"),
            Self::Ready => write!(f, "Temperature data ready"),
        }
    }
}
