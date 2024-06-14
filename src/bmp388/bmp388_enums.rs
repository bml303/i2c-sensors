use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub enum Bmp388DeviceAddress {
    Default,
    Secondary,
}

impl Default for Bmp388DeviceAddress {
    fn default() -> Self {
        Self::Default
    }
}

impl Bmp388DeviceAddress {
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
pub enum Bmp388SensorPowerMode {
    Sleep,
    Forced,
    Normal
}

impl Bmp388SensorPowerMode {
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

impl fmt::Display for Bmp388SensorPowerMode {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Sleep => write!(f, "Sleep/{:#04x}", self.value()),
            Self::Forced => write!(f, "Forced/{:#04x}", self.value()),
            Self::Normal => write!(f, "Normal/{:#04x}", self.value()),
        }
    }
}

#[derive(PartialEq)]
pub enum Bmp388StatusPressureSensor {
    Disabled,
    Enabled,
}

impl Bmp388StatusPressureSensor {
    pub fn value(&self) -> u8 {
        match *self {
            Self::Disabled => 0,
            Self::Enabled => 1,
        }
    }
}

impl fmt::Display for Bmp388StatusPressureSensor {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Disabled => write!(f, "Disabled/{}", self.value()),
            Self::Enabled => write!(f, "Enabled/{}", self.value()),
        }
    }
}

#[derive(PartialEq)]
pub enum Bmp388StatusTemperatureSensor {
    Disabled,
    Enabled,
}

impl Bmp388StatusTemperatureSensor {
    pub fn value(&self) -> u8 {
        match *self {
            Self::Disabled => 0,
            Self::Enabled => 1,
        }
    }
}

impl fmt::Display for Bmp388StatusTemperatureSensor {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Disabled => write!(f, "Disabled/{}", self.value()),
            Self::Enabled => write!(f, "Enabled/{}", self.value()),
        }
    }
}

#[allow(dead_code)]
pub enum Bmp388OverSamplingPr {
    UltraLowX1, LowX2, StandardX4,
    HighX8, UltraHighX16, HighestX32,
}

impl Bmp388OverSamplingPr {
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

impl fmt::Display for Bmp388OverSamplingPr {
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

pub enum Bmp388OverSamplingTp {
    X1, X2, X4, X8, X16, X32,
}

impl Bmp388OverSamplingTp {
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

impl fmt::Display for Bmp388OverSamplingTp {
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

pub enum Bmp388OutputDataRate {
    Ax200Hz, Bx100Hz, Cx50Hz, Dx25Hz, Ex12_5Hz, 
    Fx6_25Hz, Gx3_1Hz, Hx1_5Hz, Ix0_78Hz, Jx0_39Hz, 
    Kx0_2Hz, Lx0_1Hz, Mx0_05Hz, Nx0_02Hz, Ox0_01Hz,
    Px0_006Hz, Qx0_003Hz, Rx0_0015Hz,
}

impl Bmp388OutputDataRate {
    pub fn value(&self) -> u8 {
        match *self {
            Self::Ax200Hz => 0x00,
            Self::Bx100Hz => 0x01,
            Self::Cx50Hz => 0x02,
            Self::Dx25Hz => 0x03,
            Self::Ex12_5Hz => 0x04,
            Self::Fx6_25Hz => 0x05,
            Self::Gx3_1Hz => 0x06,
            Self::Hx1_5Hz => 0x07,
            Self::Ix0_78Hz => 0x08,
            Self::Jx0_39Hz => 0x09,
            Self::Kx0_2Hz => 0x0a,
            Self::Lx0_1Hz => 0x0b,
            Self::Mx0_05Hz => 0x0c,
            Self::Nx0_02Hz => 0x0d,
            Self::Ox0_01Hz => 0x0e,
            Self::Px0_006Hz => 0x0f,
            Self::Qx0_003Hz => 0x10,
            Self::Rx0_0015Hz => 0x11,
        }
    }
}

pub enum Bmp388IrrFilter {
    Off, Coef1, Coef3, Coef7, Coef15, Coef31, Coef63, Coef127,
}

impl Bmp388IrrFilter {
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
pub enum Bmp388StatusCommandDecoder {
    NotReady,
    Ready,
}

impl fmt::Display for Bmp388StatusCommandDecoder {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::NotReady => write!(f, "Command decoder not ready"),
            Self::Ready => write!(f, "Command decoder ready"),
        }
    }
}

#[derive(PartialEq)]
pub enum Bmp388StatusPressureData {
    NotReady,
    Ready,
}

impl fmt::Display for Bmp388StatusPressureData {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::NotReady => write!(f, "Pressure data not ready"),
            Self::Ready => write!(f, "Pressure data ready"),
        }
    }
}

#[derive(PartialEq)]
pub enum Bmp388StatusTemperatureData {
    NotReady,
    Ready,
}

impl fmt::Display for Bmp388StatusTemperatureData {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::NotReady => write!(f, "Temperature data not ready"),
            Self::Ready => write!(f, "Temperature data ready"),
        }
    }
}

#[derive(PartialEq)]
pub enum Bmp388FifoStopOnFull {
    Disabled,
    Enabled,
}

impl Bmp388FifoStopOnFull {
    pub fn value(&self) -> u8 {
        match *self {
            Self::Disabled => 0,
            Self::Enabled => 1,
        }
    }
}

#[derive(PartialEq)]
pub enum Bmp388FifoWithPressureData {
    Disabled,
    Enabled,
}

impl Bmp388FifoWithPressureData {
    pub fn value(&self) -> u8 {
        match *self {
            Self::Disabled => 0,
            Self::Enabled => 1,
        }
    }
}

#[derive(PartialEq)]
pub enum Bmp388FifoWithTemperatureData {
    Disabled,
    Enabled,
}

impl Bmp388FifoWithTemperatureData {
    pub fn value(&self) -> u8 {
        match *self {
            Self::Disabled => 0,
            Self::Enabled => 1,
        }
    }
}

#[derive(PartialEq)]
pub enum Bmp388FifoWithSensorTime {
    Disabled,
    Enabled,
}

impl Bmp388FifoWithSensorTime {
    pub fn value(&self) -> u8 {
        match *self {
            Self::Disabled => 0,
            Self::Enabled => 1,
        }
    }
}

#[derive(PartialEq)]
pub enum Bmp388FifoDataFiltered {
    Unfiltered,
    Filtered,
}

impl Bmp388FifoDataFiltered {
    pub fn value(&self) -> u8 {
        match *self {
            Self::Unfiltered => 0,
            Self::Filtered => 1,
        }
    }
}
