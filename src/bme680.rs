use i2c_linux::I2c;
#[allow(unused_imports)]
use log::{debug, info};
use std::fmt;
use std::fs::File;
use std::path::Path;
use std::{thread, time};

use crate::i2cio;


const BME680_CHIP_ID: u8 = 0x61;

// -- control, status and result registers
const BME680_REG_MEAS_STATUS_0: u8 = 0x1d;
const BME680_REG_MEAS_RESULT_BASE: u8 = 0x1f;
#[allow(dead_code)]
const BME680_REG_IDAC_HEAT_BASE: u8 = 0x50;
const BME680_REG_RES_HEAT_BASE: u8 = 0x5a;
const BME680_REG_GAS_WAIT_BASE: u8 = 0x64;
const BME680_REG_GAS_ACD_MSB: u8 = 0x2a;
const BME680_REG_GAS_ACD_LSB_RANGE: u8 = 0x2b;
const BME680_REG_CONFIG: u8 = 0x75;
const BME680_REG_CTRL_GAS_0: u8 = 0x70;
const BME680_REG_CTRL_GAS_1: u8 = 0x71;
const BME680_REG_CTRL_HUM: u8 = 0x72;
const BME680_REG_CTRL_MEAS: u8 = 0x74;
const BME680_REG_CHIP_ID: u8 = 0xd0;
const BME680_REG_RESET: u8 = 0xe0;

// -- registers for calibration data
const BME680_REG_CALIB_DATA1_BASE: u8 = 0x8a;
const BME680_REG_CALIB_DATA2_BASE: u8 = 0xe1;
const BME680_REG_CALIB_DATA3_BASE: u8 = 0x00;

const BME680_CALIB_DATA1_LEN: usize = 23;
const BME680_CALIB_DATA2_LEN: usize = 14;
const BME680_CALIB_DATA3_LEN: usize = 5;

// -- Coefficient T2 LSB position
const BME680_IDX_T2_LSB: usize = 0;
// -- Coefficient T2 MSB position
const BME680_IDX_T2_MSB: usize = 1;
// -- Coefficient T3 position
const BME680_IDX_T3: usize = 2;
// -- Coefficient P1 LSB position
const BME68X_IDX_P1_LSB: usize = 4;
// -- Coefficient P1 MSB position
const BME68X_IDX_P1_MSB: usize = 5;
// -- Coefficient P2 LSB position
const BME68X_IDX_P2_LSB: usize = 6;
// -- Coefficient P2 MSB position
const BME68X_IDX_P2_MSB: usize = 7;
// -- Coefficient P3 position
const BME68X_IDX_P3: usize = 8;
// -- Coefficient P4 LSB position
const BME68X_IDX_P4_LSB: usize = 10;
// -- Coefficient P4 MSB position
const BME68X_IDX_P4_MSB: usize = 11;
// -- Coefficient P5 LSB position
const BME68X_IDX_P5_LSB: usize = 12;
// -- Coefficient P5 MSB position
const BME68X_IDX_P5_MSB: usize = 13;
// -- Coefficient P7 position
const BME68X_IDX_P7: usize = 14;
// -- Coefficient P6 position
const BME68X_IDX_P6: usize = 15;
// -- Coefficient P8 LSB position
const BME68X_IDX_P8_LSB: usize = 18;
// -- Coefficient P8 MSB position
const BME68X_IDX_P8_MSB: usize = 19;
// -- Coefficient P9 LSB position
const BME68X_IDX_P9_LSB: usize = 20;
// -- Coefficient P9 MSB position
const BME68X_IDX_P9_MSB: usize = 21;
// -- Coefficient P10 position
const BME68X_IDX_P10: usize = 22;
// -- Coefficient H2 MSB position
const BME68X_IDX_H2_MSB: usize = 23;
// -- Coefficient H2 LSB position
const BME68X_IDX_H2_LSB: usize = 24;
// -- Coefficient H1 LSB position
const BME68X_IDX_H1_LSB: usize = 24;
// -- Coefficient H1 MSB position
const BME68X_IDX_H1_MSB: usize = 25;
// -- Coefficient H3 position
const BME68X_IDX_H3: usize = 26;
// -- Coefficient H4 position
const BME68X_IDX_H4: usize = 27;
// -- Coefficient H5 position
const BME68X_IDX_H5: usize = 28;
// -- Coefficient H6 position
const BME68X_IDX_H6: usize = 29;
// -- Coefficient H7 position
const BME68X_IDX_H7: usize = 30;
// -- Coefficient T1 LSB position
const BME680_IDX_T1_LSB: usize = 31;
// -- Coefficient T1 MSB position
const BME680_IDX_T1_MSB: usize = 32;
// -- Coefficient GH2 LSB position
const BME68X_IDX_GH2_LSB: usize = 33;
// -- Coefficient GH2 MSB position
const BME68X_IDX_GH2_MSB: usize = 34;
// -- Coefficient GH1 position
const BME68X_IDX_GH1: usize = 35;
// -- Coefficient GH3 position
const BME68X_IDX_GH3: usize = 36;
// -- Coefficient res heat value position
const BME68X_IDX_RES_HEAT_VAL: usize = 37;
// -- Coefficient res heat range position
const BME68X_IDX_RES_HEAT_RANGE: usize = 39;
// -- Coefficient range switching error position
const BME68X_IDX_RANGE_SW_ERR: usize = 41;

// -- length values for block reads
const BME680_MEAS_RESULT_LEN: usize = 8;
#[allow(dead_code)]
const BME680_IDAC_HEAT_BASE_LEN: usize = 10;
const BME680_RES_HEAT_BASE_LEN: usize = 10;
const BME680_GAS_WAIT_BASE_LEN: usize = 10;

// -- mask and bits for meas_status_0 register
const BME680_MEAS_STATUS_0_NEW_DATA_BIT: u8 = 0x80;
const BME680_MEAS_STATUS_0_GAS_MEASURING_BIT: u8 = 0x40;
const BME680_MEAS_STATUS_0_MEASURING_BIT: u8 = 0x20;
const BME680_MEAS_STATUS_0_GAS_MEAS_INDEX_MASK: u8 = 0x0f;

// -- other values
const BME680_COMMAND_SOFT_RESET: u8 = 0xb6;
const BME680_STARTUP_DELAY_MS: u64 = 2;

// -- shift, bit, and mask values
const BME680_12_BIT_SHIFT: u8 = 12;
const BME680_8_BIT_SHIFT: u8 = 8;
const BME680_6_BIT_SHIFT: u8 = 6;
const BME680_4_BIT_SHIFT: u8 = 4;
const BME680_2_BIT_SHIFT: u8 = 2;
const BME680_4_BIT_MASK: u8 = 0xf;
const BME680_CTRL_MEAS_FORCED_MODE_BIT: u8 = 1;
const BME680_CTRL_MEAS_PRESSURE_SHL: u8 = 2;
const BME680_CTRL_MEAS_TEMPERATURE_SHL: u8 = 5;
const BME680_CONTROL_IIR_FILTER_SHL: u8 = 2;
const BME680_CTRL_GAS_0_HEATER_MASK: u8 = 0x0f;
const BME680_CTRL_GAS_0_HEATER_SHL: u8 = 3;
const BME680_NB_CONV_NB_CONV_MASK: u8 = 0x0f;
const BME680_NB_CONV_RUN_GAS_SHL: u8 = 4;
const BME680_NB_CONV_RUN_GAS_MASK: u8 = 0xef;
const BME680_BIT_H1_DATA_MASK: u8 = 0x0f;
const BME680_RHRANGE_MASK: u8 = 0x30;
const BME68X_RSERROR_MASK: u8 = 0xf0;
const BME680_GAS_WAIT_MULT_FACT_SHL: u8 = 6;
const BME680_GAS_VALID_BIT: u8 = 0x20;
const BME680_HEAT_STAB_BIT: u8 = 0x10;

// -- list of gas ranges and corresponding constants used for the resistance calculation
const GAS_RANGE_C1: [f64; 16] = [
    1.0, 1.0, 1.0, 1.0, 1.0, 0.99, 1.0, 0.992,
    1.0, 1.0, 0.998, 0.995, 1.0, 0.99, 1.0, 1.0
];
const GAS_RANGE_C2: [f64; 16] = [
    8000000.0, 4000000.0, 2000000.0, 1000000.0, 499500.4995, 248262.1648, 125000.0, 63004.03226,
    31281.28128, 15625.0, 7812.5, 3906.25, 1953.125, 976.5625, 488.28125, 244.140625
];

#[derive(Clone, Debug, PartialEq)]
pub enum Bme680DeviceAddress {
    Default,
    Secondary,
}

impl Default for Bme680DeviceAddress {
    fn default() -> Self {
        Bme680DeviceAddress::Default
    }
}

impl Bme680DeviceAddress {
    const BME680_DEV_ADDR_DEFAULT: u16 = 0x77;
    const BME680_DEV_ADDR_SECONDARY: u16 = 0x76;

    fn value(&self) -> u16 {
        match *self {
            Self::Default => Self::BME680_DEV_ADDR_DEFAULT,
            Self::Secondary => Self::BME680_DEV_ADDR_SECONDARY,
        }
    }
}

pub enum Bme680SensorPowerMode {
    Sleep,
    Forced,
}

impl Bme680SensorPowerMode {
    const BME680_POWERMODE_SLEEP: u8 = 0x00;
    const BME680_POWERMODE_FORCED: u8 = 0x01;

    fn value(&self) -> u8 {
        match *self {
            Self::Sleep => Self::BME680_POWERMODE_SLEEP,
            Self::Forced => Self::BME680_POWERMODE_FORCED,
        }
    }
}

impl fmt::Display for Bme680SensorPowerMode {
    // This trait requires `fmt` with this exact signature.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::Sleep => write!(f, "Sleep/{:#04x}", self.value()),
            Self::Forced => write!(f, "Forced/{:#04x}", self.value()),
        }
    }
}

pub enum Bme680OverSampling {
    NoOversampling,
    Oversampling1x,
    Oversampling2x,
    Oversampling4x,
    Oversampling8x,
    Oversampling16x,
}

impl Bme680OverSampling {
    const BME680_NO_OVERSAMPLING: u8 = 0x00;
    const BME680_OVERSAMPLING_1X: u8 = 0x01;
    const BME680_OVERSAMPLING_2X: u8 = 0x02;
    const BME680_OVERSAMPLING_4X: u8 = 0x03;
    const BME680_OVERSAMPLING_8X: u8 = 0x04;
    const BME680_OVERSAMPLING_16X: u8 = 0x05;

    fn value(&self) -> u8 {
        match *self {
            Self::NoOversampling => Self::BME680_NO_OVERSAMPLING,
            Self::Oversampling1x => Self::BME680_OVERSAMPLING_1X,
            Self::Oversampling2x => Self::BME680_OVERSAMPLING_2X,
            Self::Oversampling4x => Self::BME680_OVERSAMPLING_4X,
            Self::Oversampling8x => Self::BME680_OVERSAMPLING_8X,
            Self::Oversampling16x => Self::BME680_OVERSAMPLING_16X,
        }
    }
}

impl fmt::Display for Bme680OverSampling {
    // This trait requires `fmt` with this exact signature.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::NoOversampling => write!(f, "NoOversampling/{:#04x}", self.value()),
            Self::Oversampling1x => write!(f, "Oversampling1x/{:#04x}", self.value()),
            Self::Oversampling2x => write!(f, "Oversampling2x/{:#04x}", self.value()),
            Self::Oversampling4x => write!(f, "Oversampling4x/{:#04x}", self.value()),
            Self::Oversampling8x => write!(f, "Oversampling8x/{:#04x}", self.value()),
            Self::Oversampling16x => write!(f, "Oversampling16x/{:#04x}", self.value()),
        }
    }
}

#[allow(dead_code)]
pub enum Bme680IrrFilter {
    FilterOff,
    Coef3,
    Coef7,
    Coef31,
    Coef63,
    Coef127,
}

impl Bme680IrrFilter {
    const BME680_FILTER_COEFF_OFF: u8 = 0x00;
    const BME680_FILTER_COEFF_3: u8 = 0x02;
    const BME680_FILTER_COEFF_7: u8 = 0x03;
    const BME680_FILTER_COEFF_31: u8 = 0x05;
    const BME680_FILTER_COEFF_63: u8 = 0x06;
    const BME680_FILTER_COEFF_127: u8 = 0x07;

    fn value(&self) -> u8 {
        match *self {
            Self::FilterOff => Self::BME680_FILTER_COEFF_OFF,
            Self::Coef3 => Self::BME680_FILTER_COEFF_3,
            Self::Coef7 => Self::BME680_FILTER_COEFF_7,
            Self::Coef31 => Self::BME680_FILTER_COEFF_31,
            Self::Coef63 => Self::BME680_FILTER_COEFF_63,
            Self::Coef127 => Self::BME680_FILTER_COEFF_127,
        }
    }
}

impl fmt::Display for Bme680IrrFilter {
    // This trait requires `fmt` with this exact signature.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Self::FilterOff => write!(f, "FilterOff/{:#04x}", self.value()),
            Self::Coef3 => write!(f, "Coefficient 3/{:#04x}", self.value()),
            Self::Coef7 => write!(f, "Coefficient 7/{:#04x}", self.value()),
            Self::Coef31 => write!(f, "Coefficient 31/{:#04x}", self.value()),
            Self::Coef63 => write!(f, "Coefficient 63/{:#04x}", self.value()),
            Self::Coef127 => write!(f, "Coefficient 127/{:#04x}", self.value()),
        }
    }
}

pub enum Bme680HeaterProfile {
    SetPoint0, SetPoint1, SetPoint2, SetPoint3, SetPoint4,
    SetPoint5, SetPoint6, SetPoint7, SetPoint8, SetPoint9,
}

impl Bme680HeaterProfile {
    const BME680_SETPOINT_0: u8 = 0;
    const BME680_SETPOINT_1: u8 = 1;
    const BME680_SETPOINT_2: u8 = 2;
    const BME680_SETPOINT_3: u8 = 3;
    const BME680_SETPOINT_4: u8 = 4;
    const BME680_SETPOINT_5: u8 = 5;
    const BME680_SETPOINT_6: u8 = 6;
    const BME680_SETPOINT_7: u8 = 7;
    const BME680_SETPOINT_8: u8 = 8;
    const BME680_SETPOINT_9: u8 = 9;

    fn value(&self) -> u8 {
        match *self {
            Self::SetPoint0 => Self::BME680_SETPOINT_0,
            Self::SetPoint1 => Self::BME680_SETPOINT_1,
            Self::SetPoint2 => Self::BME680_SETPOINT_2,
            Self::SetPoint3 => Self::BME680_SETPOINT_3,
            Self::SetPoint4 => Self::BME680_SETPOINT_4,
            Self::SetPoint5 => Self::BME680_SETPOINT_5,
            Self::SetPoint6 => Self::BME680_SETPOINT_6,
            Self::SetPoint7 => Self::BME680_SETPOINT_7,
            Self::SetPoint8 => Self::BME680_SETPOINT_8,
            Self::SetPoint9 => Self::BME680_SETPOINT_9,
        }
    }
}

pub enum Bme680GasWaitMultiplicationFactor {
    X1,
    X4,
    X16,
    X64,
}

impl Bme680GasWaitMultiplicationFactor {
    fn value(&self) -> u8 {
        match *self {
            Self::X1 => 0,
            Self::X4 => 1,
            Self::X16 => 2,
            Self::X64 => 3,
        }
    }

}

#[derive(Debug)]
pub struct Bme680MeasuringStatus {
    pub new_data: bool,
    pub gas_measuring: bool,
    pub measuring: bool,
    pub gas_meas_index: u8,
}

#[derive(Debug)]
pub struct Bme680MeasuringResult {
    pub pressure_raw: u32,
    pub temperature_raw: u32,
    pub humidity_raw: u16,
}

#[derive(Debug)]
pub struct Bme680GasMeasuringResult {    
    pub gas_res: f64,
    pub gas_valid: bool,
    pub heat_stab: bool,
}

#[derive(Debug)]
struct CalibData
{
    // -- calibration coefficients for temperature
    par_t1: f64,
    par_t2: f64,
    par_t3: f64,
    // -- calibration coefficients for pressure
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
    // -- calibration coefficients for gas
    par_gh1: f64,
    par_gh2: f64,
    par_gh3: f64,
    res_heat_range: f64,
    res_heat_val: f64,
    range_sw_err: f64,
    // -- calibration coefficients for humidity
    par_h1: f64,
    par_h2: f64,
    par_h3: f64,
    par_h4: f64,
    par_h5: f64,
    par_h6: f64,
    par_h7: f64,
}

pub struct BME680 {
    // -- i2c bus
    i2c: I2c<File>,
    // -- device address.
    device_addr: Bme680DeviceAddress,
    // -- chip id
    chip_id: u8,
    // -- calibration params
    calib_data: CalibData,
}

impl BME680 {

    pub fn new(i2c_bus_path: &Path, device_addr: Bme680DeviceAddress,
        humidity_osr: Bme680OverSampling, pressure_osr: Bme680OverSampling,
        temperature_osr: Bme680OverSampling, irr_filter: Bme680IrrFilter) -> Result<BME680, std::io::Error> {
        // -- get the bus
        let mut i2c = i2cio::get_bus(i2c_bus_path)?;
        // -- set device address
        i2cio::set_slave(&mut i2c, device_addr.value())?;
        // -- check if device is available by reading chip id
        let chip_id = i2cio::read_byte(&mut i2c, BME680_REG_CHIP_ID)?;
        if chip_id != BME680_CHIP_ID {
            let errmsg = format!("Found unknown chip id '{chip_id:#04x}', expected '{BME680_CHIP_ID:#04x}'");
            return Err(std::io::Error::new(std::io::ErrorKind::Other, errmsg))
        }
        debug!("Got chip id: {chip_id:#x}");
        let calib_data = Self::get_calib_data(&mut i2c)?;
        debug!("Got calibration data: {calib_data:#?}");
        let mut bme680 = BME680 {
            i2c,
            device_addr,
            chip_id,
            calib_data,
            //uncomp_data: Default::default(),
        };
        // -- do a soft reset since it's in an unknown state
        bme680.soft_reset()?;
        // -- set oversampling rates
        bme680.set_humidity_osr(humidity_osr)?;
        bme680.set_pressure_and_temperature_osr(pressure_osr, temperature_osr)?;
        // -- set filter
        bme680.set_irr_filter(irr_filter)?;
        // -- get calibration data
        //let calib_data = Self::get_calib_data(&mut i2c)?;
        // -- return initialized structure
        Ok(bme680)
    }

    #[allow(dead_code)]
    pub fn get_device_addr(&self) -> Bme680DeviceAddress {
        self.device_addr.clone()
    }

    #[allow(dead_code)]
    pub fn get_chip_id(&self) -> u8 {
        self.chip_id.clone()
    }

    pub fn soft_reset(&mut self) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_RESET;
        // -- initiate soft reset
        debug!("Initiating soft reset");
        i2cio::write_byte(&mut self.i2c, REG, BME680_COMMAND_SOFT_RESET)?;
        // -- wait for the device to startup
        let startup_delay = time::Duration::from_millis(BME680_STARTUP_DELAY_MS);
        thread::sleep(startup_delay);
        Ok(())
    }

    fn concat_bytes(msb: u8, lsb: u8) -> u16 {
        ((msb as u16) << 8) | (lsb as u16)
    }

    fn get_calib_data(i2c: &mut I2c<File>) -> Result<CalibData, std::io::Error> {
        // -- read calibration data block 1
        const REG_1: u8 = BME680_REG_CALIB_DATA1_BASE;
        const LEN_1: usize = BME680_CALIB_DATA1_LEN;
        let mut reg_data_1: [u8; LEN_1] = [0; LEN_1];
        let _bytes_read = i2c.i2c_read_block_data(REG_1, &mut reg_data_1)?;
        debug!("Read {_bytes_read} bytes of calibration data, block 1");
        // -- read calibration data block 2
        const REG_2: u8 = BME680_REG_CALIB_DATA2_BASE;
        const LEN_2: usize = BME680_CALIB_DATA2_LEN;
        let mut reg_data_2: [u8; LEN_2] = [0; LEN_2];
        let _bytes_read = i2c.i2c_read_block_data(REG_2, &mut reg_data_2)?;
        debug!("Read {_bytes_read} bytes of calibration data, block 2");
        // -- read calibration data block 3
        const REG_3: u8 = BME680_REG_CALIB_DATA3_BASE;
        const LEN_3: usize = BME680_CALIB_DATA3_LEN;
        let mut reg_data_3: [u8; LEN_3] = [0; LEN_3];
        let _bytes_read = i2c.i2c_read_block_data(REG_3, &mut reg_data_3)?;
        debug!("Read {_bytes_read} bytes of calibration data, block 3");
        // -- concat arrays
        let coeff_array = [reg_data_1.as_slice(), reg_data_2.as_slice(), reg_data_3.as_slice()].concat();

        // -- get calibration data for temperatire
        let par_t1 = Self::concat_bytes(coeff_array[BME680_IDX_T1_MSB], coeff_array[BME680_IDX_T1_LSB]) as f64;
        let par_t2 = Self::concat_bytes(coeff_array[BME680_IDX_T2_MSB], coeff_array[BME680_IDX_T2_LSB]) as f64;
        let par_t3 = coeff_array[BME680_IDX_T3] as f64;
        debug!("Read temperatire calibration data {par_t1} {par_t2} {par_t3} ");

        // -- get calibration data for pressure
        let par_p1 = Self::concat_bytes(coeff_array[BME68X_IDX_P1_MSB], coeff_array[BME68X_IDX_P1_LSB]) as f64;
        let par_p2 = (Self::concat_bytes(coeff_array[BME68X_IDX_P2_MSB], coeff_array[BME68X_IDX_P2_LSB]) as i16) as f64;
        let par_p3 = (coeff_array[BME68X_IDX_P3] as i8) as f64;
        let par_p4 = (Self::concat_bytes(coeff_array[BME68X_IDX_P4_MSB], coeff_array[BME68X_IDX_P4_LSB]) as i16) as f64;
        let par_p5 = (Self::concat_bytes(coeff_array[BME68X_IDX_P5_MSB], coeff_array[BME68X_IDX_P5_LSB]) as i16) as f64;
        let par_p6 = (coeff_array[BME68X_IDX_P6] as i8) as f64;
        let par_p7 = (coeff_array[BME68X_IDX_P7] as i8) as f64;
        let par_p8 = (Self::concat_bytes(coeff_array[BME68X_IDX_P8_MSB], coeff_array[BME68X_IDX_P8_LSB]) as i16) as f64;
        let par_p9 = (Self::concat_bytes(coeff_array[BME68X_IDX_P9_MSB], coeff_array[BME68X_IDX_P9_LSB]) as i16) as f64;
        let par_p10 = coeff_array[BME68X_IDX_P10] as f64;
        debug!("Read pressure calibration data {par_p1} {par_p2} {par_p3} {par_p4} {par_p5} {par_p6} {par_p7} {par_p8} {par_p9} {par_p10}");

        // -- get calibration data for humidity
        let par_h1 = (((coeff_array[BME68X_IDX_H1_MSB] as u16) << 4) |
                        ((coeff_array[BME68X_IDX_H1_LSB] & BME680_BIT_H1_DATA_MASK) as u16)) as f64;
        let par_h2 = (((coeff_array[BME68X_IDX_H2_MSB] as u16) << 4) |
                        ((coeff_array[BME68X_IDX_H2_LSB] >> 4) as u16)) as f64;
        let par_h3 = (coeff_array[BME68X_IDX_H3] as i8) as f64;
        let par_h4 = (coeff_array[BME68X_IDX_H4] as i8) as f64;
        let par_h5 = (coeff_array[BME68X_IDX_H5] as i8) as f64;
        let par_h6 = coeff_array[BME68X_IDX_H6] as f64;
        let par_h7 = (coeff_array[BME68X_IDX_H7] as i8) as f64;
        debug!("Read humidity calibration data {par_h1} {par_h2} {par_h3} {par_h4} {par_h5} {par_h6} {par_h7}");


        // -- get calibration data related to gas heater
        let par_gh1 = (coeff_array[BME68X_IDX_GH1] as i8) as f64;
        let par_gh2 = (Self::concat_bytes(coeff_array[BME68X_IDX_GH2_MSB], coeff_array[BME68X_IDX_GH2_LSB]) as i16) as f64;
        let par_gh3 = (coeff_array[BME68X_IDX_GH3] as i8) as f64;
        let res_heat_range = ((coeff_array[BME68X_IDX_RES_HEAT_RANGE] & BME680_RHRANGE_MASK) / 16) as f64;
        let res_heat_val = (coeff_array[BME68X_IDX_RES_HEAT_VAL] as i8) as f64;
        let range_sw_err = (((coeff_array[BME68X_IDX_RANGE_SW_ERR] & BME68X_RSERROR_MASK) as i8) / 16) as f64;
        debug!("Read gas heater calibration data {par_gh1} {par_gh2} {par_gh3} {res_heat_range} {res_heat_val} {range_sw_err}");

        // -- return structured calibration data
        Ok(CalibData {
            par_t1, par_t2, par_t3,
            par_p1, par_p2, par_p3, par_p4, par_p5,
            par_p6, par_p7, par_p8, par_p9, par_p10,
            par_gh1, par_gh2, par_gh3, res_heat_range,
            res_heat_val, range_sw_err,
            par_h1, par_h2, par_h3, par_h4, par_h5,
            par_h6, par_h7,
        })
    }

    pub fn get_meas_status(&mut self) -> Result<Bme680MeasuringStatus, std::io::Error> {
        const REG: u8 = BME680_REG_MEAS_STATUS_0;
        // -- read current value
        let reg_val = i2cio::read_byte(&mut self.i2c, REG)?;
        // -- extract status values
        let new_data = (reg_val & BME680_MEAS_STATUS_0_NEW_DATA_BIT) > 0;
        let gas_measuring = (reg_val & BME680_MEAS_STATUS_0_GAS_MEASURING_BIT) > 0;
        let measuring = (reg_val & BME680_MEAS_STATUS_0_MEASURING_BIT) > 0;
        let gas_meas_index = reg_val & BME680_MEAS_STATUS_0_GAS_MEAS_INDEX_MASK;
        Ok(Bme680MeasuringStatus {
            new_data, gas_measuring, measuring, gas_meas_index,
        })
    }

    pub fn get_meas_result(&mut self) -> Result<Bme680MeasuringResult, std::io::Error> {
        const REG: u8 = BME680_REG_MEAS_RESULT_BASE;
        const LEN: usize = BME680_MEAS_RESULT_LEN;
        let mut reg_data: [u8; LEN] = [0; LEN];
        // -- read current value and mask out run gas bit
        let _bytes_read = self.i2c.i2c_read_block_data(REG, &mut reg_data)?;
        debug!("Read {_bytes_read} bytes of resulting data after measuring");
        // -- store register values for pressure data
        let data_msb = (reg_data[0] as u32) << BME680_12_BIT_SHIFT;
        let data_lsb = (reg_data[1] as u32) << BME680_4_BIT_SHIFT;
        let data_xlsb = (reg_data[2] as u32) >> BME680_4_BIT_SHIFT;
        let pressure_raw = data_msb | data_lsb | data_xlsb;
        // -- store register values for temperature data
        let data_msb = (reg_data[3] as u32) << BME680_12_BIT_SHIFT;
        let data_lsb = (reg_data[4] as u32) << BME680_4_BIT_SHIFT;
        let data_xlsb = (reg_data[5] as u32) >> BME680_4_BIT_SHIFT;
        let temperature_raw = data_msb | data_lsb | data_xlsb;
        // -- store register values for humidity data
        let data_msb = (reg_data[6] as u16) << BME680_8_BIT_SHIFT;
        let data_lsb = reg_data[7] as u16;
        let humidity_raw = data_msb | data_lsb;

        Ok(Bme680MeasuringResult {
            pressure_raw, temperature_raw, humidity_raw
        })
    }

    pub fn get_gas_meas_result(&mut self) -> Result<Bme680GasMeasuringResult, std::io::Error> {
        // -- read current value
        let data_msb = i2cio::read_byte(&mut self.i2c, BME680_REG_GAS_ACD_MSB)?;
        let data_lsb = i2cio::read_byte(&mut self.i2c, BME680_REG_GAS_ACD_LSB_RANGE)?;
        let gas_adc = ((data_msb as u16) << BME680_2_BIT_SHIFT) | ((data_lsb as u16) >> BME680_6_BIT_SHIFT);
        let gas_range = (data_lsb & BME680_4_BIT_MASK) as usize;
        let gas_valid = (data_lsb & BME680_GAS_VALID_BIT) > 0;
        let heat_stab = (data_lsb & BME680_HEAT_STAB_BIT) > 0;
        let range_switching_error = self.calib_data.range_sw_err;
        let var1 = (1340.0 + (5.0 * range_switching_error)) * GAS_RANGE_C1[gas_range];
        let gas_res = var1 * GAS_RANGE_C2[gas_range] / (gas_adc as f64 - 512.0 + var1);
        let result = Bme680GasMeasuringResult {
            gas_res, gas_valid, heat_stab,
        };
        Ok(result)
        // const LOOKUP_K1_RANGE: [f64; 16] = [
        //     0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8, 0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0, 0.0,
        // ];
        // const LOOKUP_K2_RANGE: [f64; 16] = [
        //     0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        // ];
        // let gas_range_f = (1 << gas_range) as f64;
        // let var1 = (1340.0 + (5.0 * range_switching_error));
        // let var2 = var1 * (1.0 + LOOKUP_K1_RANGE[gas_range] / 100.0);
        // let var3 = 1.0 + (LOOKUP_K2_RANGE[gas_range] / 100.0);
        // let gas_res = 1.0 / (var3 * (0.000000125) * gas_range_f * (((gas_adc - 512.0) / var2) + 1.0));
        // Ok(gas_res)
    }

    pub fn get_temperature(&self, temperature_raw: u32) -> (f64, f64) {
        let temperature_raw = temperature_raw as f64;
        let par_t1 = self.calib_data.par_t1;
        let par_t2 = self.calib_data.par_t2;
        let par_t3 = self.calib_data.par_t3;
        let var1 = ((temperature_raw / 16384.0) - (par_t1 / 1024.0)) * par_t2;
        let var2 = (((temperature_raw / 131072.0) - (par_t1 / 8192.0)) * ((temperature_raw / 131072.0) - (par_t1 / 8192.0))) * (par_t3 * 16.0);
        let t_fine = var1 + var2;
        let temp_comp = t_fine / 5120.0;
        (temp_comp, t_fine)
    }

    pub fn get_pressure(&self, pressure_raw: u32, t_fine: f64) -> f64 {
        let pressure_raw = pressure_raw as f64;
        let par_p1 = self.calib_data.par_p1;
        let par_p2 = self.calib_data.par_p2;
        let par_p3 = self.calib_data.par_p3;
        let par_p4 = self.calib_data.par_p4;
        let par_p5 = self.calib_data.par_p5;
        let par_p6 = self.calib_data.par_p6;
        let par_p7 = self.calib_data.par_p7;
        let par_p8 = self.calib_data.par_p8;
        let par_p9 = self.calib_data.par_p9;
        let par_p10 = self.calib_data.par_p10;
        let var1 = (t_fine / 2.0) - 64000.0;
        let var2 = var1 * var1 * (par_p6 / 131072.0);
        let var2 = var2 + (var1 * par_p5 * 2.0);
        let var2 = (var2 / 4.0) + (par_p4 * 65536.0);
        let var1 = (((par_p3 * var1 * var1) / 16384.0) + (par_p2 * var1)) / 524288.0;
        let var1 = (1.0 + (var1 / 32768.0)) * par_p1;
        let press_comp = 1048576.0 - pressure_raw;
        let press_comp = ((press_comp - (var2 / 4096.0)) * 6250.0) / var1;
        let var1 = (par_p9 * press_comp * press_comp) / 2147483648.0;
        let var2 = press_comp * (par_p8 / 32768.0);
        let var3 = (press_comp / 256.0) * (press_comp / 256.0) * (press_comp / 256.0) * (par_p10 / 131072.0);
        let press_comp = press_comp + (var1 + var2 + var3 + (par_p7 * 128.0)) / 16.0;
        press_comp
    }

    pub fn get_humidity(&self, humidity_raw: u16, temperature: f64) -> f64 {
        let humidity_raw = humidity_raw as f64;
        let par_h1 = self.calib_data.par_h1;
        let par_h2 = self.calib_data.par_h2;
        let par_h3 = self.calib_data.par_h3;
        let par_h4 = self.calib_data.par_h4;
        let par_h5 = self.calib_data.par_h5;
        let par_h6 = self.calib_data.par_h6;
        let par_h7 = self.calib_data.par_h7;

        let var1 = humidity_raw - ((par_h1 * 16.0) + ((par_h3 / 2.0) * temperature));
        let var2 = var1 * ((par_h2 / 262144.0) * (1.0 + ((par_h4 / 16384.0) * temperature) + ((par_h5 / 1048576.0) * temperature * temperature)));
        let var3 = par_h6 / 16384.0;
        let var4 = par_h7 / 2097152.0;
        let hum_comp = var2 + ((var3 + (var4 * temperature)) * var2 * var2);
        hum_comp
    }

    pub fn set_forced_mode(&mut self) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_CTRL_MEAS;
        // -- read current value, set forced mode bit
        let reg_val = i2cio::read_byte(&mut self.i2c, REG)?;
        let reg_val = reg_val | BME680_CTRL_MEAS_FORCED_MODE_BIT;
        // -- write back register value / set power mode forced
        debug!("Setting power mode forced");
        i2cio::write_byte(&mut self.i2c, REG, reg_val)
    }

    pub fn set_humidity_osr(&mut self, humidity_osr: Bme680OverSampling) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_CTRL_HUM;
        // -- set oversampling rate for humidity
        debug!("Setting humidity oversampling rate");
        i2cio::write_byte(&mut self.i2c, REG, humidity_osr.value())
    }

    pub fn set_pressure_and_temperature_osr(&mut self, pressure_osr: Bme680OverSampling,
        temperature_osr: Bme680OverSampling) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_CTRL_MEAS;
        // -- put bits for OSR in place, power mode implicit set to sleep (bit 0 and 1)
        let reg_val = temperature_osr.value() << BME680_CTRL_MEAS_TEMPERATURE_SHL
            | pressure_osr.value() <<  BME680_CTRL_MEAS_PRESSURE_SHL;
        // -- set oversampling rate for pressure and temperature
        debug!("Setting pressure and temperature oversampling rate to {reg_val:#010b}");
        i2cio::write_byte(&mut self.i2c, REG, reg_val)
    }

    pub fn set_irr_filter(&mut self, irr_filter: Bme680IrrFilter) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_CONFIG;
        let reg_val:u8 = irr_filter.value() <<  BME680_CONTROL_IIR_FILTER_SHL;
        // -- set infinite impulse response (IIR) filter
        debug!("Setting IRR filter");
        i2cio::write_byte(&mut self.i2c, REG, reg_val)
    }

    pub fn enable_heater(&mut self) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_CTRL_GAS_0;
        // -- read current value, set heater bit
        let reg_val = i2cio::read_byte(&mut self.i2c, REG)?;
        let reg_val = reg_val | BME680_CTRL_GAS_0_HEATER_SHL;
        // -- write back register value
        debug!("Enabling heater");
        i2cio::write_byte(&mut self.i2c, REG, reg_val)
    }

    pub fn disable_heater(&mut self) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_CTRL_GAS_0;
        // -- read current value, mask out heater bit
        let reg_val = i2cio::read_byte(&mut self.i2c, REG)?;
        let reg_val = reg_val & BME680_CTRL_GAS_0_HEATER_MASK;
        // -- write back register value
        debug!("Disabling heater");
        i2cio::write_byte(&mut self.i2c, REG, reg_val)
    }

    pub fn set_heater_profile(&mut self, heater_profile: Bme680HeaterProfile) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_CTRL_GAS_1;
        // -- read current value, mask out nb conv bits and set requested bits
        let reg_val = i2cio::read_byte(&mut self.i2c, REG)?;
        let reg_val = reg_val & BME680_NB_CONV_NB_CONV_MASK;
        let reg_val = reg_val | heater_profile.value();
        // -- write back register value
        debug!("Setting heater profile");
        i2cio::write_byte(&mut self.i2c, REG, reg_val)
    }

    pub fn enable_run_gas(&mut self) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_CTRL_GAS_1;
        // -- read current value and set run gas bit
        let reg_val = i2cio::read_byte(&mut self.i2c, REG)?;
        let reg_val = reg_val | (1 <<  BME680_NB_CONV_RUN_GAS_SHL);
        // -- write back register value
        debug!("Enable run gas");
        i2cio::write_byte(&mut self.i2c, REG, reg_val)
    }

    pub fn disable_run_gas(&mut self) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_CTRL_GAS_1;
        // -- read current value and mask out run gas bit
        let reg_val = i2cio::read_byte(&mut self.i2c, REG)?;
        let reg_val = reg_val & BME680_NB_CONV_RUN_GAS_MASK;
        // -- write back register value
        debug!("Disable run gas");
        i2cio::write_byte(&mut self.i2c, REG, reg_val)
    }


    pub fn calc_res_heat(&self, amb_temp: f64, target_temp: f64) -> u8 {
        let var1 = ((self.calib_data.par_gh1 as f64) / 16.0) + 49.0;
        let var2 = (((self.calib_data.par_gh2 as f64) / 32768.0) * 0.0005) + 0.00235;
        let var3 = (self.calib_data.par_gh3 as f64)  / 1024.0;
        let var4 = var1 * (1.0 + (var2 * target_temp));
        let var5 = var4 + (var3 * amb_temp);
        let res_heat = (3.4 * ((var5 * (4.0 / (4.0 + self.calib_data.res_heat_range)) * (1.0/(1.0 +
        (self.calib_data.res_heat_val * 0.002)))) - 25.0)) as u8;
        res_heat
    }

    // pub fn get_ldac_heat(&mut self) -> Result<Vec<u8>, std::io::Error> {
    //     const REG: u8 = BME680_REG_IDAC_HEAT_BASE;
    //     let mut reg_data: [u8; BME680_IDAC_HEAT_BASE_LEN] = [0; BME680_IDAC_HEAT_BASE_LEN];
    //     // -- read current value and mask out run gas bit
    //     let reg_val = self.i2c.i2c_read_block_data(REG, &mut reg_data)?;
    //     Ok(Vec::from(reg_data))
    // }

    pub fn get_res_heat(&mut self) -> Result<Vec<u8>, std::io::Error> {
        const REG: u8 = BME680_REG_RES_HEAT_BASE;
        let mut reg_data: [u8; BME680_RES_HEAT_BASE_LEN] = [0; BME680_RES_HEAT_BASE_LEN];
        // -- read current value and mask out run gas bit
        let _bytes_read = self.i2c.i2c_read_block_data(REG, &mut reg_data)?;
        Ok(Vec::from(reg_data))
    }

    fn set_res_heat(&mut self, reg: u8, res_heat: u8) -> Result<(), std::io::Error> {
        if reg < BME680_REG_RES_HEAT_BASE || reg > BME680_REG_RES_HEAT_BASE + (BME680_RES_HEAT_BASE_LEN as u8) {
            return Err(std::io::Error::other(format!("Invalid register for gas wait: {reg:#04x}")))
        }
        // -- write back register value
        debug!("Setting heater resistance {} to {res_heat:#010b}", reg - BME680_REG_RES_HEAT_BASE);
        i2cio::write_byte(&mut self.i2c, reg, res_heat)
    }

    pub fn set_res_heat_0(&mut self, res_heat: u8) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_RES_HEAT_BASE;
        self.set_res_heat(REG, res_heat)
    }

    pub fn set_res_heat_1(&mut self, res_heat: u8) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_RES_HEAT_BASE + 1;
        self.set_res_heat(REG, res_heat)
    }

    pub fn set_res_heat_2(&mut self, res_heat: u8) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_RES_HEAT_BASE + 2;
        self.set_res_heat(REG, res_heat)
    }

    pub fn set_res_heat_3(&mut self, res_heat: u8) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_RES_HEAT_BASE + 3;
        self.set_res_heat(REG, res_heat)
    }

    pub fn set_res_heat_4(&mut self, res_heat: u8) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_RES_HEAT_BASE + 4;
        self.set_res_heat(REG, res_heat)
    }

    pub fn set_res_heat_5(&mut self, res_heat: u8) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_RES_HEAT_BASE + 5;
        self.set_res_heat(REG, res_heat)
    }

    pub fn set_res_heat_6(&mut self, res_heat: u8) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_RES_HEAT_BASE + 6;
        self.set_res_heat(REG, res_heat)
    }

    pub fn set_res_heat_7(&mut self, res_heat: u8) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_RES_HEAT_BASE + 7;
        self.set_res_heat(REG, res_heat)
    }

    pub fn set_res_heat_8(&mut self, res_heat: u8) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_RES_HEAT_BASE + 8;
        self.set_res_heat(REG, res_heat)
    }

    pub fn set_res_heat_9(&mut self, res_heat: u8) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_RES_HEAT_BASE + 9;
        self.set_res_heat(REG, res_heat)
    }

    pub fn get_gas_wait(&mut self) -> Result<Vec<u8>, std::io::Error> {
        const REG: u8 = BME680_REG_GAS_WAIT_BASE;
        let mut reg_data: [u8; BME680_GAS_WAIT_BASE_LEN] = [0; BME680_GAS_WAIT_BASE_LEN];
        // -- read current value and mask out run gas bit
        let _bytes_read = self.i2c.i2c_read_block_data(REG, &mut reg_data)?;
        Ok(Vec::from(reg_data))
    }

    fn set_gas_wait(&mut self, reg: u8, milli_secs: u8, mult_fact: Bme680GasWaitMultiplicationFactor) -> Result<(), std::io::Error> {
        let milli_secs = if milli_secs > 64 {
            64
        } else {
            milli_secs
        };
        let mult_fact = mult_fact.value();
        let reg_val = mult_fact << BME680_GAS_WAIT_MULT_FACT_SHL | milli_secs;
        if reg < BME680_REG_GAS_WAIT_BASE || reg > BME680_REG_GAS_WAIT_BASE + (BME680_GAS_WAIT_BASE_LEN as u8) {
            return Err(std::io::Error::other(format!("Invalid register for gas wait: {reg:#04x}")))
        }
        // -- write back register value
        debug!("Setting gas wait {} to {reg_val:#010b} / {reg_val:#04x}", reg - BME680_REG_GAS_WAIT_BASE);
        i2cio::write_byte(&mut self.i2c, reg, reg_val)
    }

    pub fn set_gas_wait_0(&mut self, milli_secs: u8, mult_fact: Bme680GasWaitMultiplicationFactor) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_GAS_WAIT_BASE;
        self.set_gas_wait(REG, milli_secs, mult_fact)
    }

    pub fn set_gas_wait_1(&mut self, milli_secs: u8, mult_fact: Bme680GasWaitMultiplicationFactor) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_GAS_WAIT_BASE + 1;
        self.set_gas_wait(REG, milli_secs, mult_fact)
    }

    pub fn set_gas_wait_2(&mut self, milli_secs: u8, mult_fact: Bme680GasWaitMultiplicationFactor) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_GAS_WAIT_BASE + 2;
        self.set_gas_wait(REG, milli_secs, mult_fact)
    }

    pub fn set_gas_wait_3(&mut self, milli_secs: u8, mult_fact: Bme680GasWaitMultiplicationFactor) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_GAS_WAIT_BASE + 3;
        self.set_gas_wait(REG, milli_secs, mult_fact)
    }

    pub fn set_gas_wait_4(&mut self, milli_secs: u8, mult_fact: Bme680GasWaitMultiplicationFactor) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_GAS_WAIT_BASE + 4;
        self.set_gas_wait(REG, milli_secs, mult_fact)
    }

    pub fn set_gas_wait_5(&mut self, milli_secs: u8, mult_fact: Bme680GasWaitMultiplicationFactor) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_GAS_WAIT_BASE + 5;
        self.set_gas_wait(REG, milli_secs, mult_fact)
    }

    pub fn set_gas_wait_6(&mut self, milli_secs: u8, mult_fact: Bme680GasWaitMultiplicationFactor) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_GAS_WAIT_BASE + 6;
        self.set_gas_wait(REG, milli_secs, mult_fact)
    }

    pub fn set_gas_wait_7(&mut self, milli_secs: u8, mult_fact: Bme680GasWaitMultiplicationFactor) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_GAS_WAIT_BASE + 7;
        self.set_gas_wait(REG, milli_secs, mult_fact)
    }

    pub fn set_gas_wait_8(&mut self, milli_secs: u8, mult_fact: Bme680GasWaitMultiplicationFactor) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_GAS_WAIT_BASE + 8;
        self.set_gas_wait(REG, milli_secs, mult_fact)
    }

    pub fn set_gas_wait_9(&mut self, milli_secs: u8, mult_fact: Bme680GasWaitMultiplicationFactor) -> Result<(), std::io::Error> {
        const REG: u8 = BME680_REG_GAS_WAIT_BASE + 9;
        self.set_gas_wait(REG, milli_secs, mult_fact)
    }

}