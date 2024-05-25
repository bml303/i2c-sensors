use chrono::Local;
use clap::{Parser, ValueEnum};
use log::{error, info};
use std::path::Path;
use std::process::ExitCode;
use std::{thread, time};

use i2c_sensors::bme388::{
    BME388, BME388DeviceAddress,
    BME388IrrFilter, BME388OutputDataRate,
    BME388OverSamplingPr, BME388OverSamplingTp,
    BME388SensorPowerMode, BME388StatusPressureSensor,
    BME388StatusTemperatureSensor,
    BME388StatusPressureData, BME388StatusTemperatureData,
};

const EXIT_CODE_SET_CTR_C_HNDLR_FAILED: u8 = 0x02;
const EXIT_CODE_BME388_INIT_FAILED: u8 = 0x61;
const EXIT_CODE_BME388_SET_NORMAL_POWER_MODE_FAILED: u8 = 0x62;
const EXIT_CODE_BME388_SET_FORCED_POWER_MODE_FAILED: u8 = 0x63;
const EXIT_CODE_BME388_GET_STATUS_FAILED: u8 = 0x64;
const EXIT_CODE_BME388_GET_DATA_RAW_FAILED: u8 = 0x65;
const EXIT_CODE_BME388_GET_SENSOR_MODE_FAILED: u8 = 0x67;

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum)]
enum AcquisitionMode {
    Forced,
    Normal,
}

#[derive(Parser)]
struct Args {
    // -- i2c bus device
    bus_path: String,
    #[clap(value_enum)]
    mode: AcquisitionMode,
}

fn get_sensor_settings(mode: &AcquisitionMode) -> (BME388OverSamplingPr, BME388OverSamplingTp, 
    BME388IrrFilter, BME388OutputDataRate) {    
    match mode {
        AcquisitionMode::Forced => (BME388OverSamplingPr::UltraLowX1, BME388OverSamplingTp::X1, 
            BME388IrrFilter::Off, BME388OutputDataRate::Odr0p78),
        AcquisitionMode::Normal => (BME388OverSamplingPr::HighX8, BME388OverSamplingTp::X1, 
            BME388IrrFilter::Coef1, BME388OutputDataRate::Odr0p78),
    }

}

fn main() -> ExitCode {

    // -- read .env file
    dotenv::dotenv().ok();
    // -- setup logger
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let now = Local::now();
    info!("Starting up: {now}");

    let args = Args::parse();
    let bus_path = args.bus_path;
    info!("Using i2c bus device {bus_path}");

    // -- set handler for Ctrl-C
    if let Err(err) = ctrlc::set_handler(move || {
        info!("Received Ctrl+C, terminating...");
        std::process::exit(0);
    }) {
        error!("ERROR - Failed to set Ctrl-C handler: {err}");
        return ExitCode::from(EXIT_CODE_SET_CTR_C_HNDLR_FAILED);
    }

    info!("Initializing BME388");
    let bus_path = Path::new(&bus_path);
    let dev_addr = BME388DeviceAddress::Default;
    let (osr_p, osr_t, irr_filter, odr) = get_sensor_settings(&args.mode);
    let mut bme388 = match BME388::new(bus_path, dev_addr, osr_p, osr_t, irr_filter, odr) {
        Ok(bme388) => bme388,
        Err(err) => {
            error!("ERROR - Failed to initialize BME388: {err}");
            return ExitCode::from(EXIT_CODE_BME388_INIT_FAILED);
        }
    };
    // -- start reading data
    let enable_pressure = BME388StatusPressureSensor::Enabled;
    let enable_temperature = BME388StatusTemperatureSensor::Enabled;
    if args.mode == AcquisitionMode::Normal {
        info!("Setting normal mode");
        if let Err(err) = bme388.set_sensor_mode(BME388SensorPowerMode::Normal, enable_pressure, enable_temperature) {
            error!("ERROR - Failed to set BME388 to normal power mode: {err}");
            return ExitCode::from(EXIT_CODE_BME388_SET_NORMAL_POWER_MODE_FAILED);
        }
        // -- wait for data acquisiton
        let data_acquisition_delay = time::Duration::from_millis(500);
        thread::sleep(data_acquisition_delay);
    }
    loop {
        if args.mode == AcquisitionMode::Forced {
            info!("Setting forced mode");
            if let Err(err) = bme388.set_sensor_mode(BME388SensorPowerMode::Forced, BME388StatusPressureSensor::Enabled, BME388StatusTemperatureSensor::Enabled) {
                error!("ERROR - Failed to set BME388 to forced power mode: {err}");
                return ExitCode::from(EXIT_CODE_BME388_SET_FORCED_POWER_MODE_FAILED);
            }
        }
        loop {
            match args.mode {
                AcquisitionMode::Forced => {
                    // -- forced mode
                    let (power_mode, p_enabled, t_enabled) = match bme388.get_sensor_mode() {
                        Ok(vals) => (vals.0, vals.1, vals.2),
                        Err(err) => {
                            error!("ERROR - Failed to get BME388 sensor mode: {err}");
                            return ExitCode::from(EXIT_CODE_BME388_GET_SENSOR_MODE_FAILED);
                        }
                    };
                    info!("Got mode '{power_mode}', '{p_enabled}', '{t_enabled}'");
                    if power_mode == BME388SensorPowerMode::Sleep {
                        info!("Getting data after forced mode");
                        break;
                    }
                },
                AcquisitionMode::Normal => {                    
                    let (cmd_dec_rdy, p_data_rdy, t_data_rdy) = match bme388.get_status() {
                        Ok(vals) => (vals.0, vals.1, vals.2),
                        Err(err) => {
                            error!("ERROR - Failed to get BME388 status: {err}");
                            return ExitCode::from(EXIT_CODE_BME388_GET_STATUS_FAILED);
                        }
                    };
                    info!("Got status '{cmd_dec_rdy}', '{p_data_rdy}', '{t_data_rdy}'");
                    if p_data_rdy == BME388StatusPressureData::Ready && t_data_rdy == BME388StatusTemperatureData::Ready {
                        info!("Getting data in normal mode");
                        break;
                    }                    
                }
            }
            let read_status_delay = time::Duration::from_millis(100);
            thread::sleep(read_status_delay);
        }        
        // -- get the raw data
        if let Err(err) = bme388.get_data_raw() {
            error!("ERROR - Failed to get raw data from BME388: {err}");
            return ExitCode::from(EXIT_CODE_BME388_GET_DATA_RAW_FAILED);
        };
        // -- get the compensated data
        let temperature = bme388.get_temperature();
        let pressure = bme388.get_pressure(temperature);
        info!("pressure: {pressure}, temperature: {temperature}");
        // -- delay next reading
        let data_acquisition_delay = time::Duration::from_millis(2000);
        thread::sleep(data_acquisition_delay);
    }
}