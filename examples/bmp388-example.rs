use chrono::Local;
use clap::{Parser, ValueEnum};
use log::{error, info};
use std::path::Path;
use std::process::ExitCode;
use std::{thread, time};

use i2c_sensors::bmp388::*;

const EXIT_CODE_SET_CTR_C_HNDLR_FAILED: u8 = 0x02;
const EXIT_CODE_BMP388_INIT_FAILED: u8 = 0x61;
const EXIT_CODE_BMP388_SET_NORMAL_POWER_MODE_FAILED: u8 = 0x62;
const EXIT_CODE_BMP388_SET_FORCED_POWER_MODE_FAILED: u8 = 0x63;
const EXIT_CODE_BMP388_GET_STATUS_FAILED: u8 = 0x64;
const EXIT_CODE_BMP388_GET_DATA_RAW_FAILED: u8 = 0x65;
const EXIT_CODE_BMP388_GET_SENSOR_MODE_FAILED: u8 = 0x66;
const EXIT_CODE_BMP388_ENABLE_FIFO_FAILED: u8 = 0x67;
const EXIT_CODE_BMP388_GET_FIFO_LENGTH_FAILED: u8 = 0x68;
const EXIT_CODE_BMP388_GET_FIFO_FRAME_FAILED: u8 = 0x69;
const EXIT_CODE_BMP388_GET_DATA_READY_FAILED: u8 = 0x6a;

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum)]
enum AcquisitionMode {
    Forced,
    Normal,
    Fifo,
}

#[derive(Parser)]
struct Args {
    // -- i2c bus device
    bus_path: String,
    #[clap(value_enum)]
    mode: AcquisitionMode,
}

fn get_sensor_settings(mode: &AcquisitionMode) -> (Bmp388OverSamplingPr, Bmp388OverSamplingTp,
    Bmp388IrrFilter, Bmp388OutputDataRate) {
    match mode {
        AcquisitionMode::Forced => (Bmp388OverSamplingPr::UltraLowX1, Bmp388OverSamplingTp::X1,
            Bmp388IrrFilter::Off, Bmp388OutputDataRate::Ix0_78Hz),
        AcquisitionMode::Normal => (Bmp388OverSamplingPr::HighX8, Bmp388OverSamplingTp::X1,
            Bmp388IrrFilter::Coef1, Bmp388OutputDataRate::Ix0_78Hz),
        AcquisitionMode::Fifo => (Bmp388OverSamplingPr::HighX8, Bmp388OverSamplingTp::X1,
            Bmp388IrrFilter::Coef1, Bmp388OutputDataRate::Ix0_78Hz),
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

    info!("Initializing BMP388");
    let bus_path = Path::new(&bus_path);
    let dev_addr = Bmp388DeviceAddress::Default;
    let (osr_p, osr_t, irr_filter, odr) = get_sensor_settings(&args.mode);
    let mut bmp388 = match BMP388::new(bus_path, dev_addr, osr_p, osr_t, irr_filter, odr) {
        Ok(bmp388) => bmp388,
        Err(err) => {
            error!("ERROR - Failed to initialize BMP388: {err}");
            return ExitCode::from(EXIT_CODE_BMP388_INIT_FAILED);
        }
    };
    // -- start reading data
    let enable_pressure = Bmp388StatusPressureSensor::Enabled;
    let enable_temperature = Bmp388StatusTemperatureSensor::Enabled;
    if args.mode == AcquisitionMode::Normal || args.mode == AcquisitionMode::Fifo {
        info!("Setting normal mode");
        if let Err(err) = bmp388.set_sensor_mode(Bmp388SensorPowerMode::Normal, enable_pressure, enable_temperature) {
            error!("ERROR - Failed to set BMP388 to normal power mode: {err}");
            return ExitCode::from(EXIT_CODE_BMP388_SET_NORMAL_POWER_MODE_FAILED);
        }
        if args.mode == AcquisitionMode::Fifo {
            // -- enable fifo
            let stop_on_full = Bmp388FifoStopOnFull::Enabled;
            let with_pressure = Bmp388FifoWithPressureData::Enabled;
            let with_temperature = Bmp388FifoWithTemperatureData::Enabled;
            let with_sensor_time = Bmp388FifoWithSensorTime::Enabled;
            let data_filtered = Bmp388FifoDataFiltered::Filtered;
            let subsampling = 0;
            if let Err(err) = bmp388.enable_fifo(stop_on_full, with_pressure, with_temperature, with_sensor_time, data_filtered, subsampling) {
                error!("ERROR - Failed to enable BMP388 FIFO: {err}");
                return ExitCode::from(EXIT_CODE_BMP388_ENABLE_FIFO_FAILED);
            }
        }
    }
    let mut temperature_last = 20.0;
    loop {
        if args.mode == AcquisitionMode::Forced {
            info!("Setting forced mode");
            if let Err(err) = bmp388.set_sensor_mode(Bmp388SensorPowerMode::Forced, Bmp388StatusPressureSensor::Enabled, Bmp388StatusTemperatureSensor::Enabled) {
                error!("ERROR - Failed to set BMP388 to forced power mode: {err}");
                return ExitCode::from(EXIT_CODE_BMP388_SET_FORCED_POWER_MODE_FAILED);
            }
        }
        loop {
            match args.mode {
                AcquisitionMode::Forced => {
                    // -- forced mode
                    let (power_mode, p_enabled, t_enabled) = match bmp388.get_sensor_mode() {
                        Ok(vals) => (vals.0, vals.1, vals.2),
                        Err(err) => {
                            error!("ERROR - Failed to get BMP388 sensor mode: {err}");
                            return ExitCode::from(EXIT_CODE_BMP388_GET_SENSOR_MODE_FAILED);
                        }
                    };
                    info!("Got mode '{power_mode}', '{p_enabled}', '{t_enabled}'");
                    if power_mode == Bmp388SensorPowerMode::Sleep {
                        info!("Getting data after forced mode");
                        break;
                    }
                },
                AcquisitionMode::Normal => {
                    let (cmd_dec_rdy, p_data_rdy, t_data_rdy) = match bmp388.get_status() {
                        Ok(vals) => (vals.0, vals.1, vals.2),
                        Err(err) => {
                            error!("ERROR - Failed to get BMP388 status: {err}");
                            return ExitCode::from(EXIT_CODE_BMP388_GET_STATUS_FAILED);
                        }
                    };
                    info!("Got status '{cmd_dec_rdy}', '{p_data_rdy}', '{t_data_rdy}'");
                    if p_data_rdy == Bmp388StatusPressureData::Ready && t_data_rdy == Bmp388StatusTemperatureData::Ready {
                        info!("Getting data in normal mode");
                        break;
                    }
                },
                AcquisitionMode::Fifo => {
                    let is_data_ready = match bmp388.is_data_ready() {
                        Ok(int_status) => int_status,
                        Err(err) => {
                            error!("ERROR - Failed to get BMP388 data ready: {err}");
                            return ExitCode::from(EXIT_CODE_BMP388_GET_DATA_READY_FAILED);
                        }
                    };
                    if is_data_ready {
                        info!("BMP388 INT status data ready");
                        info!("Start reading FIFO frames");
                        break;
                    }
                }
            }
            let read_status_delay = time::Duration::from_millis(100);
            thread::sleep(read_status_delay);
        }
        if args.mode == AcquisitionMode::Fifo {
            loop {
                let fifo_frame = match bmp388.read_next_fifo_data_frame() {
                    Ok(fifo_frame) => fifo_frame,
                    Err(err) => {
                        error!("ERROR - Failed to get BMP388 FIFO data frame: {err}");
                        return ExitCode::from(EXIT_CODE_BMP388_GET_FIFO_FRAME_FAILED);
                    }
                };
                if fifo_frame.config_change {
                    info!("BMP388 FIFO configuration change detected");
                }
                if let Some(sensor_time) = fifo_frame.sensor_time {
                    info!("BMP388 FIFO sensor time: {sensor_time}");
                }
                if let Some(temperature_raw) = fifo_frame.temperature_raw {
                    info!("BMP388 FIFO temperature raw: {temperature_raw}");
                    // -- get the compensated temperature
                    let temperature = bmp388.get_temperature(temperature_raw);
                    temperature_last = temperature;
                    if let Some(pressure_raw) = fifo_frame.pressure_raw {
                        info!("BMP388 FIFO pressure raw: {pressure_raw}");
                        // -- get the compensated pressure
                        let pressure = bmp388.get_pressure(pressure_raw, temperature);
                        info!("pressure: {pressure}, temperature: {temperature}");
                    } else {
                        info!("pressure: <no data>>, temperature: {temperature}");
                    }
                } else if let Some(pressure_raw) = fifo_frame.pressure_raw {
                    info!("BMP388 FIFO pressure raw: {pressure_raw}");
                    // -- get the compensated pressure
                    let pressure = bmp388.get_pressure(pressure_raw, temperature_last);
                    info!("pressure: {pressure}, temperature: <no data>");
                }
                let fifo_length = match bmp388.get_fifo_length() {
                    Ok(fifo_length) => fifo_length,
                    Err(err) => {
                        error!("ERROR - Failed to get BMP388 FIFO length: {err}");
                        return ExitCode::from(EXIT_CODE_BMP388_GET_FIFO_LENGTH_FAILED);
                    }
                };
                info!("BMP388 FIFO length is {fifo_length}");
                if fifo_length == 0 {
                    info!("Stop reading FIFO frames");
                    break;
                }
                let read_status_delay = time::Duration::from_millis(100);
                thread::sleep(read_status_delay);
            }
        } else {
            // -- get the raw data
            let data_raw = match  bmp388.get_data_raw() {
                Ok(data_raw) => data_raw,
                Err(err) => {
                    error!("ERROR - Failed to get raw data from BMP388: {err}");
                    return ExitCode::from(EXIT_CODE_BMP388_GET_DATA_RAW_FAILED);
                },
            };
            // -- get the compensated data
            let (pressure, temperature) = bmp388.get_pressure_and_temperature(&data_raw);
            info!("pressure: {pressure}, temperature: {temperature}");
        }
        // -- delay next reading
        let data_acquisition_delay = time::Duration::from_millis(2000);
        thread::sleep(data_acquisition_delay);
    }
}