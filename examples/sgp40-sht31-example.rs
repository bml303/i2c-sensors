use chrono::Local;
use clap::{Parser, ValueEnum};
use log::{error, info};
use std::path::Path;
use std::process::ExitCode;
use std::{thread, time};

use i2c_sensors::sht31::{
    SHT31, SHT31DeviceAddress, 
    SHT31SingleShotAcquisition, 
    SHT31SingleShotAcquisitionNoClockStretch, 
    SHT31ContinuousAcquisition,
};
use i2c_sensors::sgp40::SGP40;

const EXIT_CODE_SET_CTR_C_HNDLR_FAILED: u8 = 0x02;
const EXIT_CODE_UNSUPPORTED_MODE: u8 = 0x03;
const EXIT_CODE_SHT31_INIT_FAILED: u8 = 0x51;
const EXIT_CODE_SHT31_SINGLE_SHOT_DATA_ACQUISITION_FAILED: u8 = 0x52;
const EXIT_CODE_SHT31_START_CONTINUOUS_MODE_FAILED: u8 = 0x53;

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum)]
enum AcquisitionMode {
    Single,
    SingleNoClockStretch,
    Continuous,
}

#[derive(Parser)]
struct Args {
    // -- i2c bus device
    bus_path: String,
    #[clap(value_enum)]
    mode: AcquisitionMode,
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

    info!("Initializing SGP40");
    let bus_path = Path::new(&bus_path);    
    let mut sgp40 = match SGP40::new(bus_path) {
        Ok(sgp40) => sgp40,
        Err(err) => {
            error!("ERROR - Failed to initialize SGP40: {err}");
            return ExitCode::from(EXIT_CODE_SHT31_INIT_FAILED);
        }
    };

    info!("Initializing SHT31");
    let bus_path = Path::new(&bus_path);
    let dev_addr = SHT31DeviceAddress::Default;
    let mut sht31 = match SHT31::new(bus_path, dev_addr) {
        Ok(sht31) => sht31,
        Err(err) => {
            error!("ERROR - Failed to initialize SHT31: {err}");
            return ExitCode::from(EXIT_CODE_SHT31_INIT_FAILED);
        }
    };

    if args.mode == AcquisitionMode::Continuous {
        info!("Starting continuous mode");
        match sht31.start_continuous_mode(SHT31ContinuousAcquisition::RepeatabilityMedium2Mps) {
            Ok(vals) => vals,
            Err(err) => {
                error!("ERROR - Failed to start SHT45 continuous mode: {err}");
                return ExitCode::from(EXIT_CODE_SHT31_START_CONTINUOUS_MODE_FAILED);
            }
        }
        // -- wait for data acquisiton
        let data_acquisition_delay = time::Duration::from_millis(500);
        thread::sleep(data_acquisition_delay);
    }

    loop {
        let (temperature_raw, humidity_raw) = if args.mode == AcquisitionMode::SingleNoClockStretch {
            // -- no clock stretch
            info!("Getting single data without clock stretch");
            let acquisition_mode = SHT31SingleShotAcquisitionNoClockStretch::RepeatabilityMedium;
            match sht31.get_data_single_no_clock_stretch(acquisition_mode) {
                Ok(vals) => vals,
                Err(err) => {
                    error!("ERROR - Failed to acquire single shot data from SHT31: {err}");
                    return ExitCode::from(EXIT_CODE_SHT31_SINGLE_SHOT_DATA_ACQUISITION_FAILED);
                }
            }
        } else if args.mode == AcquisitionMode::Single {
            // -- with clock stretch
            info!("Getting single data with clock stretch");
            let acquisition_mode = SHT31SingleShotAcquisition::RepeatabilityMedium;
            match sht31.get_data_single(acquisition_mode) {
                Ok(vals) => vals,
                Err(err) => {
                    error!("ERROR - Failed to acquire single shot data from SHT31: {err}");
                    return ExitCode::from(EXIT_CODE_SHT31_SINGLE_SHOT_DATA_ACQUISITION_FAILED);
                }
            }
        } else if args.mode == AcquisitionMode::Continuous {
            // -- continuous data, no clock stretch possible
            info!("Getting continuous data");
            match sht31.get_data_continuous() {
                Ok(vals) => vals,
                Err(err) => {
                    error!("ERROR - Failed to acquire continuous data from SHT31: {err}");
                    return ExitCode::from(EXIT_CODE_SHT31_SINGLE_SHOT_DATA_ACQUISITION_FAILED);
                }
            }
        } else {
            return ExitCode::from(EXIT_CODE_UNSUPPORTED_MODE);
        };

        let temperature = sht31.get_temperature_celcius(temperature_raw);
        let humidity = sht31.get_humidity(humidity_raw);
        info!("temperature: {temperature}, humidity: {humidity}");

        if let Ok(voc_raw) = sgp40.get_voc_data_with_compensation(humidity_raw, temperature_raw) {
            let voc_index = sgp40.process_voc(voc_raw);
            info!("voc_raw: {voc_raw}, voc_index: {voc_index}");
        };
                
        let data_acquisition_delay = time::Duration::from_millis(1000);
        thread::sleep(data_acquisition_delay);
    }
}