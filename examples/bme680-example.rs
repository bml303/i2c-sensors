use chrono::Local;
use clap::Parser;
use log::{error, info};
use std::collections::VecDeque;
use std::path::Path;
use std::process::ExitCode;
use std::{thread, time};

use i2c_sensors::bme680::*;

const EXIT_CODE_SET_CTR_C_HNDLR_FAILED: u8 = 0x02;
const EXIT_CODE_BME680_INIT_FAILED: u8 = 0x71;
const EXIT_CODE_BME680_ENABLE_RUN_GAS_FAILED: u8 = 0x72;
const EXIT_CODE_BME680_SET_HEATER_PROFILE_FAILED: u8 = 0x73;
const EXIT_CODE_BME680_SET_GAS_WAIT_FAILED: u8 = 0x74;
const EXIT_CODE_BME680_SET_RES_HEAT_FAILED: u8 = 0x75;
const EXIT_CODE_BME680_SET_FORCED_MODE_FAILED: u8 = 0x76;
const EXIT_CODE_BME680_GET_MEASURING_STATUS_FAILED: u8 = 0x77;
const EXIT_CODE_BME680_GET_MEASURING_RESULT_FAILED: u8 = 0x78;
const EXIT_CODE_BME680_GET_GAS_MEASURING_RESULT_FAILED: u8 = 0x79;

#[derive(Parser)]
struct Args {
    // -- i2c bus device
    bus_path: String,
}

// -- curtesy of https://github.com/thstielow/raspi-bme680-iaq/blob/main/bme680IAQ.py
// -- calculates the saturation water density of air at the current temperature (in °C)
// -- return the saturation density rho_max in kg/m^3
// -- this is equal to a relative humidity of 100% at the current temperature 
fn water_sat_density(temperature: f64) -> f64 {
    let exp = (17.62 * temperature)/(243.12 + temperature).exp();
    let rho_max = (6.112 * 100.0 * exp)/(461.52 * (temperature + 273.15));
    rho_max
}

fn calc_mean(data: &VecDeque<f64>) -> f64 {
    let data_iter = data.iter();
    let mut sum = 0.0;
    for val in data_iter {
        sum += val;
    }
    sum / data.len() as f64
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

    info!("Initializing BME680");
    let bus_path = Path::new(&bus_path);
    let dev_addr = Bme680DeviceAddress::Default;
    let humidity_osr = Bme680OverSampling::Oversampling1x;
    let pressure_osr = Bme680OverSampling::Oversampling16x;
    let temperature_osr = Bme680OverSampling::Oversampling2x;
    let irr_filter = Bme680IrrFilter::Coef3;
    let mut bme680 = match BME680::new(bus_path, dev_addr, humidity_osr, pressure_osr, temperature_osr, irr_filter) {
        Ok(bme680) => bme680,
        Err(err) => {
            error!("ERROR - Failed to initialize BME680: {err}");
            return ExitCode::from(EXIT_CODE_BME680_INIT_FAILED);
        }
    };
    let chip_id = bme680.get_chip_id();
    info!("Got chip id {chip_id:#04x}");    
    
    // -- assume ambient temperature for first run, then use measured value
    const BURN_IN_CYCLES: u64 = 360;
    const BURN_IN_DELAY_SEC: u64 = 1;
    const MEASURING_DELAY_SEC: u64 = 5;
    const PH_SLOPE: f64 = 0.01;
    let mut ambient_temp = 20.0;
    let mut burn_in_cycles = BURN_IN_CYCLES;
    let mut gas_cal_data: VecDeque<f64> = VecDeque::new();
    let mut gas_ceil = 0.0;
    const GAS_RECAL_PERIOD: usize = 360; // / MEASURING_DELAY_SEC as usize;
    let mut gas_recal_step: usize = 0;

    loop {

        if let Err(err) = bme680.set_gas_wait_0(40, Bme680GasWaitMultiplicationFactor::X4) {
            error!("ERROR - BME680 failed to set gas wait: {err}");
            return ExitCode::from(EXIT_CODE_BME680_SET_GAS_WAIT_FAILED);
        }
    
        const TARGET_TEMP: f64 = 320.0;
        let res_heat = bme680.calc_res_heat(ambient_temp, TARGET_TEMP);
    
        if let Err(err) = bme680.set_res_heat_0(res_heat) {
            error!("ERROR - BME680 failed to set res heat: {err}");
            return ExitCode::from(EXIT_CODE_BME680_SET_RES_HEAT_FAILED);
        }    
    
        if let Err(err) = bme680.set_heater_profile(Bme680HeaterProfile::SetPoint0) {
            error!("ERROR - BME680 failed to set heater profile: {err}");
            return ExitCode::from(EXIT_CODE_BME680_SET_HEATER_PROFILE_FAILED);
        }
    
        if let Err(err) = bme680.enable_run_gas() {
            error!("ERROR - BME680 failed to enable run gas: {err}");
            return ExitCode::from(EXIT_CODE_BME680_ENABLE_RUN_GAS_FAILED);
        }

        if let Err(err) = bme680.set_forced_mode() {
            error!("ERROR - BME680 failed to set forced mode: {err}");
            return ExitCode::from(EXIT_CODE_BME680_SET_FORCED_MODE_FAILED);
        }

        loop {
            // -- read status
            let status = match bme680.get_meas_status() {
                Ok(status) => status,
                Err(err) => {
                    error!("ERROR - BME680 failed get measuring status: {err}");
                    return ExitCode::from(EXIT_CODE_BME680_GET_MEASURING_STATUS_FAILED);
                }
            };
            // -- check if new data is available
            info!("Got measuring status {status:#?}");
            if status.new_data {
                break;
            }
            // -- short delay
            let delay = time::Duration::from_millis(100);
            thread::sleep(delay);
        }
    
        // -- read result
        let result = match bme680.get_meas_result() {
            Ok(result) => result,
            Err(err) => {
                error!("ERROR - BME680 failed get measuring result: {err}");
                return ExitCode::from(EXIT_CODE_BME680_GET_MEASURING_RESULT_FAILED);
            }
        };
        info!("Got measuring result {result:#?}");
        
        // -- get compensated values
        let (temperature, t_fine) = bme680.get_temperature(result.temperature_raw);
        info!("Got compensated temperature {temperature} with t_fine {t_fine}");
        let pressure = bme680.get_pressure(result.pressure_raw, t_fine);
        info!("Got compensated pressure {pressure}");
        let humidity = bme680.get_humidity(result.humidity_raw, temperature);
        info!("Got compensated humidity {humidity}");
        // -- get gas resistance
        let gas_result = match bme680.get_gas_meas_result() {
            Ok(result) => result,
            Err(err) => {
                error!("ERROR - BME680 failed get gas measuring result: {err}");
                return ExitCode::from(EXIT_CODE_BME680_GET_GAS_MEASURING_RESULT_FAILED);
            } 
        };
        info!("Got gas resistance {gas_result:#?}");

        // -- store ambient temperature for next loop
        ambient_temp = temperature;
        if !(gas_result.gas_valid && gas_result.heat_stab) {
            // -- skip measurement if not valid or heater not stable (or both)
            let measuring_delay = time::Duration::from_millis(MEASURING_DELAY_SEC * 1000);
            thread::sleep(measuring_delay);
            continue
        }

        // -- get saturation water density of air
        let rho_max = water_sat_density(temperature);
        // -- absolute humidity
        let hum_abs = humidity * 10.0 * rho_max;
        // -- compensate exponential impact of humidity on resistance
		let comp_gas = gas_result.gas_res * (PH_SLOPE * hum_abs).exp();
        // -- burn-in or not 
        if burn_in_cycles > 0 {
            burn_in_cycles -= 1;
            if comp_gas > gas_ceil {
                //gas_cal_data.clear();   
                gas_cal_data.push_back(comp_gas);
                while gas_cal_data.len() > 5 {
                    gas_cal_data.pop_front();
                }
                gas_ceil = comp_gas;
            }
            info!("Burn-in cycle {burn_in_cycles}: gas_ceil {gas_ceil}");
            // -- delay next measuring
            let measuring_delay = time::Duration::from_millis(BURN_IN_DELAY_SEC * 1000);
            thread::sleep(measuring_delay);
            continue
        } else {
            // -- adapt calibration
			if comp_gas > gas_ceil {
				gas_cal_data.push_back(comp_gas);
				while gas_cal_data.len() > 50 {
                    gas_cal_data.pop_front();
                }
                gas_ceil = calc_mean(&gas_cal_data);
            }

            // -- calculate and print relative air quality on a scale of 0-100%
            let aqr = comp_gas / gas_ceil;
            // -- clip air quality at 100%
            let aq = aqr.min(1.0) * 100.0;
            // -- use quadratic ratio for steeper scaling at high air quality
			//let aq = (aqr.powi(2)).min(1.0) * 100.0;
            let aqi = (1.0 - (aq / 100.0)) * 500.0;
            info!("AQR {aqr}, Air Quality Index {aqi}, AQ {aq}%, comp_gas {comp_gas}");

            // -- for compensating negative drift (dropping resistance) of the gas sensor:
			// -- delete oldest value from calibration list and add current value
			gas_recal_step += 1;
			if gas_recal_step >= GAS_RECAL_PERIOD {
                gas_recal_step = 0;
				gas_cal_data.push_back(comp_gas);
                while gas_cal_data.len() > 50 {
                    gas_cal_data.pop_front();
                }
                gas_ceil = calc_mean(&gas_cal_data);
            }
        }

        // -- delay next measuring
        let measuring_delay = time::Duration::from_millis(MEASURING_DELAY_SEC * 1000);
        thread::sleep(measuring_delay);
    }    
}