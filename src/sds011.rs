use clap::{App, Arg, ArgGroup};
use env_logger::{Builder, Env};
use log::{debug, error, info, trace, warn};
use sds011::*;
use std::{
    path::Path,
    time::Duration,
};

const DEVICE_PATH: &str = "DEVICE_PATH";

// Attempts to find the sensor device
fn default_device() -> String {
    let files = std::fs::read_dir("/dev");
    if let Ok(files) = files {
        for file in files {
            if let Ok(file) = file {
                if let Some(file_name) = file.path().file_name() {
                    // macOS: `/dev/tty.wchusbserial141240`
                    // linux: `/dev/ttyUSB0`
                    let file_name = file_name.to_string_lossy();
                    if file_name.starts_with("tty.wchusbserial") || file_name.starts_with("ttyUSB") {
                        return file.path().to_string_lossy().into_owned();
                    }
                }
            }
        }
    }
    String::from("/dev/ttyUSB0")
}

fn main() {
    const ARG_INITIATIVE: &str = "initiative";
    const ARG_PASSIVE: &str = "passive";

    Builder::from_env(Env::default().default_filter_or("trace"))
        .try_init()
        .unwrap_or_else(|err| eprintln!("env_logger::init() failed: {}", err));
    let device_env = std::env::var(DEVICE_PATH);
    let matches = App::new("sds011")
        .version("1.0")
        .author("Chris Ballinger <chris@ballinger.io>")
        .about("SDS011 particle sensor")
        .arg(
            Arg::with_name(DEVICE_PATH)
                .help("Path to device e.g. /dev/ttyUSB0")
                .index(1),
        )
        .arg(
            Arg::with_name(ARG_INITIATIVE)
                .long(ARG_INITIATIVE)
        )
        .arg(
            Arg::with_name(ARG_PASSIVE)
                .help("Read a single value and exit")
                .long(ARG_PASSIVE)
        )
        .group(
            ArgGroup::with_name("report_mode")
                .args(&[ARG_INITIATIVE, ARG_PASSIVE])
                .multiple(false)
                .required(false),
        )
        .get_matches();

    let path_string: String = match device_env {
        Ok(path) => path,
        Err(_) => match matches.value_of(DEVICE_PATH) {
            Some(path) => String::from(path),
            None => default_device(),
        },
    };
    let report_mode = if matches.is_present(ARG_INITIATIVE) { ReportMode::Initiative } else { ReportMode::Passive };

    let path = Path::new(path_string.as_str());
    let mut sensor = Sensor::new(path).unwrap();
    info!("Opened device at path: {}", path_string);

    sensor.configure(Duration::from_secs(1)).unwrap();
    info!("Configured device");

    // Wake the sensor in case it's sleepinng
    let wake_command = SendData::set_work_state(WorkState::Measuring);
    sensor.send(&wake_command).unwrap();
    
    let sensor_info = sensor.get_sensor_info().unwrap();
    info!("Sensor info: {}", sensor_info);

    match report_mode {
        ReportMode::Initiative => {
            sensor
                .send(&SendData::set_report_mode(ReportMode::Initiative))
                .unwrap();
            loop {
                let measurement = sensor.get_measurement().unwrap();
                info!("Read {:?}", measurement);
            }
        }
        ReportMode::Passive => {
            sensor
                .send(&SendData::set_report_mode(ReportMode::Passive))
                .unwrap();

            let measurement = sensor.request_measurement().unwrap();
            info!("Read {:?}", measurement);
        }
    }
}
