use log::{debug, error, info, trace, warn};
use serde::{Deserialize, Serialize};
use serial;
use serial::SerialPort;
use std::{
    fmt,
    io::{ErrorKind, Read, Write},
    path::Path,
    time::Duration,
};

pub const RESPONSE_LENGTH: usize = 10;
pub const COMMAND_LENGTH: usize = 19;
const DATA_LENGTH: usize = 12;

type Response = [u8; RESPONSE_LENGTH];

#[repr(u8)]
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Serial {
    Start = 0xAA,
    End = 0xAB,
    SendByte = 0xB4,
    ResponseByte = 0xC5,
    ReceiveByte = 0xC0,
    CommandTerminator = 0xFF,
}

impl Serial {
    fn try_from(value: u8) -> Result<Self, TryFromIntError> {
        use Serial::*;
        match value {
            value if value == Start as u8 => Ok(Start),
            value if value == End as u8 => Ok(End),
            value if value == SendByte as u8 => Ok(SendByte),
            value if value == ResponseByte as u8 => Ok(ResponseByte),
            value if value == ReceiveByte as u8 => Ok(ReceiveByte),
            value if value == CommandTerminator as u8 => Ok(CommandTerminator),
            _ => Err(TryFromIntError),
        }
    }
}

// Enumeration of SDS011 commands
#[repr(u8)]
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Command {
    /// Get/set reporting mode.  `Initiative` to automatically generate measurements, or `Passive` to use `Request` commands.
    ReportMode = 2,
    /// Query measurement.  When `ReportMode` is `Passive`
    Request = 4,
    /// Get device id
    DeviceId = 5,
    /// Get/set awake or sleeping state
    WorkState = 6,
    /// Get firmware version
    Firmware = 7,
    /// Get/set duty cycle
    DutyCycle = 8,
}

impl Command {
    fn try_from(value: u8) -> Result<Self, TryFromIntError> {
        use Command::*;
        match value {
            value if value == ReportMode as u8 => Ok(ReportMode),
            value if value == Request as u8 => Ok(Request),
            value if value == DeviceId as u8 => Ok(DeviceId),
            value if value == WorkState as u8 => Ok(WorkState),
            value if value == Firmware as u8 => Ok(Firmware),
            value if value == DutyCycle as u8 => Ok(DutyCycle),
            _ => Err(TryFromIntError),
        }
    }
}

// Command to get the current configuration or set it
#[repr(u8)]
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum CommandMode {
    Getting = 0,
    Setting = 1,
}

//Report modes of the sensor:
#[repr(u8)]
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum ReportMode {
    /// In initiative mode, the sensor will automatically report readings just need to periodically read responses
    Initiative = 0,
    /// In passive mode one has to send a `Request` command in order to get measurement values as a response.
    Passive = 1,
}

impl ReportMode {
    fn try_from(value: u8) -> Result<Self, TryFromIntError> {
        use ReportMode::*;
        match value {
            value if value == Initiative as u8 => Ok(Initiative),
            value if value == Passive as u8 => Ok(Passive),
            _ => Err(TryFromIntError),
        }
    }
}

//the Work states:
//In sleeping mode it does not send any data, the fan is turned off.
//To get data one has to wake it up'
#[repr(u8)]
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum WorkState {
    Sleeping = 0,
    Measuring = 1,
}

//The unit of the measured values.
//Two modes are implemented:
//The default mode is MassConcentrationEuropean returning
//values in microgram/cubic meter (mg/m³).
//The other mode is ParticleConcentrationImperial returning values in
//particles / 0.01 cubic foot (pcs/0.01cft).
//The concentration is calculated by assuming
//different mean sphere diameters of pm10 or pm2.5 particles.
#[repr(u8)]
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum UnitsOfMeasure {
    /// µg / m³, the mode of the sensors firmware
    MassConcentrationEuropean = 0,
    /// pcs/0.01 cft (particles / 0.01 cubic foot )
    ParticleConcentrationImperial = 1,
}

#[repr(packed)]
#[derive(Debug)]
pub struct SendData {
    command: Command,
    data: [u8; DATA_LENGTH],
}

impl SendData {
    pub fn get_duty_cycle() -> Self {
        SendData::new_with_mode(Command::DutyCycle, CommandMode::Getting, 0)
    }

    pub fn get_report_mode() -> Self {
        SendData::new_with_mode(Command::ReportMode, CommandMode::Getting, 0)
    }

    pub fn get_firmware() -> Self {
        SendData::new_with_mode(Command::Firmware, CommandMode::Getting, 0)
    }

    pub fn set_duty_cycle(value: u8) -> Self {
        SendData::new_with_mode(Command::DutyCycle, CommandMode::Setting, value)
    }

    pub fn set_report_mode(value: ReportMode) -> Self {
        SendData::new_with_mode(Command::ReportMode, CommandMode::Setting, value as u8)
    }

    pub fn set_work_state(work_state: WorkState) -> Self {
        SendData::new_with_mode(Command::WorkState, CommandMode::Setting, work_state as u8)
    }

    pub fn query() -> Self {
        SendData::new(Command::Request)
    }

    pub fn new_with_mode(command: Command, mode: CommandMode, value: u8) -> Self {
        let mut data = [0; DATA_LENGTH];
        data[0] = mode as u8;
        data[1] = value;
        SendData { command, data }
    }

    pub fn new(command: Command) -> Self {
        let data = [0; DATA_LENGTH];
        SendData { command, data }
    }

    pub fn to_command_data(&self) -> [u8; COMMAND_LENGTH] {
        let mut data = [0; COMMAND_LENGTH];
        data[0] = Serial::Start as u8;
        data[1] = Serial::SendByte as u8;

        data[2] = self.command as u8;
        let end_index = 3 + DATA_LENGTH;
        data[3..end_index].copy_from_slice(&self.data);

        let checksum = 0;
        data[end_index..].copy_from_slice(&[
            Serial::CommandTerminator as u8,
            Serial::CommandTerminator as u8,
            checksum,
            Serial::End as u8,
        ]);
        let checksum_index = COMMAND_LENGTH - 2;
        assert_eq!(data[checksum_index], checksum, "Not replacing the checksum placeholder");
        let checksum = Sensor::generate_checksum(&data[..checksum_index]);
        data[checksum_index] = checksum;

        data
    }
}

pub struct Sensor {
    serial_port: serial::SystemPort,
    device_id: Option<[u8; 2]>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct SensorMeasurement {
    pub pm2_5: f32,
    pub pm10: f32,
}

#[derive(Debug)]
pub struct SensorInfo {
    firmware: [u8; 3],
    report_mode: Option<ReportMode>,
    work_state: WorkState,
    duty_cycle: u8,
}

impl fmt::Display for SensorInfo {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{:02}{:02}{:02}",
            self.firmware[0], self.firmware[1], self.firmware[2]
        )
    }
}

impl Sensor {
    pub fn new(path: &Path) -> Result<Self, serial::Error> {
        let serial_port = serial::open(path)?;
        let sensor = Sensor {
            serial_port,
            device_id: None,
        };
        Ok(sensor)
    }

    pub fn configure(&mut self, timeout: Duration) -> serial::Result<()> {
        const PORT_SETTINGS: serial::PortSettings = serial::PortSettings {
            baud_rate: serial::Baud9600,
            char_size: serial::Bits8,
            parity: serial::ParityNone,
            stop_bits: serial::Stop1,
            flow_control: serial::FlowNone,
        };

        self.serial_port.configure(&PORT_SETTINGS)?;
        self.serial_port.set_timeout(timeout)?;
        Ok(())
    }

    /// reads sensor info from device
    pub fn get_sensor_info(&mut self) -> serial::Result<SensorInfo> {
        let get_duty_cycle = SendData::get_duty_cycle();
        let mut response = self.send(&get_duty_cycle)?;
        let duty_cycle = response[1];

        let get_report_mode = SendData::get_report_mode();
        response = self.send(&get_report_mode)?;
        let report_mode = ReportMode::try_from(response[1]);

        let get_firmware = SendData::get_firmware();
        response = self.send(&get_firmware)?;
        let firmware = [response[0], response[1], response[2]];

        let sensor_info = SensorInfo {
            work_state: WorkState::Measuring,
            duty_cycle,
            report_mode: report_mode.ok(),
            firmware,
        };
        Ok(sensor_info)
    }

    pub fn write_bytes(&mut self, bytes: &[u8]) -> std::io::Result<()> {
        self.serial_port.write_all(bytes)
    }

    pub fn send(&mut self, send_data: &SendData) -> Result<Response, serial::Error> {
        let bytes_to_write = send_data.to_command_data();
        self.write_bytes(&bytes_to_write)?;
        self.get_response(Some(send_data.command))
    }

    pub fn request_measurement(&mut self) -> Result<SensorMeasurement, serial::Error> {
        let response = self.send(&SendData::query())?;
        Ok(Sensor::response_to_measurement(response))
    }

    pub fn get_measurement(&mut self) -> Result<SensorMeasurement, serial::Error> {
        let response = self.get_response(None)?;
        assert!(!response.is_empty());
        Ok(Sensor::response_to_measurement(response))
    }

    fn response_to_measurement(data: Response) -> SensorMeasurement {
        let pm2_5 = (f32::from(data[2]) + f32::from(data[3]) * 256.0) / 10.0;
        let pm10 = (f32::from(data[4]) + f32::from(data[5]) * 256.0) / 10.0;
        SensorMeasurement { pm2_5, pm10 }
    }

    fn validate(data: &[u8]) -> bool {
        let data_length = data.len();
        if data_length != RESPONSE_LENGTH - 2 && data_length != COMMAND_LENGTH - 2 {
            // invalid checksum length
            warn!("checksum error: invalid data length {:?}", data.len());
            return false;
        }

        // check first byte
        if Serial::try_from(data[0]) != Ok(Serial::Start) {
            warn!("checksum error: missing start byte");
            return false;
        }
        // check second byte
        let second_byte = data[1];
        let second_value = Serial::try_from(second_byte);
        match second_value {
            Ok(Serial::SendByte) | Ok(Serial::ReceiveByte) | Ok(Serial::ResponseByte) => {},
            _ => {
                warn!("checksum error: second byte is invalid");
                return false;
            }
        }
        let third_byte = data[2];
        if second_value != Ok(Serial::ReceiveByte) && Command::try_from(third_byte).is_err() {
            warn!("checksum error: data command byte is invalid");
            return false;
        }
        true
    }

    pub fn generate_checksum(data: &[u8]) -> u8 {
        Sensor::validate(data);
        let mut checksum: u8 = 0;
        for data in &data[2..] {
            checksum = checksum.wrapping_add(*data);
        }
        checksum
    }

    pub fn get_response(&mut self, command: Option<Command>) -> Result<Response, serial::Error> {
        let mut bytes_received = Response::default();
        let mut num_bytes_received;
        trace!("get_response({:?})", command);
        loop {
            num_bytes_received = match self.serial_port.read(&mut bytes_received[..1]) {
                Ok(num_bytes) => num_bytes,
                Err(err) => {
                    if err.kind() == ErrorKind::TimedOut {
                        trace!("Read timed out");
                        continue;
                    } else {
                        return Err(err.into());
                    }
                }
            };

            //            '''If no bytes are read the sensor might be in sleep mode.
            //                It makes no sense to raise an exception here. The raise condition
            //            should be checked in a context outside of this fuction.'''
            if num_bytes_received == 0 {
                return Err(serial::Error::new(serial::ErrorKind::NoDevice, "Timeout waiting for sensor response. If the sensor is sleeping, wake it up.  If in dutycycle, this is expected."));
            }

            trace!("byte0 = {:X}", bytes_received[0]);
            //                # if this is true, serial data is coming in
            match Serial::try_from(bytes_received[0]) {
                Ok(Serial::Start) => {
                    trace!("found start byte!");
                }
                other => {
                    warn!("Unexpected: {:?}", other);
                    continue;
                }
            }
            
            num_bytes_received += match self.serial_port.read(&mut bytes_received[1..2]) {
                Ok(read) => read,
                Err(err) => {
                    if err.kind() == ErrorKind::TimedOut {
                        trace!("Read timed out");
                        continue;
                    } else {
                        return Err(err.into());
                    }
                }
            };
            trace!("byte1 {:X}", bytes_received[1]);
            let serial_read = Serial::try_from(bytes_received[1]);
            trace!("serial command: {:?}", serial_read);

            let is_response = (command != None && command != Some(Command::Request)) && serial_read == Ok(Serial::ResponseByte);
            let is_receive = (command == None || command == Some(Command::Request)) && serial_read == Ok(Serial::ReceiveByte);
            if is_response || is_receive {
                while num_bytes_received < bytes_received.len() {
                    let num_bytes = self.serial_port.read(&mut bytes_received[num_bytes_received..])?;
                    trace!("read {}", num_bytes);
                    num_bytes_received += num_bytes;
                }
                

                #[cfg(feature = "validation")]
                {
                    // check if command matches response
                    if is_response && Command::try_from(bytes_received[2]).ok() != command {
                        return Err(serial::Error::new(serial::ErrorKind::InvalidInput, "Third byte of serial datareceived is not the expected response to the previous command"));
                    }
                }
                break;
            }
        }

        #[cfg(feature = "validation")]
        {
            let device_id_index = num_bytes_received - 4;
            let checksum_index = num_bytes_received - 2;
            let checksum_byte = bytes_received[checksum_index];
            let generated_checksum = Sensor::generate_checksum(&bytes_received[..checksum_index]);
            if generated_checksum != checksum_byte {
                let msg = format!(
                    "Invalid checksum! {:?} != {:?}",
                    generated_checksum, checksum_byte
                );
                return Err(serial::Error::new(serial::ErrorKind::InvalidInput, msg));
            } else {
                trace!(
                    "Checksum match: {:?} == {:?}",
                    generated_checksum,
                    checksum_byte
                );
            }

            // set device_id if needed
            let device_id = &bytes_received[device_id_index..checksum_index];
            trace!("device_id {:X}{:X}", device_id[0], device_id[1]);
            if self.device_id == None {
                let mut device_id_array = [0; 2];
                device_id_array.copy_from_slice(device_id);
                self.device_id = Some(device_id_array);
            } else if let Some(existing_device_id) = self.device_id {
                if device_id != existing_device_id {
                    warn!("SDS011 Data received  does not belong to this device with id.");
                }
            }
        }

        Ok(bytes_received)
    }
}

// TODO: 1.33/1.34 replace this with TryFrom once stabilized:
// https://doc.rust-lang.org/std/num/struct.TryFromIntError.html
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TryFromIntError;

impl std::fmt::Display for TryFromIntError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "TryFromIntError")
    }
}

impl std::error::Error for TryFromIntError {}
