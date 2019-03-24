use sds011::*;

#[test]
fn checksum() {
    let cmd = SendData::set_work_state(WorkState::Measuring).to_command_data();
    let checksum = Sensor::generate_checksum(&cmd[..cmd.len() - 2]);
    assert_eq!(checksum, 6);
    let bytes = [
        Serial::Start as u8,
        Serial::SendByte as u8,
        Command::WorkState as u8,
        CommandMode::Setting as u8,
        WorkState::Measuring as u8,
        0u8,
        0u8,
        0u8,
        0u8,
        0u8,
        0u8,
        0u8,
        0u8,
        0u8,
        0u8,
        Serial::CommandTerminator as u8,
        Serial::CommandTerminator as u8,
        checksum,
        Serial::End as u8,
    ];
    assert_eq!(cmd, bytes);
}
