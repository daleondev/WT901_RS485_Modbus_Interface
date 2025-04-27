use std::time::Duration;
use serialport::{Parity, StopBits};

use crate::imu_interface::ImuDeviceData;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum SerialError {
    None,
    OpenFailed,
    WriteFailed,
    ReadFailed
}

pub fn open_serial(device: &ImuDeviceData) -> Result<Box<dyn serialport::SerialPort>, SerialError> {
    let port = serialport::new(device.name.clone().as_str(), device.baudrate)
        .timeout(std::time::Duration::from_millis(10))
        .open()
        .map_err(|_| SerialError::OpenFailed)?;
    Ok(port)
}

pub fn write_serial(port: &mut Box<dyn serialport::SerialPort>, buff: &[u8]) -> Result<(), SerialError> {
    port.write_all(buff).map_err(|_| SerialError::WriteFailed)?;
    port.flush().map_err(|_| SerialError::WriteFailed)?;
    Ok(())
}

pub fn write_serial_byte(port: &mut Box<dyn serialport::SerialPort>, byte: u8) -> Result<(), SerialError> {
    port.write(&[byte]).map_err(|_| SerialError::WriteFailed)?;
    port.flush().map_err(|_| SerialError::WriteFailed)?;
    Ok(())
}

pub fn clear_serial(port: &mut Box<dyn serialport::SerialPort>) -> Result<(), SerialError> {
    port.set_timeout(Duration::from_micros(400)).map_err(|_| SerialError::ReadFailed)?;
    let mut buff: [u8;255] = [0;255];
    port.read(&mut buff).map_err(|_| SerialError::ReadFailed)?;
    Ok(())
}

pub fn read_serial(port: &mut Box<dyn serialport::SerialPort>, buff: &mut [u8]) -> Result<usize, SerialError> {
    let transmission_time_ms = calc_transmission_time_ms(port, 1);
    port.set_timeout(Duration::from_millis(transmission_time_ms+10)).map_err(|_| SerialError::ReadFailed)?;
    let bytes_read = port.read(buff).map_err(|_| SerialError::ReadFailed)?;
    Ok(bytes_read)
}

pub fn read_serial_exact(port: &mut Box<dyn serialport::SerialPort>, buff: &mut [u8]) -> Result<(), SerialError> {
    let transmission_time_ms = calc_transmission_time_ms(port, buff.len());
    port.set_timeout(Duration::from_millis(transmission_time_ms+10)).map_err(|_| SerialError::ReadFailed)?;
    port.read_exact(buff).map_err(|_| SerialError::ReadFailed)?;
    Ok(())
}

fn calc_bits_per_byte(port: &mut Box<dyn serialport::SerialPort>) -> usize {
    let mut bits_per_byte = 8;
    bits_per_byte += 1;
    bits_per_byte += match port.parity() {
        Ok(Parity::None) => 0,
        Ok(_) | Err(_)  => 1,
    };
    bits_per_byte += match port.stop_bits() {
        Ok(StopBits::One) => 1,
        Ok(StopBits::Two) | Err(_) => 2,
    };
    bits_per_byte
}

fn calc_transmission_time_ms(port: &mut Box<dyn serialport::SerialPort>, count: usize) -> u64 {
    let bits_per_byte = calc_bits_per_byte(port);
    let baud_rate = port.baud_rate().unwrap_or(9600);
    (((count * bits_per_byte) as f64 / baud_rate as f64) * 1000.0).ceil() as u64
}