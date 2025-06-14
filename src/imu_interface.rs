use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::hardware::{read_reg_cmd, write_reg_cmd, Bandwidth, Register, Save, KEY_UNLOCK};
use crate::imu_interface_ffi::{ImuData, ImuDataCallback, ImuError};
use crate::imu_interface_manager::Uuid;
use crate::serial_interface::{clear_serial, open_serial, read_serial_exact, write_serial, SerialError};

// fn convert_angle(angle: f64) -> f32 {
//     (if angle > 180.0 { angle - 360.0 } else { angle }) as f32
// }

#[derive(Debug, Clone)]
pub struct ImuDeviceData {
    pub name: String,
    pub baudrate: u32,
    pub addr: u8,
}

pub struct ImuInterface {
    pub id: Uuid,
    pub device: ImuDeviceData,
    pub thread: Option<std::thread::JoinHandle<()>>,
    pub port: Arc<Mutex<Option<Box<dyn serialport::SerialPort>>>>,
    pub running: Arc<AtomicBool>,
    pub data_callback: Arc<Mutex<ImuDataCallback>>,
    pub data: Arc<Mutex<ImuData>>,
}

impl ImuInterface {
    pub fn new(id: Uuid, device: ImuDeviceData) -> Self {
        Self {
            id,
            device,
            thread: None,
            port: Arc::new(Mutex::new(None)),           
            running: Arc::new(AtomicBool::new(false)),
            data_callback: Arc::new(Mutex::new(None)),
            data: Arc::new(Mutex::new(ImuData {
                acc: [0.0; 3],
                gyro: [0.0; 3],
                angle: [0.0; 3],
                mag: [0; 3],
                temp: 0.0,
            })),
        }
    }

    pub fn zeroed() -> Self {
        Self {
            id: 0,
            device: ImuDeviceData { 
                name: String::new(), 
                baudrate: 0, 
                addr: 0, 
            },
            thread: None,
            port: Arc::new(Mutex::new(None)),           
            running: Arc::new(AtomicBool::new(false)),
            data_callback: Arc::new(Mutex::new(None)),
            data: Arc::new(Mutex::new(ImuData {
                acc: [0.0; 3],
                gyro: [0.0; 3],
                angle: [0.0; 3],
                mag: [0; 3],
                temp: 0.0,
            })),
        }
    }

    pub fn init(&mut self) -> Result<(), ImuError> {
        let mut port = self.port.lock().map_err(|_| ImuError::ImuErrorUnknown)?;
        port.replace(open_serial(&self.device).map_err(|err| match err { 
            SerialError::OpenFailed => ImuError::ImuErrorOpenFailed,
            _ => ImuError::ImuErrorUnknown,
        })?);
        Ok(())
    }

    pub fn set_data_callback(&mut self, callback: ImuDataCallback) -> Result<(), ImuError> {
        if callback.is_some() {
            self.data_callback.lock()
                .map_err(|_| ImuError::ImuErrorUnknown)?
                .replace(callback.unwrap());
        }
        Ok(())
    }

    pub fn read_registers<const REG_NUM: usize, const BUFF_SIZE: usize>(&mut self, start_reg: Register) -> Result<[i16; REG_NUM], ImuError> {      
        if self.running.load(Ordering::SeqCst) || self.thread.is_some() {
            return Err(ImuError::ImuErrorAlreadyRunning);
        }

        let mut port = self.port.lock().map_err(|_| ImuError::ImuErrorUnknown)?;
        let port = match port.as_mut() {
            Some(port) => port,
            None => return Err(ImuError::ImuErrorUnknown),
        };

        let read_cmd = read_reg_cmd(self.device.addr, start_reg, REG_NUM).map_err(|_| ImuError::ImuErrorReadFailed)?;
        write_serial(port, &read_cmd).map_err(|_| ImuError::ImuErrorReadFailed)?;

        let mut result: [u8; BUFF_SIZE] = [0; BUFF_SIZE];
        read_serial_exact(port, &mut result).map_err(|_| ImuError::ImuErrorReadFailed)?;

        if result[0] != read_cmd[0] || result[1] != read_cmd[1] || result[2] != 2*REG_NUM as u8 {
            return Err(ImuError::ImuErrorReadFailed);
        }

        let mut data: [i16; REG_NUM] = [0; REG_NUM];
        let mut j: usize = 3;
        for i in 0..REG_NUM {
            data[i] = ((result[j] as i16) << 8) | (result[j+1] as i16);
            j += 2;
        }
        Ok(data)
    }

    pub fn read_register(&mut self, reg: Register) -> Result<i16, ImuError> {
        if self.running.load(Ordering::SeqCst) || self.thread.is_some() {
            return Err(ImuError::ImuErrorAlreadyRunning);
        }

        let mut port = self.port.lock().map_err(|_| ImuError::ImuErrorUnknown)?;
        let port = match port.as_mut() {
            Some(port) => port,
            None => return Err(ImuError::ImuErrorUnknown),
        };

        let read_cmd = read_reg_cmd(self.device.addr, reg, 1).map_err(|_| ImuError::ImuErrorReadFailed)?;
        write_serial(port, &read_cmd).map_err(|_| ImuError::ImuErrorReadFailed)?;

        let mut result: [u8; 7] = [0; 7];
        read_serial_exact(port, &mut result).map_err(|_| ImuError::ImuErrorReadFailed)?;

        for i in 0..7 {
            println!("0x{:02X} - 0x{:02X}", read_cmd[i], result[i]);
        }
        println!("0x{:02X} - 0x00", read_cmd[7]);

        if result[0] != read_cmd[0] || result[1] != read_cmd[1] || result[2] != 2 {
            return Err(ImuError::ImuErrorReadFailed);
        }
        Ok(((result[3] as i16) << 8) | (result[4] as i16))
    }

    pub fn write_register(&mut self, reg: Register, data: i16) -> Result<(), ImuError> {
        if self.running.load(Ordering::SeqCst) || self.thread.is_some() {
            return Err(ImuError::ImuErrorAlreadyRunning);
        }

        let mut port = self.port.lock().map_err(|_| ImuError::ImuErrorUnknown)?;
        let port = match port.as_mut() {
            Some(port) => port,
            None => return Err(ImuError::ImuErrorUnknown),
        };

        let write_cmd = write_reg_cmd(self.device.addr, reg, data).map_err(|_| ImuError::ImuErrorWriteFailed)?;
        write_serial(port, &write_cmd).map_err(|_| ImuError::ImuErrorWriteFailed)?;

        let mut handshake: [u8; 8] = [0; 8];
        read_serial_exact(port, &mut handshake).map_err(|_| ImuError::ImuErrorReadFailed)?;

        for i in 0..8 {
            println!("0x{:02X} - 0x{:02X}", write_cmd[i], handshake[i]);
        }

        if write_cmd != handshake {
            return Err(ImuError::ImuErrorWriteFailed);
        }
        Ok(())
    }

    pub fn start(&mut self) -> Result<(), ImuError> {
        if self.running.load(Ordering::SeqCst) {
            return Err(ImuError::ImuErrorAlreadyRunning);
        }
        
        let port = Arc::clone(&self.port);
        let running = Arc::clone(&self.running);
        let data = Arc::clone(&self.data);
        let data_callback = Arc::clone(&self.data_callback);
        let interface_id = self.id.clone();
        let addr = self.device.addr.clone();

        self.running.store(true, Ordering::SeqCst);
        self.thread = Some(std::thread::spawn(move || {
            const REG_NUM: usize = (Register::TEMP as usize + 1) - Register::AX as usize;
            const BUFF_SIZE: usize = 2*REG_NUM+5;
            const MAX_ERRORS: u32 = 10;

            let mut buff: [u8; BUFF_SIZE] = [0; BUFF_SIZE];
            let mut error_count: u32 = 0;

            macro_rules! handle_err {
                () => {
                    {
                        error_count += 1;
                        if error_count >= MAX_ERRORS {
                            eprintln!("Too many errors, stopping thread.");
                            running.store(false, Ordering::SeqCst);
                            break;
                        }
                        continue;
                    }
                };
            }

            while running.load(Ordering::SeqCst) {
                let result = port.lock();
                if result.is_err() {
                    handle_err!();
                }
                let mut port = result.unwrap();

                let result = port.as_mut();
                if result.is_none() {
                    handle_err!();
                }
                let port = result.unwrap();

                if clear_serial(port).is_err() {
                    handle_err!();
                }

                match read_reg_cmd(addr, Register::AX, REG_NUM) {
                    Ok(read_cmd) => {
                        if write_serial(port, &read_cmd).is_err() {
                            handle_err!();
                        } 

                        if read_serial_exact(port, &mut buff).is_err() {
                            handle_err!();
                        }

                        if buff[0] != read_cmd[0] || buff[1] != read_cmd[1] || buff[2] != 2*REG_NUM as u8 {
                            handle_err!();
                        }
                    },
                    Err(_) => {
                        handle_err!();
                    }
                }

                let mut raw_data: [i16; REG_NUM] = [0; REG_NUM];
                let mut j: usize = 3;
                for i in 0..REG_NUM {
                    raw_data[i] = ((buff[j] as i16) << 8) | (buff[j+1] as i16);
                    j += 2;
                }

                match data.lock() {
                    Ok(mut data) => {
                        data.acc[0] = (raw_data[0] as f64 / 32768.0 * 16.0) as f32;
                        data.acc[1] = (raw_data[1] as f64 / 32768.0 * 16.0) as f32;
                        data.acc[2] = (raw_data[2] as f64 / 32768.0 * 16.0) as f32;
                        data.gyro[0] = (raw_data[3] as f64 / 32768.0 * 2000.0) as f32;
                        data.gyro[1] = (raw_data[4] as f64 / 32768.0 * 2000.0) as f32;
                        data.gyro[2] = (raw_data[5] as f64 / 32768.0 * 2000.0) as f32;
                        data.mag[0] = raw_data[6] as i32;
                        data.mag[1] = raw_data[7] as i32;
                        data.mag[2] = raw_data[8] as i32;
                        data.angle[0] = (raw_data[9] as f64 / 32768.0 * 180.0) as f32; 
                        data.angle[1] = (raw_data[10] as f64 / 32768.0 * 180.0) as f32; 
                        data.angle[2] = (raw_data[11] as f64 / 32768.0 * 180.0) as f32;

                        match data_callback.lock() {
                            Ok(mut callback) => {
                                if let Some(callback) = callback.as_mut() {
                                    callback(interface_id, &*data as *const ImuData);
                                }
                            },
                            Err(_) => {
                                handle_err!();
                            }
                        }
                    }
                    Err(_) => {
                        handle_err!();
                    }
                }

                error_count = 0;
                std::thread::sleep(Duration::from_millis(1));
            }
        }));
        Ok(())
    }    

    pub fn stop(&mut self) -> Result<(), ImuError> {
        if !self.running.load(Ordering::SeqCst) {
            return Err(ImuError::ImuErrorNotRunning);
        }

        self.running.store(false, Ordering::SeqCst);
        if let Some(handle) = self.thread.take() {
            match handle.join() {
                Ok(_) => {}
                Err(_) => return Err(ImuError::ImuErrorUnknown),
            }
        }
        Ok(())
    }

    fn unlock(&mut self) -> Result<(), ImuError> {
        self.write_register(Register::KEY, KEY_UNLOCK)
    }

    pub fn save(&mut self) -> Result<(), ImuError> {
        self.write_register(Register::SAVE, Save::Save as i16)
    }

    pub fn reboot(&mut self) -> Result<(), ImuError> {
        self.write_register(Register::SAVE, Save::Reboot as i16)
    }

    pub fn reset(&mut self) -> Result<(), ImuError> {
        self.write_register(Register::SAVE, Save::Reset as i16)
    }

    pub fn get_band_width(&mut self) -> Result<u16, ImuError> {
        match self.read_register(Register::BANDWIDTH) {
            Ok(bandwidth) => Ok(match Bandwidth::try_from(bandwidth).map_err(|_| ImuError::ImuErrorUnknown)? {
                Bandwidth::HZ256 => 256,
                Bandwidth::HZ188 => 188,
                Bandwidth::HZ98  => 98 ,
                Bandwidth::HZ42  => 42 ,
                Bandwidth::HZ20  => 20 ,
                Bandwidth::HZ10  => 10 ,
                Bandwidth::HZ5   => 5  ,
            }),
            Err(err) => Err(err),
        }
    }

    pub fn set_band_width(&mut self, bandwidth: u16) -> Result<(), ImuError> {
        self.unlock()?;
        self.write_register(Register::BANDWIDTH, match bandwidth {
            256 => Bandwidth::HZ256.into(),
            188 => Bandwidth::HZ188.into(),
            98  => Bandwidth::HZ98.into(),
            42  => Bandwidth::HZ42.into(),
            20  => Bandwidth::HZ20.into(),
            10  => Bandwidth::HZ10.into(),
            5   => Bandwidth::HZ5.into(),
            _ => return Err(ImuError::ImuErrorInvalidParams),
        })
    }

    pub fn get_data(&mut self) -> Result<ImuData, ImuError> {
        const REG_NUM: usize = (Register::TEMP as usize + 1) - Register::AX as usize;
        const BUFF_SIZE: usize = 2*REG_NUM+5;
        let raw_data = self.read_registers::<REG_NUM, BUFF_SIZE>(Register::AX)
            .map_err(|_| ImuError::ImuErrorReadFailed)?;

        Ok(ImuData {
            acc: [
                (raw_data[0] as f64 / 32768.0 * 16.0) as f32, 
                (raw_data[1] as f64 / 32768.0 * 16.0) as f32, 
                (raw_data[2] as f64 / 32768.0 * 16.0) as f32,
            ],
            gyro: [
                (raw_data[3] as f64 / 32768.0 * 2000.0) as f32, 
                (raw_data[4] as f64 / 32768.0 * 2000.0) as f32, 
                (raw_data[5] as f64 / 32768.0 * 2000.0) as f32,
            ],
            angle: [
                (raw_data[9] as f64 / 32768.0 * 180.0) as f32, 
                (raw_data[10] as f64 / 32768.0 * 180.0) as f32, 
                (raw_data[11] as f64 / 32768.0 * 180.0) as f32,
            ],
            mag: [
                raw_data[6] as i32, 
                raw_data[7] as i32, 
                raw_data[8] as i32,
            ],
            temp: raw_data[12] as f32 / 100.0,
        })
    }

    pub fn check_device(&mut self) -> Result<Option<ImuDeviceData>, ImuError> {
        match self.read_register(Register::IICADDR) {
            Ok(addr) => if addr as u8 == self.device.addr {
                Ok(Some(self.device.clone()))
            } else {
                Ok(None)
            }
            Err(ImuError::ImuErrorReadFailed) => Ok(None),
            Err(err) => Err(err),          
        }
    }
}
