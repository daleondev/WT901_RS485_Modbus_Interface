use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use crate::hardware::{read_reg_cmd, write_reg_cmd, Bandwidth, Register, Save, KEY_UNLOCK};
use crate::imu_interface_ffi::{ImuData, ImuDataCallback, ImuError};
use crate::imu_interface_manager::Uuid;
use crate::serial_interface::{clear_serial, open_serial, read_serial, read_serial_exact, write_serial, SerialError};

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

    pub fn read_registers<const REG_NUM: usize, const BUFF_SIZE: usize>(&mut self, start_reg: Register) -> Result<[u16; REG_NUM], ImuError> {      
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

        let mut data: [u16; REG_NUM] = [0; REG_NUM];
        let mut j: usize = 3;
        for i in 0..REG_NUM {
            data[i] = ((result[j] as u16) << 8) | (result[j+1] as u16);
            j += 2;
        }
        Ok(data)
    }

    pub fn read_register(&mut self, reg: Register) -> Result<u16, ImuError> {
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
        Ok(((result[3] as u16) << 8) | (result[4] as u16))
    }

    pub fn write_register(&mut self, reg: Register, data: u16) -> Result<(), ImuError> {
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
        let addr = self.device.addr.clone();

        self.running.store(true, Ordering::SeqCst);
        self.thread = Some(std::thread::spawn(move || {
            const REG_NUM: usize = (Register::TEMP as usize + 1) - Register::AX as usize;
            const BUFF_SIZE: usize = 2*REG_NUM+5;
            let mut buff: [u8; BUFF_SIZE] = [0; BUFF_SIZE];
            let mut clear_buff: [u8; 255] = [0; 255];

            while running.load(Ordering::SeqCst) {
                let result = port.lock();
                if result.is_err() {
                    continue;
                }
                let mut port = result.unwrap();

                let result = port.as_mut();
                if result.is_none() {
                    continue;
                }
                let port = result.unwrap();

                let _ = clear_serial(port);
                if let Ok(read_cmd) = read_reg_cmd(addr, Register::AX, REG_NUM) {
                    if write_serial(port, &read_cmd).is_err() {
                        continue;
                    }
                    if let Ok(_) = read_serial_exact(port, &mut buff) {
                        if buff[0] != read_cmd[0] || buff[1] != read_cmd[1] || buff[2] != 2*REG_NUM as u8 {
                            continue;
                        }
                    }
                }

                let mut raw_data: [u16; REG_NUM] = [0; REG_NUM];
                let mut j: usize = 3;
                for i in 0..REG_NUM {
                    raw_data[i] = ((buff[j] as u16) << 8) | (buff[j+1] as u16);
                    j += 2;
                }

                if let Ok(mut data) = data.lock() {
                    data.acc[0] = (raw_data[0] as f64 / 32768.0 * 16.0) as f32;
                    data.acc[1] = (raw_data[1] as f64 / 32768.0 * 16.0) as f32;
                    data.acc[2] = (raw_data[2] as f64 / 32768.0 * 16.0) as f32;
                    data.gyro[0] = (raw_data[3] as f64 / 32768.0 * 2000.0) as f32;
                    data.gyro[1] = (raw_data[4] as f64 / 32768.0 * 2000.0) as f32;
                    data.gyro[2] = (raw_data[5] as f64 / 32768.0 * 2000.0) as f32;
                    data.mag[0] = raw_data[6] as i32;
                    data.mag[1] = raw_data[7] as i32;
                    data.mag[2] = raw_data[8] as i32;
                    data.angle[0] = convert_angle(raw_data[9] as f64 / 32768.0 * 180.0); 
                    data.angle[1] = convert_angle(raw_data[10] as f64 / 32768.0 * 180.0); 
                    data.angle[2] = convert_angle(raw_data[11] as f64 / 32768.0 * 180.0);

                    if let Ok(mut callback) = data_callback.lock() {
                        if let Some(callback) = callback.as_mut() {
                            callback(&*data as *const ImuData);
                        }
                    }
                }
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
        self.write_register(Register::SAVE, Save::Save as u16)
    }

    pub fn reboot(&mut self) -> Result<(), ImuError> {
        self.write_register(Register::SAVE, Save::Reboot as u16)
    }

    pub fn reset(&mut self) -> Result<(), ImuError> {
        self.write_register(Register::SAVE, Save::Reset as u16)
    }

    pub fn get_band_width(&mut self) -> Result<u16, ImuError> {
        match self.read_register(Register::BANDWIDTH) {
            Ok(bandwidth) => Ok(match Bandwidth::try_from(bandwidth).map_err(|_| ImuError::ImuErrorUnknown)? {
                Bandwidth::HZ256 => 256,
                Bandwidth::HZ184 => 188,
                Bandwidth::HZ94  => 98 ,
                Bandwidth::HZ44  => 42 ,
                Bandwidth::HZ21  => 20 ,
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
            188 => Bandwidth::HZ184.into(),
            98  => Bandwidth::HZ94.into(),
            42  => Bandwidth::HZ44.into(),
            20  => Bandwidth::HZ21.into(),
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
                convert_angle(raw_data[9] as f64 / 32768.0 * 180.0), 
                convert_angle(raw_data[10] as f64 / 32768.0 * 180.0), 
                convert_angle(raw_data[11] as f64 / 32768.0 * 180.0),
            ],
            mag: [
                raw_data[6] as i32, 
                raw_data[7] as i32, 
                raw_data[8] as i32,
            ],
            temp: raw_data[12] as f32 / 100.0,
        })
    }
}

fn convert_angle(angle: f64) -> f32 {
    (if angle > 180.0 { angle - 360.0 } else { angle }) as f32
}

// pub fn write_serial(
//     port: Arc<Mutex<Option<Box<dyn serialport::SerialPort>>>>,
//     data: &[u8],
// ) -> Result<(), ()> {
//     let mut port = port.lock().map_err(|_| ())?;
//     let port = port.as_mut().ok_or(())?;
//     port.write_all(data).map_err(|_| ())?;
//     port.flush().map_err(|_| ())?;
//     // let delay_us = ((1000000 / (port.baud_rate().ok().unwrap() / 10)) * data.len() as u32) + 300;
//     // std::thread::sleep(std::time::Duration::from_micros(delay_us as u64));
//     Ok(())
// }

// pub fn read_serial(
//     port: Arc<Mutex<Option<Box<dyn serialport::SerialPort>>>>,
//     data: &mut [u8],
// ) -> Result<usize, ()> {
//     let mut port = port.lock().map_err(|_| ())?;
//     let port = port.as_mut().unwrap();
//     let bytes_read = match port.read(data) {
//         Ok(bytes) => bytes,
//         Err(_) => 0,
//     };
//     Ok(bytes_read)
// }

// pub fn scan_devices() -> Result<Vec<ImuDeviceData>, ImuError> {
//     let mut devices = Vec::new();
//     let ports = serialport::available_ports().unwrap_or_else(|_| vec![]);
//     if ports.is_empty() {
//         return Err(ImuError::ImuErrorNoDevicesFound);
//     }

//     const BAUDRATES: [u32; 5] = [115200, 57600, 38400, 19200, 9600];

//     let mut itf = ImuInterface::new(
//         u32::MAX,
//         ImuDeviceData {
//             name: "".to_string(),
//             baudrate: 0,
//             addr: 0,
//         },
//     );
//     for port in ports {
//         itf.device.name = port.port_name.clone();
//         for baudrate in BAUDRATES.iter() {
//             itf.device.baudrate = *baudrate;
//             if let Ok(_) = itf.open_serial() {
//                 for addr in 0x00..0xFF {
//                     itf.device.addr = addr;
//                     println!(
//                         "Trying port: {}, baudrate: {}, addr: {}",
//                         port.port_name, baudrate, addr
//                     );
//                     if let Err(_) = itf.init() {
//                         continue;
//                     }
//                     if let Ok(device) = itf.scan_device() {
//                         if device.is_some() {
//                             println!("Found device {:?}", itf.device.clone());
//                             devices.push(itf.device.clone());
//                         }
//                     }
//                 }
//             } else {
//                 println!(
//                     "Failed to open port: {}, baudrate: {}",
//                     port.port_name, baudrate
//                 );
//             }
//         }
//     }
//     if devices.is_empty() {
//         return Err(ImuError::ImuErrorNoDevicesFound);
//     }
//     Ok(devices)
// }

// impl ImuInterface {
//     pub fn new(id: Uuid, device: ImuDeviceData) -> ImuInterface {
//         ImuInterface {
//             id,
//             device,
//             port: Arc::new(Mutex::new(None)),
//             thread: None,
//             running: Arc::new(AtomicBool::new(false)),
//             data_callback: None,
//             data: Arc::new(Mutex::new(ImuData {
//                 acc: [0.0; 3],
//                 gyro: [0.0; 3],
//                 angle: [0.0; 3],
//                 mag: [0; 3],
//                 temp: 0.0,
//             })),
//             hal: Arc::new(Mutex::new(HAL::new())),
//         }
//     }

//     pub fn init(&mut self) -> Result<(), ImuError> {
//         let mut hal = self.hal.lock().map_err(|_| ImuError::ImuErrorUnknown)?;
//         hal.init(self.device.addr);
//         Ok(())
//     }

//     pub fn open_serial(&mut self) -> Result<(), ImuError> {
//         let port = serialport::new(self.device.name.clone().as_str(), self.device.baudrate)
//             .timeout(std::time::Duration::from_millis(10))
//             .open()
//             .map_err(|_| ImuError::ImuErrorOpenFailed)?;
//         *self.port.lock().map_err(|_| ImuError::ImuErrorUnknown)? = Some(port);
//         Ok(())
//     }

//     pub fn scan_device(&mut self) -> Result<Option<ImuDeviceData>, ImuError> {
//         let mut hal = self.hal.lock().unwrap();

//         let read_cmd = hal.read_reg_cmd(AX as u32, 12).map_err(|_| ImuError::ImuErrorUnknown)?;
//         match write_serial(Arc::clone(&self.port), &read_cmd) {
//             Ok(_) => {}
//             Err(_) => return Err(ImuError::ImuErrorUnknown),
//         }
//         // std::thread::sleep(std::time::Duration::from_millis(50));

//         let mut buff: [u8; 1] = [0; 1];
//         while read_serial(Arc::clone(&self.port), &mut buff).map_err(|_| ImuError::ImuErrorUnknown)? > 0 {
//             hal.data_in(buff[0]);
//         }

//         if hal.data_update != 0 {
//             return Ok(Some(self.device.clone()));
//         }

//         Ok(None)
//     }

//     pub fn set_data_callback(&mut self, callback: ImuDataCallback) {
//         self.data_callback = Some(Arc::new(callback));
//     }

//     pub fn start(&mut self) -> Result<(), ImuError> {
//         if self.running.load(Ordering::SeqCst) {
//             return Err(ImuError::ImuErrorAlreadyRunning);
//         }

//         let port = Arc::clone(&self.port);
//         let running = Arc::clone(&self.running);
//         let data = Arc::clone(&self.data);
//         let mut data_callback: Option<Arc<ImuDataCallback>> = None;
//         if self.data_callback.is_some() {
//             data_callback = Some(Arc::clone(&self.data_callback.as_ref().unwrap()));
//         }
//         let hal = Arc::clone(&self.hal);

//         self.running.store(true, Ordering::SeqCst);
//         self.thread = Some(std::thread::spawn(move || {
//             while running.load(Ordering::SeqCst) {
//                 let mut hal = hal.lock().unwrap();

//                 let read_cmd = hal.read_reg_cmd(AX as u32, 13).unwrap();
//                 write_serial(Arc::clone(&port), &read_cmd).unwrap();
//                 // std::thread::sleep(std::time::Duration::from_millis(50));

//                 let mut buff: [u8; 1] = [0; 1];
//                 while read_serial(Arc::clone(&port), &mut buff).unwrap() > 0 {
//                     hal.data_in(buff[0]);
//                 }

//                 if hal.data_update != 0 {
//                     if hal.data_update & ACC_UPDATE != 0 {
//                         if let Ok(mut data) = data.lock() {
//                             data.acc[0] = (hal.reg[AX as usize] as f64 / 32768.0 * 16.0) as f32;
//                             data.acc[1] = (hal.reg[AY as usize] as f64 / 32768.0 * 16.0) as f32;
//                             data.acc[2] = (hal.reg[AZ as usize] as f64 / 32768.0 * 16.0) as f32;
//                         }
//                         hal.data_update &= !ACC_UPDATE;
//                     }
//                     if hal.data_update & GYRO_UPDATE != 0 {
//                         if let Ok(mut data) = data.lock() {
//                             data.gyro[0] = (hal.reg[GX as usize] as f64 / 32768.0 * 2000.0) as f32;
//                             data.gyro[1] = (hal.reg[GY as usize] as f64 / 32768.0 * 2000.0) as f32;
//                             data.gyro[2] = (hal.reg[GZ as usize] as f64 / 32768.0 * 2000.0) as f32;
//                         }
//                         hal.data_update &= !GYRO_UPDATE;
//                     }
//                     if hal.data_update & ANGLE_UPDATE != 0 {
//                         if let Ok(mut data) = data.lock() {
//                             let mut tmp = hal.reg[ROLL as usize] as f64 / 32768.0 * 180.0;
//                             data.angle[0] = if tmp > 180.0 { tmp - 360.0 } else { tmp } as f32;

//                             tmp = hal.reg[PITCH as usize] as f64 / 32768.0 * 180.0;
//                             data.angle[1] = if tmp > 180.0 { tmp - 360.0 } else { tmp } as f32;

//                             tmp = hal.reg[YAW as usize] as f64 / 32768.0 * 180.0;
//                             data.angle[2] = if tmp > 180.0 { tmp - 360.0 } else { tmp } as f32;
//                         }
//                         hal.data_update &= !ANGLE_UPDATE;
//                     }
//                     if hal.data_update & MAG_UPDATE != 0 {
//                         if let Ok(mut data) = data.lock() {
//                             data.mag[0] = hal.reg[HX as usize] as i32;
//                             data.mag[1] = hal.reg[HY as usize] as i32;
//                             data.mag[2] = hal.reg[HZ as usize] as i32;
//                         }
//                         hal.data_update &= !MAG_UPDATE;
//                     }
//                     if hal.data_update & TEMP_UPDATE != 0 {
//                         if let Ok(mut data) = data.lock() {
//                             data.temp = hal.reg[TEMP as usize] as f32 / 100.0;
//                             if let Some(ref callback) = data_callback {
//                                 callback(&*data as *const ImuData);
//                             }
//                         }
//                         hal.data_update &= !TEMP_UPDATE;
//                     }
//                     hal.data_update = 0;
//                 }
//             }
//         }));
//         Ok(())
//     }

//     pub fn stop(&mut self) -> Result<(), ImuError> {
//         if !self.running.load(Ordering::SeqCst) {
//             return Err(ImuError::ImuErrorNotRunning);
//         }

//         self.running.store(false, Ordering::SeqCst);
//         if let Some(handle) = self.thread.take() {
//             match handle.join() {
//                 Ok(_) => {}
//                 Err(_) => return Err(ImuError::ImuErrorUnknown),
//             }
//         }
//         Ok(())
//     }

//     pub fn read_register(&mut self, reg: u32) -> Result<u16, ImuError> {
//         if self.running.load(Ordering::SeqCst) || self.thread.is_some() {
//             return Err(ImuError::ImuErrorAlreadyRunning);
//         }

//         let mut hal = self.hal.lock().map_err(|_| ImuError::ImuErrorReadFailed)?;

//         let read_cmd = hal.read_reg_cmd(reg, 1).map_err(|_| ImuError::ImuErrorReadFailed)?;
//         write_serial(Arc::clone(&self.port), &read_cmd).map_err(|_| ImuError::ImuErrorReadFailed)?;

//         let mut buff: [u8; 1] = [0; 1];
//         let mut result: [u8; 8] = [0; 8];
//         let mut i: usize = 0; 
//         while i < 7 {
//             if read_serial(Arc::clone(&self.port), &mut buff).map_err(|_| ImuError::ImuErrorReadFailed)? > 0 {              
//                 result[i] = buff[0];
//                 i += 1;
//             }
//         }

//         for i in 0..8 {
//             println!("0x{:02X} - 0x{:02X}", read_cmd[i], result[i]);
//         }

//         if result[0] != read_cmd[0] || result[1] != read_cmd[1] || result[2] != 2 {
//             return Err(ImuError::ImuErrorReadFailed);
//         }

//         let result = ((result[3] as u16) << 8) | (result[4] as u16);
//         Ok(result)
//     }

//     pub fn write_register(&mut self, reg: u32, data: u16) -> Result<(), ImuError> {
//         if self.running.load(Ordering::SeqCst) || self.thread.is_some() {
//             return Err(ImuError::ImuErrorAlreadyRunning);
//         }

//         let mut hal = self.hal.lock().map_err(|_| ImuError::ImuErrorWriteFailed)?;

//         let write_cmd = hal.write_reg_cmd(reg, data).map_err(|_| ImuError::ImuErrorWriteFailed)?;
//         write_serial(Arc::clone(&self.port), &write_cmd).map_err(|_| ImuError::ImuErrorWriteFailed)?;

//         let mut buff: [u8; 1] = [0; 1];
//         let mut handshake: [u8; 8] = [0; 8];
//         let mut i: usize = 0; 
//         while i < 8 {
//             if read_serial(Arc::clone(&self.port), &mut buff).map_err(|_| ImuError::ImuErrorWriteFailed)? > 0 {
//                 handshake[i] = buff[0];
//                 i += 1;
//             }
//         }

//         for i in 0..8 {
//             println!("0x{:02X} - 0x{:02X}", write_cmd[i], handshake[i]);
//         }

//         if write_cmd != handshake {
//             return Err(ImuError::ImuErrorWriteFailed);
//         }

//         Ok(())
//     }

//     fn unlock(&mut self) -> Result<(), ImuError> {
//         self.write_register(KEY as u32, 0xB588)
//     }

//     pub fn save(&mut self) -> Result<(), ImuError> {
//         self.write_register(SAVE as u32, 0x0000)
//     }

//     pub fn reboot(&mut self) -> Result<(), ImuError> {
//         self.write_register(SAVE as u32, 0x00FF)
//     }

//     pub fn reset(&mut self) -> Result<(), ImuError> {
//         self.write_register(SAVE as u32, 0x0001)
//     }

//     pub fn get_band_width(&mut self) -> Result<u16, ImuError> {
//         match self.read_register(BANDWIDTH as u32) {
//             Ok(bandwidth) => Ok(match bandwidth {
//                 0x00 => 256,
//                 0x01 => 188,
//                 0x02 => 98 ,
//                 0x03 => 42 ,
//                 0x04 => 20 ,
//                 0x05 => 10 ,
//                 0x06 => 5  ,
//                 _ => return Err(ImuError::ImuErrorInvalidParams),
//             }),
//             Err(err) => Err(err),
//         }
//     }

//     pub fn set_band_width(&mut self, bandwidth: u16) -> Result<(), ImuError> {
//         self.unlock()?;
//         self.write_register(BANDWIDTH as u32, match bandwidth {
//             256 => 0x00,
//             188 => 0x01,
//             98  => 0x02,
//             42  => 0x03,
//             20  => 0x04,
//             10  => 0x05,
//             5   => 0x06,
//             _ => return Err(ImuError::ImuErrorInvalidParams),
//         })
//     }
// }
