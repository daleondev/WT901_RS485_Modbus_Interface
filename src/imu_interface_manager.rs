use std::collections::HashMap;
use std::sync::{Mutex, MutexGuard, OnceLock,};
use std::sync::atomic::{AtomicU32, Ordering};

use crate::imu_interface_ffi::ImuError;
use crate::imu_interface::{ImuDeviceData, ImuInterface};

pub type Uuid = u32;
pub const UUID_NULL: Uuid = u32::MIN;

pub fn new_uuid() -> Uuid {
    static ID: AtomicU32 = AtomicU32::new(UUID_NULL+1);
    ID.fetch_add(1, Ordering::SeqCst)
}

pub fn get_imu_manager() -> MutexGuard<'static, Option<ImuInterfaceManager>> {
    static IMU_MANAGER: OnceLock<Mutex<Option<ImuInterfaceManager>>> = OnceLock::new();
    IMU_MANAGER.get_or_init(Default::default)
        .lock()
        .expect("Let's hope the lock isn't poisoned")
}

pub struct ImuInterfaceManager {
    interfaces: HashMap<Uuid, ImuInterface>,
}

impl ImuInterfaceManager {
    pub fn new() -> Self {
        Self {
            interfaces: HashMap::new(),
        }
    }

    pub fn add_interface(&mut self, interface: ImuInterface) -> Result<&mut ImuInterface, ImuError> {
        let id = interface.id;
        assert!(id != UUID_NULL, "Cannot add interface with invalid ID");
        
        if self.interfaces.contains_key(&id) {
            println!("Interface with ID {} already exists", id);
            return Err(ImuError::ImuErrorInterfaceAlreadyExists);
        }

        self.interfaces.insert(id, interface);
        Ok(self.interfaces.get_mut(&id).unwrap())
    }

    pub fn remove_interface(&mut self, id: Uuid) -> Result<(), ImuError> {
        assert!(id != UUID_NULL, "Cannot remove interface with invalid ID");

        match self.interfaces.remove(&id) {
            Some(_) => Ok(()),
            None => {
                println!("Interface with ID {} does not exist", id);
                Err(ImuError::ImuErrorInterfaceDoesntExist)
            }
        }
    }

    pub fn get_interface(&mut self, id: Uuid) -> Result<&mut ImuInterface, ImuError> {
        assert!(id != UUID_NULL, "Cannot get interface with invalid ID");
        
        match self.interfaces.get_mut(&id) {
            Some(interface) => Ok(interface),
            None => {
                println!("Interface with ID {} does not exist", id);
                Err(ImuError::ImuErrorInterfaceDoesntExist)
            }
        }
    }

    pub fn scan_interfaces(&self, device_names: Vec<String>, baud_rates: Vec<u32>, addr_range: [u8;2]) -> Result<Vec<ImuDeviceData>, ImuError> {
        if device_names.is_empty() || baud_rates.is_empty() || addr_range[0] > addr_range[1] {
            return Err(ImuError::ImuErrorNoDevicesFound);
        }

        let mut devices = Vec::new(); 
        let mut itf = ImuInterface::zeroed();
        for name in device_names {
            itf.device.name = name.clone();
            for baudrate in baud_rates.iter() {
                itf.device.baudrate = *baudrate;
                if let Ok(_) = itf.init() {
                    for addr in addr_range[0]..(addr_range[1]+1) {
                        itf.device.addr = addr;
                        println!("Trying port: {}, baudrate: {}, addr: {}", name, baudrate, addr);
                        if let Ok(device) = itf.check_device() {
                            if device.is_some() {
                                println!("Found device {:?}", itf.device.clone());
                                devices.push(itf.device.clone());
                            }
                        }
                    }
                } else {
                    println!("Failed to open port: {}, baudrate: {}", name, baudrate);
                }
            }
        }

        if devices.is_empty() {
            return Err(ImuError::ImuErrorNoDevicesFound);
        }
        Ok(devices)
    }
}
