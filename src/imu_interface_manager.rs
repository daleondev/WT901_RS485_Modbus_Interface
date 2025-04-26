use std::collections::HashMap;
use std::sync::{Mutex, MutexGuard, OnceLock,};
use std::sync::atomic::{AtomicU32, Ordering};

use crate::imu_interface_ffi::ImuError;
use crate::imu_interface::ImuInterface;

pub type Uuid = u32;
pub const UUID_NULL: Uuid = u32::MIN;

pub fn new_uuid() -> Uuid {
    static ID: AtomicU32 = AtomicU32::new(UUID_NULL+1);
    ID.fetch_add(1, Ordering::Relaxed)
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

        if !self.interfaces.contains_key(&id) {
            println!("Interface with ID {} does not exist", id);
            return Err(ImuError::ImuErrorInterfaceDoesntExist);
        }

        self.interfaces.remove(&id);
        Ok(())
    }

    pub fn get_interface(&mut self, id: Uuid) -> Result<&mut ImuInterface, ImuError> {
        assert!(id != UUID_NULL, "Cannot get interface with invalid ID");

        if !self.interfaces.contains_key(&id) {
            println!("Interface with ID {} does not exist", id);
            return Err(ImuError::ImuErrorInterfaceDoesntExist);
        }

        Ok(self.interfaces.get_mut(&id).unwrap())
    }
}
