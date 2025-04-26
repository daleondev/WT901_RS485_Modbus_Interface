use std::ffi::{c_char, c_float, c_int, c_uint, c_ushort};
use std::mem::zeroed;

use crate::imu_interface_manager::{ImuInterfaceManager, get_imu_manager, new_uuid, UUID_NULL};
use crate::imu_interface::{ImuDeviceData, ImuInterface};

#[repr(C)]
pub struct ImuDevice {
    pub name: *const c_char,
    pub baudrate: c_uint,
    pub addr: c_uint,
}

#[repr(C)]
#[derive(Debug)]
pub struct ImuData {
    pub acc: [c_float; 3],
    pub gyro: [c_float; 3],
    pub angle: [c_float; 3],
    pub mag: [c_int; 3],
    pub temp: c_float,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ImuError {
    ImuSuccess = 0,
    ImuErrorAlreadyInit,
    ImuErrorNotInit,
    ImuErrorInitFailed,
    ImuErrorNoDevicesFound,
    ImuErrorInterfaceAlreadyExists,
    ImuErrorInterfaceDoesntExist,
    ImuErrorOpenFailed,
    ImuErrorAlreadyRunning,
    ImuErrorNotRunning,
    ImuErrorReadFailed,
    ImuErrorWriteFailed,
    ImuErrorInvalidParams,
    ImuErrorUnknown,
}

pub type ImuDataCallback = Option<extern "C" fn(data: *const ImuData)>;

#[unsafe(no_mangle)]
pub extern "C" fn imu_init() -> ImuError
{
    let mut manager = get_imu_manager();
    match *manager {
        Some(_) => {
            return ImuError::ImuErrorAlreadyInit;
        },
        None => *manager = Some(ImuInterfaceManager::new()),
    }

    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_shutdown() -> ImuError
{
    let mut manager = get_imu_manager();
    match *manager {
        Some(_) => {
            *manager = None;
        },
        None => return ImuError::ImuErrorNotInit,
    }

    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_scan_devices(out_devices: *mut ImuDevice, out_num_devices: *mut c_uint) -> ImuError
{
    // unsafe { *out_num_devices = 0 };

    // let devices = match scan_devices() {
    //     Ok(devices) => devices,
    //     Err(err) => return err,
    // };

    // unsafe {
    //     for (i, device) in devices.iter().enumerate() {
    //         let device_ptr = out_devices.add(i);
    //         (*device_ptr).name = device.name.as_ptr() as *const c_char;
    //         (*device_ptr).baudrate = device.baudrate;
    //         (*device_ptr).addr = device.addr as c_uint;
    //     }
    //     *out_num_devices = devices.len() as c_uint;
    // }

    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_create_interface(device: *const ImuDevice, out_interface_id: *mut c_uint) -> ImuError
{
    unsafe { *out_interface_id = UUID_NULL };

    let mut manager = get_imu_manager();
    let manager = match manager.as_mut() {
        Some(manager) => manager,
        None => return ImuError::ImuErrorNotInit,
    };

    let device = ImuDeviceData {
        name: unsafe { std::ffi::CStr::from_ptr((*device).name).to_string_lossy().into_owned() },
        baudrate: unsafe { (*device).baudrate },
        addr: unsafe { (*device).addr as u8 },
    };

    let id = new_uuid();
    let interface = ImuInterface::new(id, device);
    println!("Creating interface with ID {} and device name {}", id, interface.device.name);    
    
    let interface = match manager.add_interface(interface) {
        Ok(interface) => interface,
        Err(err) => return err,
    };

    if interface.init().is_err() {
        match manager.remove_interface(id) {
            Ok(_) => {},
            Err(err) => {
                return err;
            }
        }
        return ImuError::ImuErrorInitFailed;
    }

    // match interface.open_serial() {
    //     Ok(_) => {},
    //     Err(err) => {
    //         match manager.remove_interface(id) {
    //             Ok(_) => {},
    //             Err(err) => {
    //                 return err;
    //             }
    //         }
    //         return err;
    //     }
    // }

    unsafe { *out_interface_id = id };
    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_destroy_interface(interface_id: c_uint) -> ImuError
{
    let mut manager = get_imu_manager();
    let manager = match manager.as_mut() {
        Some(manager) => manager,
        None => return ImuError::ImuErrorNotInit,
    };

    match manager.remove_interface(interface_id) {
        Ok(_) => {},
        Err(err) => return err,
    };

    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_set_data_callback(interface_id: c_uint, callback: ImuDataCallback) -> ImuError
{
    let mut manager = get_imu_manager();
    let manager = match manager.as_mut() {
        Some(manager) => manager,
        None => return ImuError::ImuErrorNotInit,
    };

    let interface = match manager.get_interface(interface_id) {
        Ok(interface) => interface,
        Err(err) => return err,
    };

    match interface.set_data_callback(callback) {
        Ok(_) => {},
        Err(err) => return err,
    }
    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_start_task(interface_id: c_uint) -> ImuError
{
    let mut manager = get_imu_manager();
    let manager = match manager.as_mut() {
        Some(manager) => manager,
        None => return ImuError::ImuErrorNotInit,
    };

    let interface = match manager.get_interface(interface_id) {
        Ok(interface) => interface,
        Err(err) => return err,
    };

    println!("Starting interface with ID {}", interface_id);
    match interface.start() {
        Ok(_) => {},
        Err(err) => return err,
    };

    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_stop_task(interface_id: c_uint) -> ImuError
{
    let mut manager = get_imu_manager();
    let manager = match manager.as_mut() {
        Some(manager) => manager,
        None => return ImuError::ImuErrorNotInit,
    };

    let interface = match manager.get_interface(interface_id) {
        Ok(interface) => interface,
        Err(err) => return err,
    };

    match interface.stop() {
        Ok(_) => {},
        Err(err) => return err,
    };

    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_save(interface_id: c_uint) -> ImuError
{
    let mut manager = get_imu_manager();
    let manager = match manager.as_mut() {
        Some(manager) => manager,
        None => return ImuError::ImuErrorNotInit,
    };

    let interface = match manager.get_interface(interface_id) {
        Ok(interface) => interface,
        Err(err) => return err,
    };

    match interface.save() {
        Ok(_) => {},
        Err(err) => return err,
    };

    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_reboot(interface_id: c_uint) -> ImuError
{
    let mut manager = get_imu_manager();
    let manager = match manager.as_mut() {
        Some(manager) => manager,
        None => return ImuError::ImuErrorNotInit,
    };

    let interface = match manager.get_interface(interface_id) {
        Ok(interface) => interface,
        Err(err) => return err,
    };

    match interface.reboot() {
        Ok(_) => {},
        Err(err) => return err,
    };

    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_reset(interface_id: c_uint) -> ImuError
{
    let mut manager = get_imu_manager();
    let manager = match manager.as_mut() {
        Some(manager) => manager,
        None => return ImuError::ImuErrorNotInit,
    };

    let interface = match manager.get_interface(interface_id) {
        Ok(interface) => interface,
        Err(err) => return err,
    };

    match interface.reset() {
        Ok(_) => {},
        Err(err) => return err,
    };

    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_get_bandwidth(interface_id: c_uint, out_bandwidth: *mut c_ushort) -> ImuError
{
    unsafe { *out_bandwidth = 0 };

    let mut manager = get_imu_manager();
    let manager = match manager.as_mut() {
        Some(manager) => manager,
        None => return ImuError::ImuErrorNotInit,
    };

    let interface = match manager.get_interface(interface_id) {
        Ok(interface) => interface,
        Err(err) => return err,
    };

    let bandwidth =  match interface.get_band_width() {
        Ok(bandwidth) => bandwidth,
        Err(err) => return err,
    };

    unsafe { *out_bandwidth = bandwidth };
    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_set_bandwidth(interface_id: c_uint, bandwidth: c_ushort) -> ImuError
{
    let mut manager = get_imu_manager();
    let manager = match manager.as_mut() {
        Some(manager) => manager,
        None => return ImuError::ImuErrorNotInit,
    };

    let interface = match manager.get_interface(interface_id) {
        Ok(interface) => interface,
        Err(err) => return err,
    };

    match interface.set_band_width(bandwidth) {
        Ok(_) => {},
        Err(err) => return err,
    };

    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_get_data(interface_id: c_uint, out_data: *mut ImuData) -> ImuError
{
    unsafe { *out_data = zeroed() };

    let mut manager = get_imu_manager();
    let manager = match manager.as_mut() {
        Some(manager) => manager,
        None => return ImuError::ImuErrorNotInit,
    };

    let interface = match manager.get_interface(interface_id) {
        Ok(interface) => interface,
        Err(err) => return err,
    };

    let data =  match interface.get_data() {
        Ok(data) => data,
        Err(err) => return err,
    };

    unsafe { *out_data = data };
    ImuError::ImuSuccess
}