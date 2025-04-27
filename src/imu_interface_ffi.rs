use std::ffi::{c_char, c_float, c_int, c_uchar, c_uint, c_ushort, CString};
use std::mem::zeroed;
use std::sync::atomic::Ordering;

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

#[allow(unused_assignments)]
#[unsafe(no_mangle)]
pub extern "C" fn imu_scan_devices(
    device_names: *const *const c_char, num_device_names: usize,
    baud_rates: *const c_uint, num_baud_rates: usize,
    addr_range_low: c_uchar, addr_range_high: c_uchar,
    out_devices: *mut *mut ImuDevice, out_num_devices: *mut usize) -> ImuError
{
    unsafe { 
        *out_devices = std::ptr::null_mut();
        *out_num_devices = 0;
    };

    let mut names: Vec<String> = Vec::new();
    let mut bauds: Vec<u32> = Vec::new();
    unsafe {
        for i in 0..num_device_names as usize {
            let name_ptr = device_names.add(i);
            names.push(std::ffi::CStr::from_ptr(*name_ptr).to_string_lossy().into_owned());
        }
        for i in 0..num_baud_rates as usize {
            let baud_ptr = baud_rates.add(i);
            bauds.push(*baud_ptr);
        }        
    }

    let mut addr_range: [u8;2] = [0;2];
    addr_range[0] = addr_range_low;
    addr_range[1] = addr_range_high;

    let mut manager = get_imu_manager();
    let manager = match manager.as_mut() {
        Some(manager) => manager,
        None => return ImuError::ImuErrorNotInit,
    };

    let devices_data = match manager.scan_interfaces(names, bauds, addr_range) {
        Ok(devices_data) => devices_data,
        Err(err) => return err,
    };

    let mut devices: Vec<ImuDevice> = Vec::with_capacity(devices_data.len());
    for device_data in devices_data {
        devices.push(
            ImuDevice { 
                name: match CString::new(device_data.name) {
                    Ok(name) => name.into_raw(),
                    Err(_) => return ImuError::ImuErrorUnknown,
                },
                baudrate: device_data.baudrate, 
                addr: device_data.addr as c_uint 
            }
        );
    }

    let num_devices = devices.len();
    let devices_ptr = devices.as_mut_ptr() as *mut ImuDevice;
    std::mem::forget(devices);

    unsafe {
        *out_devices = devices_ptr;
        *out_num_devices = num_devices as usize;
    }

    ImuError::ImuSuccess
}

#[unsafe(no_mangle)]
pub extern "C" fn imu_free_devices(devices: *mut ImuDevice, num_devices: usize) 
{  
    if devices.is_null() {
        return;
    }
    unsafe {
        let devices_vec = Vec::from_raw_parts(devices, num_devices as usize, num_devices);
        for device in devices_vec {
            if !device.name.is_null() {
                drop(CString::from_raw(device.name as *mut c_char));
            }
        }
    }
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
pub extern "C" fn imu_running(interface_id: c_uint, running: *mut bool) -> ImuError
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

    unsafe { *running = interface.running.load(Ordering::SeqCst) };
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