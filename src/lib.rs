mod imu_interface;
mod imu_interface_manager;
mod imu_interface_ffi;
mod hal;
mod reg;

// #[cfg(test)]
// mod tests {

//     #[test]
//     fn it_works() {
        
//     }
// }

// #[repr(C)]
// #[derive(Debug, Copy, Clone, PartialEq, Eq)]
// pub enum ImuError {
//     ImuSuccess = 0,
//     ImuErrorAlreadyInit,
//     ImuErrorNotInit,
//     ImuErrorInitFailed,
//     ImuErrorNoDevicesFound,
//     ImuErrorInterfaceAlreadyExists,
//     ImuErrorInterfaceDoesntExist,
//     ImuErrorOpenFailed,
//     ImuErrorAlreadyRunning,
//     ImuErrorNotRunning,
//     ImuErrorUnknown,
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn init_imu() -> ImuError
// {
//     ImuError::ImuSuccess
// }