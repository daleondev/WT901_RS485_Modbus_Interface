use num_enum::{IntoPrimitive, TryFromPrimitive};

pub const REG_SIZE: u16 = 0x90;
pub const KEY_UNLOCK: i16 = 0xB588u16 as i16;

#[derive(TryFromPrimitive, IntoPrimitive)]
#[repr(u16)]
#[allow(non_camel_case_types)]
pub enum Register {
    SAVE        = 0x00,
    CALSW       = 0x01,
    BAUD        = 0x04,
    AXOFFSET    = 0x05,
    AYOFFSET    = 0x06,
    AZOFFSET    = 0x07,
    GXOFFSET    = 0x08,
    GYOFFSET    = 0x09,
    GZOFFSET    = 0x0a,
    HXOFFSET    = 0x0b,
    HYOFFSET    = 0x0c,
    HZOFFSET    = 0x0d,
    IICADDR     = 0x1a,
    LEDOFF      = 0x1b,
    MAGRANGX    = 0x1c,
    MAGRANGY    = 0x1d,
    MAGRANGZ    = 0x1e,
    BANDWIDTH   = 0x1f,
    GYRORANGE   = 0x20,
    ACCRANGE    = 0x21,
    SLEEP       = 0x22,
    ORIENT      = 0x23,
    AXIS6       = 0x24,
    FILTK       = 0x25,
    GPSBAUD     = 0x26,
    READADDR    = 0x27,
    ACCFILT     = 0x2A,
    VERSION     = 0x2e,
    YYMM        = 0x30,
    DDHH        = 0x31,
    MMSS        = 0x32,
    MS          = 0x33,
    AX          = 0x34,
    AY          = 0x35,
    AZ          = 0x36,
    GX          = 0x37,
    GY          = 0x38,
    GZ          = 0x39,
    HX          = 0x3a,
    HY          = 0x3b,
    HZ          = 0x3c,
    Roll        = 0x3d,
    Pitch       = 0x3e,
    Yaw         = 0x3f,
    TEMP        = 0x40,
    PressureL   = 0x45,
    PressureH	= 0x46,
    HeightL		= 0x47,
    HeightH		= 0x48,
    LonL		= 0x49,
    LonH		= 0x4a,
    LatL		= 0x4b,
    LatH		= 0x4c,
    GPSHeight   = 0x4d,
    GPSYAW      = 0x4e,
    GPSVL		= 0x4f,
    GPSVH		= 0x50,
    q0			= 0x51,
    q1			= 0x52,
    q2			= 0x53,
    q3			= 0x54,
    SVNUM		= 0x55,
    PDOP		= 0x56,
    HDOP		= 0x57,
    VDOP		= 0x58,
    DELAYT		= 0x59,
    XMIN        = 0x5a,
    XMAX        = 0x5b,
    ALARMPIN    = 0x5d,
    YMIN        = 0x5e,
    YMAX        = 0x5f,
    GYROCALITHR = 0x61,
    ALARMLEVEL  = 0x62,
    GYROCALTIME	= 0x63,
    TRIGTIME    = 0x68,
    KEY         = 0x69,
    WERROR      = 0x6a,
    TIMEZONE    = 0x6b,
    WZTIME      = 0x6e,
    WZSTATIC    = 0x6f,
    MODDELAY    = 0x74,
    XREFROLL    = 0x79,
    YREFPITCH   = 0x7a,
    NUMBERID1   = 0x7f,
    NUMBERID2   = 0x80,
    NUMBERID3   = 0x81,
    NUMBERID4   = 0x82,
    NUMBERID5   = 0x83,
    NUMBERID6   = 0x84,
}

#[derive(TryFromPrimitive)]
#[repr(i16)]
pub enum Baud {
    B4800       = 0x01,
    B9600       = 0x02,
    B19200      = 0x03,
    B38400      = 0x04,
    B57600      = 0x05,
    B115200     = 0x06,
    B230400     = 0x07,
}

#[derive(TryFromPrimitive, IntoPrimitive)]
#[repr(i16)]
pub enum Bandwidth {
    HZ256       = 0x00,
    HZ188       = 0x01,
    HZ98        = 0x02,
    HZ42        = 0x03,
    HZ20        = 0x04,
    HZ10        = 0x05,
    HZ5         = 0x06,
}

#[derive(TryFromPrimitive, IntoPrimitive)]
#[repr(i16)]
pub enum Save {
    Save        = 0x00,
    Reset       = 0x01,
    Reboot      = 0xff,
}

#[derive(TryFromPrimitive, IntoPrimitive)]
#[repr(i16)]
pub enum Orientation {
    Horizontal  = 0x00,
    Vertical    = 0x01, 
}

pub const FUNC_R: u8 = 0x03;
pub const FUNC_W: u8 = 0x06;

pub static CRC_HI: [u8; 256] = [
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
];

pub static CRC_LO: [u8; 256] = [
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
];

pub fn crc16(data: &[u8]) -> u16 {
    let mut crc_hi: u8 = 0xFF;
    let mut crc_lo: u8 = 0xFF;
    for i in 0..data.len() {
        let index = crc_hi ^ data[i as usize];
        crc_hi = crc_lo ^ CRC_HI[index as usize];
        crc_lo = CRC_LO[index as usize];
    }
    ((crc_hi as u16) << 8 | (crc_lo as u16)) as u16
}

pub fn read_reg_cmd(addr: u8, reg: Register, read_num: usize) -> Result<[u8; 8], ()> {
    let reg: u16 = reg.into();
    if (reg + read_num as u16) >= REG_SIZE {
        return Err(());
    }

    let mut buff: [u8; 8] = [0; 8];
    buff[0] = addr;
    buff[1] = FUNC_R;
    buff[2] = (reg >> 8) as u8; // High byte of register
    buff[3] = (reg & 0xFF) as u8; // Low byte of register
    buff[4] = (read_num >> 8) as u8; // High byte of read_num 
    buff[5] = (read_num & 0xFF) as u8; // Low byte of read_num

    let crc = crc16(&buff[0..6]); // Calculate CRC
    buff[6] = (crc >> 8) as u8; // High byte of CRC
    buff[7] = (crc & 0xFF) as u8; // Low byte of CRC

    Ok(buff)
}

pub fn write_reg_cmd(addr: u8, reg: Register, data: i16) -> Result<[u8; 8], ()> {
    let reg: u16 = reg.into();
    if reg >= REG_SIZE {
        return Err(());
    }

    let mut buff: [u8; 8] = [0; 8];
    buff[0] = addr;
    buff[1] = FUNC_W;
    buff[2] = (reg >> 8) as u8; // High byte of register
    buff[3] = (reg & 0xFF) as u8; // Low byte of register
    buff[4] = (data >> 8) as u8; // High byte of write_num 
    buff[5] = (data & 0xFF) as u8; // Low byte of write_num

    let crc = crc16(&buff[0..6]); // Calculate CRC
    buff[6] = (crc >> 8) as u8; // High byte of CRC
    buff[7] = (crc & 0xFF) as u8; // Low byte of CRC

    Ok(buff)
}