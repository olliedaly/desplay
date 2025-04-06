//! CST328 touchscreen driver for ESP32-S3 based on ESP-IDF driver
//! Adapted for esp-hal v1.0.0 (Handler Pattern)

use esp_hal::{
    delay::Delay,
    gpio::{Output}, // Only Output needed now for RST
    i2c::master::{self},
};

use defmt::{info, error};

// --- Constants ---
const CST328_ADDR: u8 = 0x1A;
const CST328_REG_READ_POINT_NUM: u16 = 0xD005;
const CST328_REG_READ_TOUCH_DATA: u16 = 0xD000;
const CST328_REG_DEBUG_INFO_MODE: u16 = 0xD101;
const CST328_REG_NORMAL_MODE: u16 = 0xD109;
const CST328_REG_INFO_X_RES: u16 = 0xD1F8;
const CST328_REG_INFO_Y_RES: u16 = 0xD1FA;
const MAX_TOUCH_POINTS: usize = 5;


/// CST328 touchscreen driver
pub struct CST328<'a, Dm: esp_hal::DriverMode> {
    i2c: master::I2c<'a, Dm>,
    rst: Output<'a>, // Keep RST pin
    delay: Delay,
    max_x: u16,
    max_y: u16,
    initialized: bool,
}

/// Represents a touch point
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct TouchPoint {
    pub x: u16,
    pub y: u16,
    pub strength: u16,
}

/// Error type
#[derive(Debug, defmt::Format)]
pub enum Error {
    CommunicationError,
    NotInitialized,
}

impl<'a, Dm: esp_hal::DriverMode> CST328<'a, Dm> {
    /// Create a new touchscreen instance
    // ** Removed int_pin parameter **
    pub fn new(
        i2c: master::I2c<'a, Dm>,
        rst_pin: Output<'a>,
        delay: Delay,
    ) -> Self {
        Self {
            i2c,
            rst: rst_pin,
            delay,
            max_x: 0, // Will be read in read_config
            max_y: 0, // Will be read in read_config
            initialized: false,
        }
    }

    /// Initialize the touchscreen
    pub fn init(&mut self) {
        info!("Initializing CST328 touchscreen");
        self.rst.set_low();
        self.delay.delay_millis(10);
        self.rst.set_high();
        self.delay.delay_millis(50);

        match self.read_config() {
             Ok(_) => {
                if self.max_x > 0 && self.max_y > 0 {
                    info!("Touchscreen configured successfully with resolution {}x{}", self.max_x, self.max_y);
                    self.initialized = true;
                } else {
                    error!("Touchscreen config read finished, but resolution is invalid ({}x{})", self.max_x, self.max_y);
                    self.initialized = false;
                }
            },
            Err(e) => {
                error!("Failed to configure touchscreen: {:?}", e);
                self.initialized = false;
            }
        }
    }

    pub fn is_initialized(&self) -> bool { self.initialized }

    /// Read touchscreen configuration (resolution)
    fn read_config(&mut self) -> Result<(), Error> {
        info!("Switching to debug info mode");
        self.i2c_write(CST328_REG_DEBUG_INFO_MODE, &[])?;
        self.delay.delay_millis(10);

        let mut buffer = [0u8; 2];

        info!("Reading X resolution");
        match self.i2c_read(CST328_REG_INFO_X_RES, &mut buffer) {
             Ok(_) => self.max_x = u16::from_le_bytes(buffer),
             Err(e) => {
                 error!("Failed reading X res: {:?}", e);
                 let _ = self.i2c_write(CST328_REG_NORMAL_MODE, &[]);
                 return Err(e);
             }
        }
        info!("X resolution: {}", self.max_x);

        info!("Reading Y resolution");
        match self.i2c_read(CST328_REG_INFO_Y_RES, &mut buffer) {
            Ok(_) => self.max_y = u16::from_le_bytes(buffer),
             Err(e) => {
                 error!("Failed reading Y res: {:?}", e);
                 let _ = self.i2c_write(CST328_REG_NORMAL_MODE, &[]);
                 return Err(e);
             }
        }
        info!("Y resolution: {}", self.max_y);

        info!("Switching back to normal mode");
        match self.i2c_write(CST328_REG_NORMAL_MODE, &[]) {
             Ok(_) => self.delay.delay_millis(10),
             Err(e) => return Err(e),
        }

        Ok(())
    }

    /// Read touch points (called after interrupt)
    pub fn read_touch_points(&mut self) -> Result<[Option<TouchPoint>; MAX_TOUCH_POINTS], Error> {
        if !self.initialized {
            error!("Touchscreen not initialized");
            return Err(Error::NotInitialized);
        }
        let mut points = [None; MAX_TOUCH_POINTS];

        let mut point_num_buffer = [0u8; 1];
        match self.i2c_read(CST328_REG_READ_POINT_NUM, &mut point_num_buffer) {
            Ok(_) => {
                let touch_count = point_num_buffer[0] & 0x0F;
                if touch_count == 0 || touch_count > MAX_TOUCH_POINTS as u8 {
                    if touch_count > MAX_TOUCH_POINTS as u8 { error!("Invalid touch count: {}", touch_count); }
                    let _ = self.i2c_write(CST328_REG_READ_POINT_NUM, &[0]);
                    return Ok(points);
                }

                let mut touch_data = [0u8; 27];
                match self.i2c_read(CST328_REG_READ_TOUCH_DATA, &mut touch_data) {
                    Ok(_) => {
                        let _ = self.i2c_write(CST328_REG_READ_POINT_NUM, &[0]);

                        // --- PARSING LOGIC (User's current version with -1 offset) ---
                         for i in 0..touch_count as usize {
                             let num = if i > 0 { 2 } else { 0 };
                             let x_high_idx = i * 5 + 2 + num - 1;
                             let y_high_idx = i * 5 + 3 + num - 1;
                             let xy_low_idx = i * 5 + 4 + num - 1;
                             let str_idx    = i * 5 + 5 + num - 1;

                             if str_idx >= touch_data.len() {
                                 error!("Parsing index {} out of bounds (len {})", str_idx, touch_data.len());
                                 break;
                             }

                             let x_high = (touch_data[x_high_idx] as u16) << 4;
                             let x_low = ((touch_data[xy_low_idx] & 0xF0) >> 4) as u16;
                             let x = x_high | x_low;
                             let y_high = (touch_data[y_high_idx] as u16) << 4;
                             let y_low = (touch_data[xy_low_idx] & 0x0F) as u16;
                             let y = y_high | y_low;
                             let strength = touch_data[str_idx] as u16;

                             if x < self.max_x && y < self.max_y {
                                points[i] = Some(TouchPoint { x, y, strength });
                             } else {
                                 error!("Parsed coords out of range: x={} (max={}), y={} (max={})", x, self.max_x, y, self.max_y);
                             }
                         }
                        // -----------------------------------------------------------------
                        Ok(points)
                    },
                    Err(e) => {
                         error!("Error reading touch data: {:?}", e);
                         let _ = self.i2c_write(CST328_REG_READ_POINT_NUM, &[0]);
                         Err(Error::CommunicationError)
                    }
                }
            },
            Err(e) => {
                 error!("Error reading touch point count: {:?}", e);
                 Err(Error::CommunicationError)
            }
        }
    }

    /// Read a single touch point (finds first Some in array)
    pub fn read_touch_point(&mut self) -> Result<Option<TouchPoint>, Error> {
        match self.read_touch_points() {
             Ok(points) => Ok(points.iter().find_map(|&p| p)),
            Err(e) => Err(e)
        }
    }

    // --- I2C helpers ---
    fn i2c_read(&mut self, reg: u16, buffer: &mut [u8]) -> Result<(), Error> {
        let reg_bytes = [(reg >> 8) as u8, reg as u8]; // Big-endian register address
        match self.i2c.write_read(CST328_ADDR, &reg_bytes, buffer) {
            Ok(_) => Ok(()),
            Err(e) => {
                error!("I2C read error on register 0x{:04x}: {:?}", reg, defmt::Debug2Format(&e));
                Err(Error::CommunicationError)
            }
        }
    }

    fn i2c_write(&mut self, reg: u16, data: &[u8]) -> Result<(), Error> {
        let mut buffer = [0u8; 32];
        buffer[0] = (reg >> 8) as u8; // Big-endian register address
        buffer[1] = reg as u8;
        let data_len = data.len();
        if data_len > buffer.len() - 2 {
             error!("I2C write data len {} too large for buffer", data_len);
             return Err(Error::CommunicationError);
        }
        if data_len > 0 {
            buffer[2..2 + data_len].copy_from_slice(data);
        }
        match self.i2c.write(CST328_ADDR, &buffer[..2 + data_len]) {
            Ok(_) => Ok(()),
            Err(e) => {
                 error!("I2C write error on register 0x{:04x}: {:?}", reg, defmt::Debug2Format(&e));
                Err(Error::CommunicationError)
            }
        }
    }
}