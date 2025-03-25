//! ST7789 display driver for ESP32-S3 with embedded-graphics support

use esp_hal::{
    delay::Delay,
    gpio,
    spi::master,
};

use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::Rectangle,
};

/// ST7789 display driver that implements embedded-graphics traits
pub struct ST7789<'a, Dm: esp_hal::DriverMode> {
    spi: master::Spi<'a, Dm>,
    dc: gpio::Output<'a>,
    rst: gpio::Output<'a>,
    delay: Delay,
    width: u16,
    height: u16,
}

impl<'a, Dm: esp_hal::DriverMode> ST7789<'a, Dm> {
    /// Create a new display instance
    pub fn new(
        spi: master::Spi<'a, Dm>,
        dc: gpio::Output<'a>,
        rst: gpio::Output<'a>,
        delay: Delay,
        width: u16,
        height: u16,
    ) -> Self {
        Self {
            spi,
            dc,
            rst,
            delay,
            width,
            height,
        }
    }

    /// Send a command to the display
    fn send_command(&mut self, command: u8) {
        self.dc.set_low(); // Command mode
        self.spi.write(&[command]).unwrap();
    }

    /// Send data to the display
    fn send_data(&mut self, data: &[u8]) {
        self.dc.set_high(); // Data mode
        self.spi.write(data).unwrap();
    }

    /// Initialize the display
    pub fn init(&mut self) {
        // Reset the display
        self.rst.set_low();
        self.delay.delay_millis(10);  // Hold reset low for 10ms
        self.rst.set_high();
        self.delay.delay_millis(100); // Wait 100ms for display to stabilize

        // Initialize the display (ST7789 sequence)
        self.send_command(0x01); // Software reset
        self.delay.delay_millis(100);
        self.send_command(0x11); // Exit sleep mode
        self.delay.delay_millis(120);
        self.send_command(0x3A); // Set pixel format
        self.send_data(&[0x55]); // 16-bit RGB565 color
        
        // Memory data access control
        self.send_command(0x36);
        self.send_data(&[0x00]); // Normal orientation, RGB color order
        
        // Fix the color inversion issue
        self.send_command(0x21); // INVOFF - Inversion OFF
        
        // Turn display on
        self.send_command(0x29); 
        self.delay.delay_millis(100);

        // Alternative: Enable inversion if they're still inverted
        self.send_command(0x21); // INVON - Inversion ON
    }

    /// Set the address window for drawing
    fn set_address_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        // Column address set
        self.send_command(0x2A);
        self.send_data(&[
            (x0 >> 8) as u8,
            (x0 & 0xFF) as u8,
            (x1 >> 8) as u8,
            (x1 & 0xFF) as u8,
        ]);

        // Row address set
        self.send_command(0x2B);
        self.send_data(&[
            (y0 >> 8) as u8,
            (y0 & 0xFF) as u8,
            (y1 >> 8) as u8,
            (y1 & 0xFF) as u8,
        ]);

        // Start writing to memory
        self.send_command(0x2C);
    }

    /// Clear the screen with a specific color
    pub fn clear(&mut self, color: Rgb565) -> Result<(), Error> {
        self.fill_solid(
            &Rectangle::new(Point::new(0, 0), Size::new(self.width.into(), self.height.into())),
            color
        )
    }
    
    /// Fill a rectangular area with a specific color
    fn fill_solid_rect(&mut self, area: &Rectangle, color: Rgb565) -> Result<(), Error> {
        let x_start = area.top_left.x.max(0) as u16;
        let y_start = area.top_left.y.max(0) as u16;
        let x_end = (area.bottom_right().unwrap().x as u16).min(self.width - 1);
        let y_end = (area.bottom_right().unwrap().y as u16).min(self.height - 1);
        
        if x_start > x_end || y_start > y_end {
            return Ok(());
        }
        
        self.set_address_window(x_start, y_start, x_end, y_end);
        
        // Convert Rgb565 to bytes
        let color_bytes = [(color.into_storage() >> 8) as u8, color.into_storage() as u8];
        
        // Calculate dimensions safely to avoid overflow
        let width = (x_end - x_start + 1) as u32;
        let height = (y_end - y_start + 1) as u32;
        
        // Send pixels in small batches to avoid buffer allocation and overflow issues
        const BATCH_SIZE: u32 = 20;
        
        // Process in row-by-row batches to avoid multiplication overflow
        for _ in 0..height {
            let mut remaining_pixels = width;
            
            while remaining_pixels > 0 {
                let batch = remaining_pixels.min(BATCH_SIZE);
                for _ in 0..batch {
                    self.send_data(&color_bytes);
                }
                remaining_pixels -= batch;
            }
        }
        
        Ok(())
    }
}

// Implement the embedded-graphics DrawTarget trait
impl<'a, Dm: esp_hal::DriverMode> DrawTarget for ST7789<'a, Dm> {
    type Color = Rgb565;
    type Error = Error;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels.into_iter() {
            // Skip pixels outside the screen bounds
            if point.x < 0 || point.y < 0 || point.x >= self.width as i32 || point.y >= self.height as i32 {
                continue;
            }
            
            let x = point.x as u16;
            let y = point.y as u16;
            
            self.set_address_window(x, y, x, y);
            
            // Convert Rgb565 to bytes
            let color_value = color.into_storage();
            self.send_data(&[(color_value >> 8) as u8, color_value as u8]);
        }
        
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_solid_rect(area, color)
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_solid(
            &Rectangle::new(Point::new(0, 0), Size::new(self.width.into(), self.height.into())),
            color
        )
    }
}

// Implement the OriginDimensions trait
impl<'a, Dm: esp_hal::DriverMode> OriginDimensions for ST7789<'a, Dm> {
    fn size(&self) -> Size {
        Size::new(self.width.into(), self.height.into())
    }
}

/// Error type for the display driver
#[derive(Debug)]
pub enum Error {
    OutOfBounds,
} 