#![no_std]
#![no_main]

use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio;
use esp_hal::main;
use esp_hal::spi::master;
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use defmt::info;
use panic_rtt_target as _;

// Import our display module
mod display;
use display::ST7789;

// Import embedded-graphics items
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle, Triangle},
    text::{Alignment, Text},
};

#[main]
fn main() -> ! {
    // Initialize defmt for logging over RTT
    rtt_target::rtt_init_defmt!();

    // Initialize the ESP32 with maximum CPU clock speed
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Configure GPIO pins
    let dc = gpio::Output::new(peripherals.GPIO41, gpio::Level::High, gpio::OutputConfig::default());
    let rst = gpio::Output::new(peripherals.GPIO39, gpio::Level::High, gpio::OutputConfig::default());
    let _backlight = gpio::Output::new(peripherals.GPIO5, gpio::Level::High, gpio::OutputConfig::default()); // Backlight on

    // Assign SPI pins (SCLK, MOSI, CS handled by SPI driver)
    let sclk = peripherals.GPIO40;
    let mosi = peripherals.GPIO45;
    let cs = peripherals.GPIO42;

    // Initialize SPI interface
    let spi = master::Spi::new(
        peripherals.SPI2,
        master::Config::default()
            .with_frequency(Rate::from_mhz(20)) // 20 MHz for faster display updates
            .with_mode(Mode::_0)                // SPI Mode 0 (common for displays)
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_cs(cs);

    // Initialize delay utility
    let delay = Delay::new();

    // Create and initialize the display (240x320 resolution for ST7789)
    let mut display = ST7789::new(spi, dc, rst, delay, 240, 320);
    display.init();

    // Clear the display with blue background
    display.clear(Rgb565::BLUE).unwrap();
    info!("Display cleared with blue background");

    // Draw a rectangle
    Rectangle::new(Point::new(20, 30), Size::new(200, 100))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(&mut display)
        .unwrap();

    // Draw a filled rectangle
    Rectangle::new(Point::new(50, 50), Size::new(50, 50))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
        .draw(&mut display)
        .unwrap();

    // Draw a circle
    Circle::new(Point::new(120, 160), 50)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
        .draw(&mut display)
        .unwrap();

    // Draw a filled circle
    Circle::new(Point::new(120, 240), 30)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::YELLOW))
        .draw(&mut display)
        .unwrap();

    // Draw a line
    Line::new(Point::new(10, 10), Point::new(230, 310))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::MAGENTA, 1))
        .draw(&mut display)
        .unwrap();

    // Draw some text
    let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    Text::with_alignment(
        "ESP32-S3",
        Point::new(120, 20),
        text_style,
        Alignment::Center,
    )
    .draw(&mut display)
    .unwrap();

    // Draw a triangle
    Triangle::new(
        Point::new(40, 270),
        Point::new(80, 220),
        Point::new(120, 270),
    )
    .into_styled(PrimitiveStyle::with_fill(Rgb565::CYAN))
    .draw(&mut display)
    .unwrap();

    // Keep the program running and log status
    let delay = Delay::new();
    loop {
        info!("Display demo running");
        delay.delay_millis(1000); // Log every second
    }
}