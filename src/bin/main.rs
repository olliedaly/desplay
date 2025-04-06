#![no_std]
#![no_main]

// --- Imports ---
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    // Updated GPIO imports for the new pattern
    gpio::{Io, Input, InputConfig, Output, OutputConfig, Pull, Event, Level}, // Added Level
    i2c::{master::I2c, master::Config as I2cConfig},
    main,
    peripherals, // Keep peripherals
    ram, // Added ram for handler attribute
    handler, // Added handler attribute/macro
    spi::{
        master::{self, Spi},
        Mode,
    },
    time::Rate,
    Config,
};

// Logging, panic handler
use defmt::{info, error};
use panic_rtt_target as _;
use esp_backtrace as _;

// Synchronization primitives
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use critical_section::Mutex;

// --- Module imports ---
mod display;
use display::ST7789;
mod touch;
use touch::CST328;

// --- Graphics imports ---
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle},
    text::{Alignment, Text},
};

// --- Global shared state for Interrupt Pin ---
static TOUCH_PIN: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
static TOUCH_INTERRUPT_FIRED: AtomicBool = AtomicBool::new(false);

// --- Interrupt Handler ---
#[handler]
#[ram]
fn handler() {
    critical_section::with(|cs| {
        let mut touch_pin_ref = TOUCH_PIN.borrow_ref_mut(cs);
        if let Some(pin) = touch_pin_ref.as_mut() {
            if pin.is_interrupt_set() {
                 info!("Touch Interrupt Triggered!");
                 TOUCH_INTERRUPT_FIRED.store(true, Ordering::SeqCst);
                 pin.clear_interrupt();
            }
        }
    });
}


#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();
    info!("Starting ESP32-S3 Application with Touch Interrupt (Handler Pattern)");

    // --- Init ---
    let peripherals = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));
    let mut delay = Delay::new();

    // --- IO and Interrupt Handler Setup ---
    // ** CORRECTED Io initialization - only takes IO_MUX **
    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(handler);

    // --- Display Setup (using peripherals directly for pins) ---
    info!("Setting up display...");
    // ** CORRECTED Pin Access & Level Path **
    let dc = Output::new(peripherals.GPIO41, Level::High, OutputConfig::default());
    let rst_disp = Output::new(peripherals.GPIO39, Level::High, OutputConfig::default());
    let _backlight = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    let sclk = peripherals.GPIO40;
    let mosi = peripherals.GPIO45;
    let cs = peripherals.GPIO42;

    let spi = match master::Spi::new(
        peripherals.SPI2,
        master::Config::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(Mode::_0)
    ) {
        Ok(s) => s.with_sck(sclk).with_mosi(mosi).with_cs(cs),
        Err(e) => { error!("SPI display init error: {:?}", defmt::Debug2Format(&e)); loop {} }
    };
    let mut display = ST7789::new(spi, dc, rst_disp, Delay::new(), 240, 320);
    display.init();
    display.clear(Rgb565::BLACK).unwrap();
    info!("Display initialized");

    // --- Touch Setup (using peripherals directly for pins) ---
    info!("Setting up touch controller (Handler Pattern)...");
    // ** CORRECTED Pin Access **
    let i2c_sda_untyped = peripherals.GPIO1;
    let i2c_scl_untyped = peripherals.GPIO3;

    // Configure INT pin (GPIO4) as Input, pulled up
    // ** CORRECTED Pin Access **
    let mut touch_int_pin = Input::new(
        peripherals.GPIO4, // Access directly from peripherals
        InputConfig::default().with_pull(Pull::Up)
    );

    // Configure RST pin as Output
    // ** CORRECTED Pin Access & Level Path **
    let touch_rst = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());

    // Initialize I2C
    let i2c_touch = match I2c::new(
            peripherals.I2C1,
            I2cConfig::default().with_frequency(Rate::from_khz(400)),
        ) {
            Ok(i2c) => i2c.with_sda(i2c_sda_untyped).with_scl(i2c_scl_untyped),
            Err(e) => { error!("I2C touch init error: {:?}", defmt::Debug2Format(&e)); loop {} }
        };

    // Create and initialize the touch driver
    let mut touch = CST328::new(i2c_touch, touch_rst, Delay::new());
    touch.init();

    if !touch.is_initialized() {
        error!("Touch controller initialization failed!");
        Text::with_alignment(
            "TOUCH INIT FAILED!",
            display.bounding_box().center(),
            MonoTextStyle::new(&FONT_10X20, Rgb565::RED),
            Alignment::Center,
        ).draw(&mut display).ok();
        loop { Delay::new().delay_millis(1000); }
    }
    info!("Touch controller initialized");

    // --- Enable Interrupt Listening on the Pin ---
    critical_section::with(|cs| {
        touch_int_pin.listen(Event::FallingEdge); // Enable listening for falling edge
        TOUCH_PIN.borrow_ref_mut(cs).replace(touch_int_pin); // Store the pin
    });
    info!("Touch interrupt listening enabled on GPIO4");


    // --- Initial drawing ---
    let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::YELLOW);
    Text::with_alignment(
        "Touch Active!",
        Point::new(120, 20),
        text_style,
        Alignment::Center,
    )
    .draw(&mut display)
    .unwrap();

    let mut last_point: Option<Point> = None;

    // --- Main Loop (Interrupt Flag Driven) ---
    info!("Entering main loop...");
    loop {
        if TOUCH_INTERRUPT_FIRED.compare_exchange(true, false, Ordering::SeqCst, Ordering::SeqCst).is_ok() {
            match touch.read_touch_points() {
                Ok(points) => {
                    let mut processed_point = false;
                    for point_option in points.iter() {
                        if let Some(tp) = point_option {
                             let current_point = Point::new(tp.x as i32, tp.y as i32);
                             Circle::new(current_point, 3)
                                 .into_styled(PrimitiveStyle::with_fill(Rgb565::CYAN))
                                 .draw(&mut display)
                                 .unwrap();
                             if let Some(last) = last_point {
                                if last != current_point {
                                    Line::new(last, current_point)
                                        .into_styled(PrimitiveStyle::with_stroke(Rgb565::MAGENTA, 1))
                                        .draw(&mut display)
                                        .unwrap();
                                 }
                             }
                              last_point = Some(current_point);
                             processed_point = true;
                        }
                    }
                    if !processed_point {
                         last_point = None;
                    }
                }
                Err(e) => {
                    error!("Error reading touch points after interrupt: {:?}", e);
                     last_point = None;
                }
            }
        } else {
            delay.delay_millis(10);
        }
    }
}