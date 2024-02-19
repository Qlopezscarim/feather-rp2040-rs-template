#![no_std]  //not using the standard 
#![no_main] //not using typical function enter

/**** low-level imports *****/
 use panic_halt as _;
use cortex_m_rt::entry;
use embedded_hal::{
    digital::v2::{OutputPin},
};

/***** board-specific imports *****/
use adafruit_feather_rp2040::hal as hal;
use hal::{
    i2c::I2C,
    gpio::{FunctionI2C,Pin,PullUp},
    //pac::interrupt,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    watchdog::Watchdog,
    Sio,
};
use adafruit_feather_rp2040::{
    Pins, XOSC_CRYSTAL_FREQ,
};

//use crate::hal::gpio::FunctionI2C; //-----try this if we need to force i2c later -----------------------

use embedded_hal::blocking::i2c::{Write,WriteRead};
use hal::fugit::RateExtU32;
//use ws2812_pio::Ws2812;
use smart_leds::{RGB8, SmartLedsWrite};
//use ws2812_pio::Ws2812;
//use fugit::RateExtU32;
//use rp2040_hal::{i2c::I2C, gpio::Pins, pac, Sio};
//let mut peripherals = pac::Peripherals::take().unwrap();
//let sio = Sio::new(peripherals.SIO);
//let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);


#[entry]
fn main() -> ! {
    // Grab the singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    // Init the watchdog timer, to pass into the clock init
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    ).ok().unwrap();
    
    // Setup USB
    /*let usb = unsafe {
        USB_BUS = Some(UsbBusAllocator::new(hal::usb::UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        )));
        USB_MANAGER = Some(UsbManager::new(USB_BUS.as_ref().unwrap()));
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
        USB_MANAGER.as_mut().unwrap()
    };*/

    // initialize the Single Cycle IO
    let sio = Sio::new(pac.SIO);
    // initialize the pins to default state
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut timer = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut led_pin = pins.d13.into_push_pull_output();

    let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.sda.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.scl.reconfigure();
//    let sda_pin: Pin<FunctionI2C, PullUp> = pins.sda.reconfigure();
//    let scl_pin: Pin<FunctionI2C, PullUp> = pins.scl.reconfigure();

    let mut i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        //pins.sda.into_function(),
        //pins.scl.into_function(),
        //sda_pin.into_function(),//pins.sda.into_function(), //SDA pin is 2 SCl is 3
        //scl_pin.into_function(),//pins.scl.into_function(),
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    /*
    Loop Section
    */
    let delay: u32 = 500;   // loop delay in ms
    //let mut n: u32 = 0;
    i2c.write(0x18u8, &[0x20,0b10010011]).unwrap();
    i2c.write(0x18u8, &[0x23,0x80]).unwrap();
    i2c.write(0x18u8, &[0xC0,0xC0]).unwrap();
    let mut data: [u8; 1] = [0; 1]; //recieved data
    //i2c.write(0x18u8, &[0x23,0x80]).unwrap();
    //i2c.write()
    loop {
        //write!(usb, "starting loop number {:?}\r\n", n).unwrap();
        i2c.write_read(0x18u8,&[0x2A],&mut data).unwrap(); //this very likely works
        led_pin.set_low().unwrap();
        timer.delay_ms(delay as u32);
        led_pin.set_high().unwrap();
        timer.delay_ms(delay as u32);
        //n = n + 1;
    }

}
