#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use rp_pico as bsp;
use rp_pico::hal;

// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2)
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut xpin = pins.gpio0.into_push_pull_output();
    let mut ypin = pins.gpio1.into_push_pull_output();
    let mut zpin = pins.gpio2.into_push_pull_output();
    let apin = pins.gpio3.into_pull_up_input();
    let bpin = pins.gpio4.into_pull_up_input();
    let cpin = pins.gpio5.into_pull_up_input();
    let dpin = pins.gpio6.into_pull_up_input();

    let mut fst_connect = false;

    loop {
        if !fst_connect && timer.get_counter() >= 2_000_000 {
            fst_connect = true;
            serial.write(b"Hello, world").unwrap();
        }

        usb_dev.poll(&mut [&mut serial]);

        xpin.set_low().unwrap();
        ypin.set_high().unwrap();
        zpin.set_high().unwrap();
        let xkey = (apin.is_low().unwrap() as u8) << 3
            | (bpin.is_low().unwrap() as u8) << 2
            | (cpin.is_low().unwrap() as u8) << 1
            | (dpin.is_low().unwrap() as u8) << 0;

        xpin.set_high().unwrap();
        ypin.set_low().unwrap();
        zpin.set_high().unwrap();
        let ykey = (apin.is_low().unwrap() as u8) << 3
            | (bpin.is_low().unwrap() as u8) << 2
            | (cpin.is_low().unwrap() as u8) << 1
            | (dpin.is_low().unwrap() as u8) << 0;

        xpin.set_high().unwrap();
        ypin.set_high().unwrap();
        zpin.set_low().unwrap();
        let zkey = (apin.is_low().unwrap() as u8) << 3
            | (bpin.is_low().unwrap() as u8) << 2
            | (cpin.is_low().unwrap() as u8) << 1
            | (dpin.is_low().unwrap() as u8) << 0;

        // serial.write(b"Hello, world").unwrap();
        // usb_dev.poll(&mut [&mut serial]);

        if xkey != 0 {
            if xkey == 8 {
                serial.write(b"1\r\n").unwrap();
            } else if xkey == 4 {
                serial.write(b"4\r\n").unwrap();
            } else if xkey == 2 {
                serial.write(b"7\r\n").unwrap();
            } else if xkey == 1 {
                serial.write(b"*\r\n").unwrap();
            }
        } else if ykey != 0 {
            if ykey == 8 {
                serial.write(b"2\r\n").unwrap();
            } else if ykey == 4 {
                serial.write(b"5\r\n").unwrap();
            } else if ykey == 2 {
                serial.write(b"8\r\n").unwrap();
            } else if ykey == 1 {
                serial.write(b"0\r\n").unwrap();
            }
        } else if zkey != 0 {
            if zkey == 8 {
                serial.write(b"3\r\n").unwrap();
            } else if zkey == 4 {
                serial.write(b"6\r\n").unwrap();
            } else if zkey == 2 {
                serial.write(b"9\r\n").unwrap();
            } else if zkey == 1 {
                serial.write(b"#\r\n").unwrap();
            }
        }

        delay.delay_ms(100);
    }
}

// End of file
