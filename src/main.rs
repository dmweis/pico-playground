#![no_std]
#![no_main]

use heapless::String;
// The macro for our start-up function
use embedded_hal::pwm::SetDutyCycle;
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_reset as _;

use rp_pico::hal;
use rp_pico::hal::pac;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::{embedded_io::Write, SerialPort};

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // The default is to generate a 125 MHz system clock
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

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("picoplayground")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    // blinding led
    let _led_pin = pins.led.into_push_pull_output();

    // pwm 0
    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let pwm0 = &mut pwm_slices.pwm0;
    pwm0.set_ph_correct();
    pwm0.set_div_int(20u8); // 50 hz
    pwm0.enable();

    let gpio0 = &mut pwm0.channel_a;
    gpio0.output_to(pins.gpio0);

    let gpio1 = &mut pwm0.channel_b;
    gpio1.output_to(pins.gpio1);

    let mut motor_a = Motor::new(gpio0, gpio1);

    let pwm1 = &mut pwm_slices.pwm1;
    pwm1.set_ph_correct();
    pwm1.set_div_int(20u8); // 50 hz
    pwm1.enable();

    let gpio2 = &mut pwm1.channel_a;
    gpio2.output_to(pins.gpio2);

    let gpio3 = &mut pwm1.channel_b;
    gpio3.output_to(pins.gpio3);

    let mut motor_b = Motor::new(gpio2, gpio3);

    let mut buf = [0u8; 64];

    let mut text: String<256> = String::new();

    loop {
        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    let new_slice = &buf[..count];
                    if text
                        .push_str(core::str::from_utf8(new_slice).unwrap_or_default())
                        .is_err()
                    {
                        text.clear()
                    }

                    if let Some((start, end)) =
                        text.clone().rsplit_once(|c| char::is_ascii_whitespace(&c))
                    {
                        for segment in start.split_ascii_whitespace() {
                            if let Ok(value) = segment.parse::<i8>() {
                                _ = writeln!(&mut serial, "Value is {}", value);
                                motor_a.drive(value);
                                motor_b.drive(value);
                            } else if segment.eq_ignore_ascii_case("reset") {
                                rp2040_hal::rom_data::reset_to_usb_boot(0, 0);
                            }
                        }

                        text.clear();
                        _ = text.push_str(end);
                    }
                }
            }
        }
    }
}

struct Motor<A: SetDutyCycle, B: SetDutyCycle> {
    a: A,
    b: B,
}

impl<A: SetDutyCycle, B: SetDutyCycle> Motor<A, B> {
    fn new(mut a: A, mut b: B) -> Self {
        _ = a.set_duty_cycle_percent(0);
        _ = b.set_duty_cycle_percent(0);
        Motor { a, b }
    }

    fn drive(&mut self, drive: i8) {
        let drive = drive.clamp(-100, 100);
        if drive.is_positive() {
            _ = self.a.set_duty_cycle_percent(drive as u8);
            _ = self.b.set_duty_cycle_percent(0);
        } else {
            _ = self.a.set_duty_cycle_percent(0);
            _ = self.b.set_duty_cycle_percent(drive.unsigned_abs());
        }
    }
}
