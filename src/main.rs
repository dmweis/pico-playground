#![no_std]
#![no_main]

use embedded_hal::{digital::OutputPin, pwm::SetDutyCycle};
use postcard::accumulator::{CobsAccumulator, FeedResult};
use rp2040_hal::Timer;
use rp_pico::{
    entry,
    hal::{self, pac},
};
use serde::{Deserialize, Serialize};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::{embedded_io::Write, SerialPort};

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_reset as _;

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
struct MotorCommand {
    a: i8,
    b: i8,
    c: i8,
    d: i8,
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
struct LedCommand {
    status: bool,
}

#[derive(Serialize, Deserialize, Debug, Eq, PartialEq)]
#[allow(clippy::enum_variant_names)]
enum Command {
    ResetToUsbBoot,
    MotorCommand(MotorCommand),
    LedCommand(LedCommand),
}

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

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

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
    let mut led_pin = pins.led.into_push_pull_output();

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

    let pwm6 = &mut pwm_slices.pwm6;
    pwm6.set_ph_correct();
    pwm6.set_div_int(20u8); // 50 hz
    pwm6.enable();

    let gpio12 = &mut pwm6.channel_a;
    gpio12.output_to(pins.gpio12);

    let gpio13 = &mut pwm6.channel_b;
    gpio13.output_to(pins.gpio13);

    let mut motor_c = Motor::new(gpio12, gpio13);

    let pwm7 = &mut pwm_slices.pwm7;
    pwm7.set_ph_correct();
    pwm7.set_div_int(20u8); // 50 hz
    pwm7.enable();

    let gpio14 = &mut pwm7.channel_a;
    gpio14.output_to(pins.gpio14);

    let gpio15 = &mut pwm7.channel_b;
    gpio15.output_to(pins.gpio15);

    let mut motor_d = Motor::new(gpio14, gpio15);

    let mut raw_buf = [0u8; 64];
    let mut cobs_buf: CobsAccumulator<256> = CobsAccumulator::new();

    let mut last_command = timer.get_counter();
    // Timeout in microseconds (1 second -> 200 millis)
    let timeout = 1_000_000 / 5;

    loop {
        // check timeout
        let now = timer.get_counter();
        if now.ticks() - last_command.ticks() > timeout {
            motor_a.drive(0);
            motor_b.drive(0);
            motor_c.drive(0);
            motor_d.drive(0);
        }
        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            match serial.read(&mut raw_buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    let buf = &raw_buf[..count];
                    let mut window = buf;

                    'cobs: while !window.is_empty() {
                        window = match cobs_buf.feed::<Command>(window) {
                            FeedResult::Consumed => break 'cobs,
                            FeedResult::OverFull(new_wind) => new_wind,
                            FeedResult::DeserError(new_wind) => new_wind,
                            FeedResult::Success { data, remaining } => {
                                match data {
                                    Command::ResetToUsbBoot => {
                                        rp2040_hal::rom_data::reset_to_usb_boot(0, 0);
                                    }
                                    Command::MotorCommand(motor_command) => {
                                        motor_a.drive(motor_command.a);
                                        motor_b.drive(motor_command.b);
                                        motor_c.drive(motor_command.c);
                                        motor_d.drive(motor_command.d);

                                        last_command = timer.get_counter();

                                        _ = writeln!(
                                            &mut serial,
                                            "Motor command: {:?}",
                                            motor_command
                                        );
                                    }
                                    Command::LedCommand(led_command) => {
                                        if led_command.status {
                                            led_pin.set_high().unwrap();
                                        } else {
                                            led_pin.set_low().unwrap();
                                        }
                                    }
                                }
                                remaining
                            }
                        };
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
