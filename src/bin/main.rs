#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::fmt::Error;

use embassy_crypto_rustcrypto as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcChannel, AdcConfig, AdcPin, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, Level, Pin};
use esp_hal::peripherals::ADC1;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{Async, rng};
use log::{info, warn};
use trouble_host::{gatt, prelude::*};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use esp_hal::peripherals::{GPIO0, GPIO1};

static HID_REPORTS: Channel<CriticalSectionRawMutex, [u8; 4], 4> = Channel::new();

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// Standard USB HID report map
const REPORT_MAP: [u8; 54] = [
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x02, // USAGE (Mouse)
    0xa1, 0x01, // COLLECTION (Application)
    0x85, 0x01, //   Report ID 1
    0x09, 0x01, //   USAGE (Pointer)
    0xa1, 0x00, //   COLLECTION (Physical)
    0x05, 0x09, //     USAGE_PAGE (Button)
    0x19, 0x01, //     USAGE_MINIMUM (Button 1)
    0x29, 0x03, //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00, //     LOGICAL_MINIMUM (0)
    0x25, 0x01, //     LOGICAL_MAXIMUM (1)
    0x95, 0x03, //     REPORT_COUNT (3)
    0x75, 0x01, //     REPORT_SIZE (1)
    0x81, 0x02, //     INPUT (Data,Var,Abs)
    0x95, 0x01, //     REPORT_COUNT (1)
    0x75, 0x05, //     REPORT_SIZE (5)
    0x81, 0x03, //     INPUT (Cnst,Var,Abs)
    0x05, 0x01, //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30, //     USAGE (X)
    0x09, 0x31, //     USAGE (Y)
    0x09, 0x38, //     USAGE (Wheel)          <-- added
    0x15, 0x81, //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f, //     LOGICAL_MAXIMUM (127)
    0x75, 0x08, //     REPORT_SIZE (8)
    0x95, 0x03, //     REPORT_COUNT (3)       <-- was 0x02
    0x81, 0x06, //     INPUT (Data,Var,Rel)
    0xc0, //   END_COLLECTION
    0xc0, // END_COLLECTION
];

// we just want one connection for our controller
const CONNECTIONS_MAX: usize = 1;

// 2 max L2CAP channels
const L2CAP_CHANNELS_MAX: usize = 4;

// GATT server definition
#[gatt_server]
struct Server {
    battery_service: BatteryService,
    hid_service: HidService,
}

#[gatt_service(uuid = service::HUMAN_INTERFACE_DEVICE)]
struct HidService {
    #[characteristic(uuid = characteristic::HID_INFORMATION, read, value = [0x11, 0x01, 0x00, 0x02])]
    hid_info: [u8; 4],
    #[characteristic(uuid = characteristic::REPORT_MAP, read, value = REPORT_MAP)]
    report_map: [u8; 54],
    #[characteristic(uuid = characteristic::HID_CONTROL_POINT, write_without_response)]
    control_point: u8,
    #[descriptor(uuid = descriptors::REPORT_REFERENCE, read, value = [1,1])]
    #[characteristic(uuid = characteristic::REPORT, read, notify)]
    input_report: [u8; 4],
    #[characteristic(uuid = characteristic::PROTOCOL_MODE, read, write_without_response, value = 1)]
    protocol: u8,
}

#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, name = "hello", read, value = "Battery Level", type = &'static str)]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 100)]
    level: u8,
    #[characteristic(uuid = "408813df-5dd4-1f87-ec11-cdb001100000", write, read, notify)]
    status: (),
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    let _ = spawner;

    // BLE setup
    let bleconnector = esp_radio::ble::controller::BleConnector::new(
        peripherals.BT,
        esp_radio::ble::Config::default(),
    )
    .unwrap();
    let controller: ExternalController<_, 20> = ExternalController::new(bleconnector);

    // ADC module setup
    let mut adc1_config = AdcConfig::new();

    // ADC channel configuration
    let pin1 = adc1_config.enable_pin(peripherals.GPIO0, Attenuation::_11dB);
    let pin2 = adc1_config.enable_pin(peripherals.GPIO1, Attenuation::_11dB);

    // ADC start
    let adc1 = Adc::new(peripherals.ADC1, adc1_config).into_async();

    // pull up input config
    let input_config = esp_hal::gpio::InputConfig::default().with_pull(esp_hal::gpio::Pull::Up);

    // initialize left click
    let left_click = esp_hal::gpio::Input::new(peripherals.GPIO21, input_config);
    // initialize right click
    let right_click = esp_hal::gpio::Input::new(peripherals.GPIO20, input_config);
    // scroll click
    let scroll_mode = esp_hal::gpio::Input::new(peripherals.GPIO10, input_config);

    spawner.spawn(input_task(adc1, pin1, pin2, left_click, right_click, scroll_mode).unwrap());

    run(controller).await;
    loop {}
}

#[allow(clippy::large_stack_frames)]
pub async fn run<C>(controller: C)
where
    C: Controller,
{
    // RNG setup
    let mut buffer = [0u8; 6];
    let rng = rng::Rng::new();
    buffer[5] |= 0b1100_0000;
    rng.read(&mut buffer);

    // let address: Address = Address::random(buffer);
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    info!("Address is {}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .set_io_capabilities(IoCapabilities::NoInputNoOutput)
        .build();

    let runner = stack.runner();
    let mut peripheral = stack.peripheral();

    info!("Starting GATT Service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Rouse",
        appearance: &appearance::human_interface_device::MOUSE,
    }))
    .unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise("Rouse", &mut peripheral, &server).await {
                Ok(conn) => {
                    let a = gatt_events_task(&server, &conn);
                    let b = custom_task(&server, &conn);

                    select(a, b).await;
                }
                Err(e) => {
                    #[cfg(feature = "defmt")]
                    let e = defmt::Debug2Format(&e);
                    panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;
}

async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) -> Result<(), Error> {
    let level = server.battery_service.level;
    let status_handle = server.battery_service.status.handle;
    let mut status = false;
    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::Gatt { event } => {
                let reply = match event {
                    GattEvent::Read(event) => {
                        if event.handle() == level.handle {
                            let value = conn.get(&level);
                            info!("[gatt] Read Event to Level Characteristic: {:?}", value);
                            event.accept()
                        } else if event.handle() == status_handle {
                            event.accept_unprocessed(&status)
                        } else {
                            event.accept()
                        }
                    }
                    GattEvent::Write(event) => {
                        if event.handle() == level.handle {
                            event.with_data(|offset, data| {
                                info!(
                                    "[gatt] Write Event to Level Characteristic at {}: {:?}",
                                    offset, data
                                )
                            });
                            event.accept()
                        } else if event.handle() == status_handle {
                            match event.validate(1, 1) {
                                Ok(()) => {
                                    event.with_data(|offset, data| {
                                        if data.len() == 1 {
                                            // If data.len() is 1, offset must be 0 or else validate would have errored
                                            assert!(offset == 0);
                                            status = data[0] != 0;
                                        }
                                    });
                                    event.accept_unprocessed()
                                }
                                Err(err) => event.reject(err),
                            }
                        } else {
                            event.accept()
                        }
                    }
                    _ => event.accept(),
                };
                // This step is also performed at drop(), but writing it explicitly is necessary
                // in order to ensure reply is sent.
                match reply {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[gatt] error sending response: {:?}", e),
                };
            }
            _ => {} // ignore other Gatt Connection Events
        }
    };
    info!("[gatt] disconnected: {:?}", reason);
    Ok(())
}

// Advertise function
async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::IncompleteServiceUuids16(&[[0x12, 0x18]]),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[adv] connection established");
    Ok(conn)
}

// Ble task that must run alongside other BLE tasks.
async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            #[cfg(feature = "defmt")]
            let e = defmt::Debug2Format(&e);
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

/// Example task to use the BLE notifier interface.
/// This task will notify the connected central of a counter value every 2 seconds.
/// It will also read the RSSI value every 2 seconds.
/// and will stop when the connection is closed by the central or an error occurs.
async fn custom_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) -> Result<(), Error> {
    let input = server.hid_service.input_report;
    loop {
        let report = HID_REPORTS.receive().await;
        if let Err(e) = input.notify(conn, &report, true).await {
            warn!("[hid] notify skipped: {:?}", e);
        }
    }
}

#[embassy_executor::task]
async fn input_task(
    mut adc: Adc<'static, ADC1<'static>, Async>,
    mut pin_x: AdcPin<GPIO0<'static>, ADC1<'static>>,
    mut pin_y: AdcPin<GPIO1<'static>, ADC1<'static>>,
    left_click: Input<'static>,
    right_click: Input<'static>,
    scroll_pin: Input<'static>,
) {
    // Deadzone setup -> poll 5 inputs upon startup as center on that
    let mut dx: u32 = 0;
    let mut dy: u32 = 0;

    for _i in 0..5 {
        dx += adc.read_oneshot(&mut pin_x).await as u32;
    }
    dx /= 5;

    for _i in 0..5 {
        dy += adc.read_oneshot(&mut pin_y).await as u32;
    }
    dy /= 5;

    let mut prev_buf: u8 = 0x00;
    let mut prev_state: bool = false;
    let mut scroll_mode: bool = false;
    let mut alternate = 0;

    loop {
        let mut buf: u8 = 0x00;
        // set bit 1 according to HID map
        if left_click.is_low() {
            buf |= 0b0000_0001;
            info!("Left Click");
        }
        // set bit 1 according to HID map
        if right_click.is_low() {
            buf |= 0b0000_0010;
            info!("Right Click");
        }

        // Outline of this section. Button will be read as low for at least a few cycles. So, we
        // need to check if it was low the previous loop, and if it was low, ignore it. If it was
        // high the previous loop, we want to toggle our scroll mode
        // if pin is pressed, and previous state is pressed (also true)
        if scroll_pin.is_low() && prev_state {
        }
        // if button is pressed and previous state is unpressed, toggle scroll mode and set previous
        // state to pressed
        else if scroll_pin.is_low() && !prev_state {
            scroll_mode = !scroll_mode;
            prev_state = true;
        }
        // If scroll pin is not pressed, set previous state to floating
        else {
            prev_state = false;
        }

        // .await allows adc_read to borrow adc1 twice, datax holds onto it until its done, then lets
        // datay have it
        let data_x = adc.read_oneshot(&mut pin_x).await;
        let data_y = adc.read_oneshot(&mut pin_y).await;

        // Deadzones
        let tx_x = ((data_x as i32 - dx as i32) / 64).clamp(-127, 127) as i8;
        let tx_y = ((data_y as i32 - dy as i32) / 64).clamp(-127, 127) as i8;

        // Transmit variables
        let tx_x = if tx_x.abs() > 5 { tx_x - 5 * tx_x.signum() } else { 0 };
        let tx_y = if tx_y.abs() > 5 { tx_y - 5 * tx_y.signum() } else { 0 };

        if scroll_mode {
            if (alternate % 300) == 1 {
                if tx_x != 0 || tx_y != 0 || buf != prev_buf {
                    let _ = HID_REPORTS.try_send([buf, 0, 0, (tx_y.clamp(-1, 1)) as u8]);
                }
                alternate = 0;
            } else {
                alternate += 1;
            }
        } else {
            if tx_x != 0 || tx_y != 0 || buf != prev_buf {
                let _ = HID_REPORTS.try_send([buf, tx_x as u8, tx_y as u8, 0]);
            }
        }

        prev_buf = buf;
        Timer::after_millis(10).await;
    }
}
