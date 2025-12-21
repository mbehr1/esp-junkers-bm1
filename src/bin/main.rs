// Todos:
// [] add console commands for i2c read/write of pcf8570 ram
// [x] add i2c slave device emulating the junkers bm1 pcf8570 ram
// [] add homematic support
// [] add defmt_via_tcp.rs for logging to remote-defmt-srv
// [] refactor ota to standalone crate
// [] refactor defmt_via_tcp to standalone crate
// [x] add console to interact (reset, overwrite, read/write)

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, Runner, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::{
    clock::CpuClock,
    rng::Rng,
    time::{Instant, Rate},
    timer::timg::TimerGroup,
};
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{ModeConfig, WifiController, WifiDevice, WifiError};
use panic_rtt_target as _;

use esp_junkers_bm1::{
    console::console_task,
    i2c::{BOILER_STATE, i2c_task},
    ota::ota_task,
};
use smart_leds::{RGB8, SmartLedsWrite};

extern crate alloc;

// If you are okay with using a nightly compiler, you can use the macro provided by the static_cell crate: https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

#[toml_cfg::toml_config]
struct ConfigToml {
    #[default("hostnameToUseForDHCP")]
    pub hostname: &'static str,
    #[default("ssidToConnectTo")]
    pub wifi_ssid: &'static str,
    #[default("wifiPasswordToUse")]
    pub wifi_password: &'static str,
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.0.1

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // RAM 512kb for heap (64kb + 100kb) & stack
    esp_alloc::heap_allocator!(size: 100 * 1024);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    let reset_reason = esp_hal::rtc_cntl::reset_reason(esp_hal::system::Cpu::ProCpu);

    match reset_reason {
        Some(esp_hal::rtc_cntl::SocResetReason::SysRtcWdt) => {
            error!("System reset due to SysRtcWdt watchdog!");
        }
        Some(reason) => {
            info!("Reset reason: {:?}", defmt::Debug2Format(&reason));
        }
        None => {
            warn!("Reset reason: None!");
        }
    }

    // configure watchdog (Rwdt)
    let rtc = esp_hal::rtc_cntl::Rtc::new(peripherals.LPWR);
    let mut wdt = rtc.rwdt;
    wdt.set_stage_action(
        esp_hal::rtc_cntl::RwdtStage::Stage0,
        esp_hal::rtc_cntl::RwdtStageAction::Off,
    );
    wdt.set_stage_action(
        esp_hal::rtc_cntl::RwdtStage::Stage1,
        esp_hal::rtc_cntl::RwdtStageAction::Off,
    );
    wdt.set_stage_action(
        esp_hal::rtc_cntl::RwdtStage::Stage2,
        esp_hal::rtc_cntl::RwdtStageAction::Off,
    );
    wdt.set_stage_action(
        esp_hal::rtc_cntl::RwdtStage::Stage3,
        esp_hal::rtc_cntl::RwdtStageAction::ResetSystem,
    );
    wdt.set_timeout(
        esp_hal::rtc_cntl::RwdtStage::Stage3,
        esp_hal::time::Duration::from_secs(2),
    ); // 2s timeout for final stage
    wdt.enable();

    // the ws2812 led on the c6-zero controlled via spi:
    let spi = esp_hal::spi::master::Spi::new(
        peripherals.SPI2,
        esp_hal::spi::master::Config::default()
            .with_frequency(Rate::from_mhz(3)) //.with_mode(ws2812_spi::MODE),
            .with_write_bit_order(esp_hal::spi::BitOrder::MsbFirst),
    )
    .unwrap()
    .with_mosi(peripherals.GPIO8);
    let mut led = ws2812_spi::Ws2812::new(spi);

    let _ = led.write([RGB8::new(0, 0, 0x80)]); // colors are mixed... it's GRB order (weird as the ws2812 sends it properly)

    // MARK: init wifi
    let rng = Rng::new(); // peripherals.RNG);
    let net_seed = rng.random() as u64 | ((rng.random() as u64) << 32);

    //let (mut controller, interfaces) = esp_radio::wifi::new(wifi, Default::default()).unwrap();

    /*let radio_init = mk_static!(
        esp_radio::Controller,
        esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller")
    );*/
    let wifi_ctrl_config = esp_radio::wifi::Config::default().with_country_code(*b"DE");
    let (wifi_controller, interfaces) = esp_radio::wifi::new(peripherals.WIFI, wifi_ctrl_config)
        .expect("Failed to initialize Wi-Fi controller");

    let wifi_interface = interfaces.station;
    info!("Wi-Fi MAC address: {}", wifi_interface.mac_address());

    let mut dhcp_config = DhcpConfig::default();
    dhcp_config.hostname = Some(CONFIG_TOML.hostname.try_into().unwrap());

    let config = embassy_net::Config::dhcpv4(dhcp_config);
    // Init network stack
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<5>, StackResources::<5>::new()),
        net_seed,
    );

    // spawn the tasks:
    let flash = esp_storage::FlashStorage::new(peripherals.FLASH);
    let sha = esp_hal::sha::Sha::new(peripherals.SHA);
    spawner.spawn(net_task(runner)).unwrap();
    spawner.spawn(connection(wifi_controller)).unwrap();
    spawner
        .spawn(i2c_task(
            peripherals.I2C0,
            peripherals.GPIO4,
            peripherals.GPIO5,
        ))
        .unwrap();
    spawner.spawn(ota_task(stack, flash, sha)).unwrap();
    spawner.spawn(console_task(stack)).unwrap();

    info!(
        "Free heap before main loop: {} bytes",
        esp_alloc::HEAP.free()
    );
    info!("Heap stats before main loop: {:?}", esp_alloc::HEAP.stats());

    /*
    // test LP_I2C master here:
    let lp_i2c = esp_hal::i2c::lp_i2c::LpI2c::new(
        peripherals.LP_I2C0,
        esp_hal::gpio::lp_io::LowPowerOutputOpenDrain::new(peripherals.GPIO6),
        esp_hal::gpio::lp_io::LowPowerOutputOpenDrain::new(peripherals.GPIO7),
        esp_hal::time::Rate::from_khz(60),
    );
    // start the LP core:
    let mut lp_core = esp_hal::lp_core::LpCore::new(peripherals.LP_CORE);

    // load code to LP core
    let lp_core_code = esp_hal::load_lp_code!(
        "../esp-junkers-lp/target/riscv32imac-unknown-none-elf/release/esp-junkers-lp"
    );

    // start LP core
    lp_core_code.run(&mut lp_core, LpCoreWakeupSource::HpCpu, lp_i2c);
    info!("lpcore running...");
    let data_from_lp = (0x5000_2000) as *mut u32;
    let mut last_data_from_lp = 0;
    */

    // MARK: main loop
    let mut last_stack_link_state = None;
    let mut last_stack_ipv4_addr = None;

    loop {
        wdt.feed(); // feed it every main loop iteration

        // check for wifi link state changes
        let link_state = stack.is_link_up();
        if last_stack_link_state != Some(link_state) {
            info!("Network link state changed: {}", link_state);
            last_stack_link_state = Some(link_state);
        }
        let ipv4_addr = stack.config_v4();
        if last_stack_ipv4_addr != ipv4_addr {
            info!("Network IPv4 address changed: {}", ipv4_addr);
            last_stack_ipv4_addr = ipv4_addr;
        }

        // let cur_data_from_lp = unsafe { data_from_lp.read_volatile() };
        // if cur_data_from_lp != last_data_from_lp {
        //     info!("New data from lp_data {:x}", cur_data_from_lp);
        //     last_data_from_lp = cur_data_from_lp;
        // }

        // if boiler state is older than 1s, use yellow led
        // if boiler state is older than 5s, use red led
        // else use green led
        let now = Instant::now();
        let led_color = critical_section::with(|cs| {
            let bs_ref = BOILER_STATE.borrow_ref(cs);
            if let Some((_bs, timestamp)) = bs_ref.as_ref() {
                let age = now
                    .duration_since_epoch()
                    .saturating_sub(timestamp.duration_since_epoch());
                if age > esp_hal::time::Duration::from_secs(10) {
                    if age.as_secs() % 10 == 0 {
                        error!("Boiler state is stale (age: {}s)", age.as_secs());
                    }
                    RGB8::new(0, 128, 0) // red (red and green mixed?) 0,0,80 = blue
                } else if age > esp_hal::time::Duration::from_secs(3) {
                    // warn!("Boiler state is a bit stale (age: {}s)", age.as_secs());
                    RGB8::new(128, 128, 0) // yellow = red + green
                } else {
                    // info!("Boiler state is fresh (age: {}s)", age.as_secs());
                    RGB8::new(80, 0, 0) // green
                }
            } else {
                // no boiler state yet
                RGB8::new(0, 128, 0) // red
            }
        });
        // if we have no link up, blink red every second:
        let led_color = if !link_state {
            if now.duration_since_epoch().as_secs() % 2 == 0 {
                RGB8::new(0, 128, 0) // red
            } else {
                led_color
            }
        } else {
            led_color
        };
        // led_color.r = led_color.r.wrapping_add(8);
        let _ = led.write([led_color]); // RB mixed? yellow for 80,0,0 ???

        Timer::after(Duration::from_millis(1000)).await;
    }
}

// MARK: connection task
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("task 'connection' running...");
    info!("Device capabilities: {:?}", controller.capabilities());
    let mut was_connected_once = false;
    loop {
        match controller.is_connected() {
            Ok(true) => {
                // info!("Already connected to wifi");
                Timer::after(Duration::from_secs(2)).await;
                continue;
            }
            Ok(false) => {}
            Err(WifiError::Disconnected) => {
                info!("Wifi controller disconnected.");
                // fallthrough to reconnect
            }
            Err(e) => {
                warn!("Failed to get wifi connection state: {:?}", e);
                Timer::after(Duration::from_millis(3000)).await;
                continue;
            }
        }

        // not connected, check whether controller is started:
        // error codes are here: https://github.com/esp-rs/esp-wifi-sys/blob/99dd43f9992b276efff3c75467839801e7ea55c7/esp-wifi-sys/src/include/esp32c6.rs#L1291
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = if CONFIG_TOML.wifi_password.is_empty() {
                ModeConfig::Station(
                    StationConfig::default()
                        .with_ssid(CONFIG_TOML.wifi_ssid.into())
                        .with_auth_method(esp_radio::wifi::AuthMethod::None),
                )
            } else {
                ModeConfig::Station(
                    StationConfig::default()
                        .with_ssid(CONFIG_TOML.wifi_ssid.into())
                        .with_password(CONFIG_TOML.wifi_password.into()),
                )
            };
            controller.set_config(&client_config).unwrap();
            info!("Starting wifi");
            controller.start_async().await.unwrap();
            info!("Wifi started!: {:?}", controller.is_started());
            Timer::after(Duration::from_millis(1000)).await;
            info!("Wifi started!: {:?}", controller.is_started());
        }
        info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => {
                info!("Wifi connected!");
                was_connected_once = true;
            }
            Err(e) => {
                info!("Failed to connect to wifi: {:?}", e);
                let scan_config = esp_radio::wifi::scan::ScanConfig::default();
                let scan_res = controller.scan_with_config(scan_config);
                match scan_res {
                    Ok(networks) => {
                        info!("Available wifi networks:");
                        for net in networks {
                            info!("  SSID: '{}'", net.ssid.as_str(),);
                            if net.ssid.as_str() == CONFIG_TOML.wifi_ssid {
                                info!("    -> Target network found! (Auth: {:?})", net.auth_method);
                            }
                        }
                    }
                    Err(e) => {
                        warn!("Failed to scan for wifi networks: {:?}", e);
                    }
                }
                // lets stop controller here and try again:
                if !was_connected_once {
                    // TODO or if we do loop here for e.g. 1min? difficult to test...
                    info!("Stopping wifi controller to retry...");
                    if let Err(e) = controller.stop_async().await {
                        warn!("Failed to stop wifi controller: {:?}", e);
                    }
                }
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
    //info!("end connection task");
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    info!("task 'net_task' running...");
    runner.run().await;
    //info!("end net_task");
}
