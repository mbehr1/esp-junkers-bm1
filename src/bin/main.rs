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
use esp_hal::{clock::CpuClock, rng::Rng, timer::timg::TimerGroup};
use esp_radio::wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice, WifiError};
use panic_rtt_target as _;

use esp_junkers_bm1::ota::ota_task;

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

    // MARK: init wifi
    let radio_init = mk_static!(
        esp_radio::Controller,
        esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller")
    );
    let (wifi_controller, interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let wifi_interface = interfaces.sta;
    info!("Wi-Fi MAC address: {}", wifi_interface.mac_address());

    let rng = Rng::new(); // peripherals.RNG);
    let net_seed = rng.random() as u64 | ((rng.random() as u64) << 32);

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

    // spawn the OTA task:
    let flash = esp_storage::FlashStorage::new(peripherals.FLASH);
    let sha = esp_hal::sha::Sha::new(peripherals.SHA);
    spawner.spawn(connection(wifi_controller)).unwrap();
    spawner.spawn(net_task(runner)).unwrap();
    spawner.spawn(ota_task(stack, flash, sha)).unwrap();

    info!(
        "Free heap before main loop: {} bytes",
        esp_alloc::HEAP.free()
    );
    info!("Heap stats before main loop: {:?}", esp_alloc::HEAP.stats());

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

        Timer::after(Duration::from_secs(1)).await;
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
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(CONFIG_TOML.wifi_ssid.into())
                    .with_password(CONFIG_TOML.wifi_password.into()),
            );
            controller.set_config(&client_config).unwrap();
            info!("Starting wifi");
            controller.start_async().await.unwrap();
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
