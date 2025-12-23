// Todos:
// [] add regulator for remote target values
// [] add console commands for i2c read/write of pcf8570 ram
// [x] add i2c slave device emulating the junkers bm1 pcf8570 ram
// [x] add homematic support
// [x] add defmt_via_tcp.rs for logging to remote-defmt-srv
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

use core::cell::RefCell;
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, IpAddress, IpEndpoint, Runner, Stack, StackResources};
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
    defmt_via_tcp::{self, log_serve_task},
    homematic::{
        HmIpGetHostResponse, TIMESTAMP_LAST_HMIP_UPDATE, json_process_device, json_process_home,
        single_https_request, websocket_connection,
    },
    i2c::{BOILER_STATE, i2c_task},
    ota::ota_task,
};
use jiff::Timestamp;
use reqwless::client::HttpClient;
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

    // HMIP config:
    #[default("homematicip-python")]
    pub hmip_application_identifier: &'static str,
    #[default("1.0")]
    pub hmip_application_version: &'static str,
    #[default("none")]
    pub hmip_device_manufacturer: &'static str,
    #[default("Computer")]
    pub hmip_device_type: &'static str,
    #[default("Darwin")]
    pub hmip_os_type: &'static str,
    #[default("25.0.0")]
    pub hmip_os_version: &'static str,
    #[default("https://lookup.homematic.com:48335/getHost")]
    pub hmip_lookup_host: &'static str,
    #[default("23dig-accpointid")]
    pub hmip_accesspoint_id: &'static str,
    #[default("61dig-authtoken")]
    pub hmip_authtoken: &'static str,
    #[default("128dig-clientauth")]
    pub hmip_clientauth: &'static str,

    // Log server config:
    #[default("")] // use the ip addr here (not a hostname, empty = disabled)
    pub log_server_ip: &'static str,
    #[default(65455)]
    pub log_server_port: u16,
}

static mut RNG_INSTANCE: Option<Rng> = None;

#[unsafe(no_mangle)]
extern "C" fn random() -> core::ffi::c_ulong {
    unsafe {
        if let Some(ref mut rng) = RNG_INSTANCE {
            rng.random() as core::ffi::c_ulong
        } else {
            warn!("RNG_INSTANCE not initialized!");
            0
        }
    }
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.0.1

    let log_consumer = defmt_via_tcp::init();

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
    unsafe {
        RNG_INSTANCE = Some(rng);
    }

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
        mk_static!(StackResources<6>, StackResources::<6>::new()),
        net_seed,
    );

    // spawn the tasks:
    let flash = esp_storage::FlashStorage::new(peripherals.FLASH);
    //let sha = esp_hal::sha::Sha::new(peripherals.SHA);
    spawner.spawn(net_task(runner)).unwrap();
    spawner.spawn(connection(wifi_controller)).unwrap();

    // Log server config:
    let log_server_address: Result<IpAddress, _> = CONFIG_TOML.log_server_ip.parse();
    if let Ok(log_server_address) = log_server_address {
        let log_server_endpoint = IpEndpoint::new(log_server_address, CONFIG_TOML.log_server_port);
        info!("Log server configured at {}", log_server_endpoint);
        spawner
            .spawn(log_serve_task(stack, log_consumer, log_server_endpoint))
            .unwrap();
    } else {
        info!("No valid log server IP address configured, logging disabled");
    }

    spawner
        .spawn(i2c_task(
            peripherals.I2C0,
            peripherals.GPIO4,
            peripherals.GPIO5,
            peripherals.GPIO0,
        ))
        .unwrap();
    spawner.spawn(ota_task(stack, flash)).unwrap();
    spawner.spawn(console_task(stack)).unwrap();
    spawner
        .spawn(cloud_connection_task(
            stack,
            peripherals.SHA,
            peripherals.RSA,
        ))
        .unwrap();

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

        // cyclic regulator trigger
        esp_junkers_bm1::regulator::regulator_tick();

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

// MARK: homematic cloud connection task
#[embassy_executor::task]
async fn cloud_connection_task(
    stack: Stack<'static>,
    sha: esp_hal::peripherals::SHA<'static>,
    rsa: esp_hal::peripherals::RSA<'static>,
) {
    info!("task 'cloud_connection_task' running...");

    // determine free heap before TLS init
    info!("Free heap before TLS: {} bytes", esp_alloc::HEAP.free());
    info!("Heap stats before TLS: {:?}", esp_alloc::HEAP.stats());

    let mut tls = esp_mbedtls::Tls::new(sha).unwrap().with_hardware_rsa(rsa);
    tls.set_debug(5);
    //info!("TLS self-test(SHA384) result: {:?}", tls.self_test(esp_mbedtls::TlsTest::Sha384, true));
    //info!("TLS self-test(AES) result: {:?}", tls.self_test(esp_mbedtls::TlsTest::Aes, true));
    //info!("ssl_config_check result: {:x}", tls.ssl_config_check());
    let tls_ref = tls.reference();

    use embassy_net::tcp::client::{TcpClient, TcpClientState};
    let tcp_state = mk_static!(TcpClientState::<1, 16384, 16640>, TcpClientState::new());

    loop {
        // Wait for network to be ready
        loop {
            if stack.is_link_up()
                && let Some(config) = stack.config_v4()
            {
                info!("CC: network ready with IP: {:?}", config.address);
                break;
            }
            Timer::after(Duration::from_millis(500)).await;
        }
        info!("Network is ready, starting cloud connection...");

        // connect to homematic ip cloud and get data from there
        let config = reqwless::client::TlsConfig::new(
            reqwless::TlsVersion::Tls1_2,
            reqwless::Certificates::default(),
            tls_ref,
        );

        let tcp_client = TcpClient::new(stack, tcp_state);
        let dns_socket = embassy_net::dns::DnsSocket::new(stack);

        let mut client = HttpClient::new_with_tls(&tcp_client, &dns_socket, config);
        info!(
            "HTTP client with TLS initialized. Free heap after TLS: {} bytes",
            esp_alloc::HEAP.free()
        );
        info!("Heap stats after TLS: {:?}", esp_alloc::HEAP.stats());

        let url_rest: RefCell<Option<heapless::String<64>>> = RefCell::new(None);
        let url_websocket: RefCell<Option<heapless::String<64>>> = RefCell::new(None);

        let process_gethost_response = |code: reqwless::response::StatusCode, body: &mut [u8]| {
            info!("Processing getHost response with status code {}:", code);
            if !matches!(code.0, 200..=299) {
                warn!("getHost response returned error code: {}", code);
                if let Ok(body_str) = core::str::from_utf8(body) {
                    info!("Response body: {}", body_str);
                }
            }
            // todo: unescape needed? can host names contain json escaped chars?
            match serde_json_core::from_slice::<HmIpGetHostResponse>(body) {
                Ok((resp, _consumed)) => {
                    //info!("Parsed JSON response, consumed {} bytes", consumed);
                    info!("urlREST: {}", resp.url_rest);
                    *url_rest.borrow_mut() = heapless::String::try_from(resp.url_rest).ok();
                    info!("urlWebSocket: {}", resp.url_websocket);
                    *url_websocket.borrow_mut() =
                        heapless::String::try_from(resp.url_websocket).ok();
                    // replace wss: with https:
                    if let Some(ref mut url_ws) = *url_websocket.borrow_mut()
                        && url_ws.starts_with("wss:")
                    {
                        let https_url = heapless::format!(64; "https:{}", &url_ws.as_str()[4..])
                            .unwrap_or_default();
                        *url_ws = https_url;
                        info!("Converted urlWebSocket to HTTPS URL: {}", url_ws);
                    }
                }
                Err(err) => {
                    warn!(
                        "Failed to parse JSON response: {}",
                        defmt::Debug2Format(&err)
                    );
                }
            };
        };

        let client_characteristics = heapless::format!(400; "{{\"clientCharacteristics\": {{\"apiVersion\": \"10\", \"applicationIdentifier\": \"{}\", \"applicationVersion\": \"{}\", \"deviceManufacturer\": \"{}\", \"deviceType\": \"Computer\", \"language\": \"de_DE\", \"osType\": \"{}\", \"osVersion\": \"{}\"}}, \"id\": \"{}\"}}",
        CONFIG_TOML.hmip_application_identifier,
        CONFIG_TOML.hmip_application_version,
        CONFIG_TOML.hmip_device_manufacturer,
        CONFIG_TOML.hmip_os_type,
        CONFIG_TOML.hmip_os_version,
        CONFIG_TOML.hmip_accesspoint_id).unwrap_or_default();

        let get_host_res = single_https_request(
            reqwless::request::Method::POST,
            CONFIG_TOML.hmip_lookup_host,
            &[
                ("Content-Type", "application/json"),
                ("Accept", "application/json"),
            ],
            client_characteristics.as_bytes(),
            process_gethost_response,
            &mut client,
        )
        .await;
        if get_host_res.is_err() {
            warn!("Error during getHost request: {:?}", get_host_res.err());
            // reset URLs
            *url_rest.borrow_mut() = None;
            *url_websocket.borrow_mut() = None;
        }

        // do we have a rest url? then open a session to that one:
        let url_rest_clone = url_rest.borrow().clone();
        if let Some(mut url) = url_rest_clone {
            info!("Got REST URL from getHost: {}", url);
            let process_rest_response = |code, body: &mut [u8]| {
                info!(
                    "Processing REST response with status code {}:, body.len(): {}",
                    code,
                    body.len()
                );
                /*if let Ok(body_str) = core::str::from_utf8(body) {
                    info!("Response body: {}", body_str);
                }*/
                // use core_json streaming deserializer
                if let Ok(ref mut des) =
                    core_json::Deserializer::<&[u8], core_json::ConstStack<20>>::new(&body[..])
                {
                    if let Ok(mut val) = des.value() {
                        if let Ok(core_json::Type::Object) = val.kind()
                            && let Ok(mut fields) = val.fields()
                        {
                            loop {
                                match fields.next() {
                                    Some(Ok(mut field)) => {
                                        // collect field key:
                                        let key: heapless::String<200> =
                                            field.key().filter_map(|k| k.ok()).collect();
                                        // info!("Field key: {}", key);
                                        match key.as_ref() {
                                            "home" => {
                                                info!("Processing 'home' object");
                                                let _ = json_process_home(field);
                                            }
                                            "devices" => {
                                                // info!("Processing 'devices':");
                                                // iterate over devices: (object keys are device ids)
                                                if let Ok(mut devices) = field.value().fields() {
                                                    while let Some(Ok(device)) = devices.next() {
                                                        let _ = json_process_device(device);
                                                    }
                                                }
                                            }
                                            "clients" | "groups" => {}
                                            _ => {
                                                // clients, groups
                                                warn!("Ignoring field key: {}", key);
                                            }
                                        }
                                    }
                                    Some(Err(e)) => {
                                        warn!(
                                            "Error iterating field: {:?}",
                                            defmt::Debug2Format(&e)
                                        );
                                        // dont break;
                                    }
                                    None => {
                                        break;
                                    }
                                }
                            }
                        }
                        // iterate over the fields: (expect home, groups, devices, clients)
                    } else {
                        warn!("Failed to parse first JSON value");
                    }
                } else {
                    warn!("Failed to create core_json deserializer");
                }
            };

            // add /hmip/home/getCurrentState
            url.push_str("/hmip/home/getCurrentState").unwrap();
            info!("Full REST URL for getCurrentState: {}", url);
            let _rest_res = single_https_request(
                reqwless::request::Method::POST,
                &url,
                &[
                    ("Content-Type", "application/json"), // ("Accept", "application/json"),
                    ("VERSION", "12"), // todo is 12 from the response from getHost?
                    ("AUTHTOKEN", CONFIG_TOML.hmip_authtoken),
                    ("CLIENTAUTH", CONFIG_TOML.hmip_clientauth),
                    ("ACCESSPOINT-ID", CONFIG_TOML.hmip_accesspoint_id),
                ],
                client_characteristics.as_bytes(),
                process_rest_response,
                &mut client,
            )
            .await;
            if _rest_res.is_err() {
                warn!(
                    "Error during getCurrentState request: {:?}",
                    _rest_res.err()
                );
                // continuing anyway to try websocket EVENT updates only... ? TODO!
            }
        } else {
            warn!("No REST URL obtained from getHost, cannot proceed");
        }

        let url_websocket_clone: Option<alloc::string::String> =
            url_websocket.borrow().as_ref().map(|s| s.as_str().into());
        if let Some(url_websocket) = url_websocket_clone {
            let process_binary_cb = |data: &mut [u8]| {
                //info!("Received websocket binary data, len: {}", data.len());
                if let Ok(ref mut des) =
                    core_json::Deserializer::<&[u8], core_json::ConstStack<20>>::new(&data[..])
                {
                    if let Ok(val) = des.value() {
                        if let Ok(mut fields) = val.fields() {
                            while let Some(Ok(mut field)) = fields.next() {
                                let key: heapless::String<64> =
                                    field.key().filter_map(|k| k.ok()).collect();
                                match key.as_ref() {
                                    "events" => {
                                        //info!("Processing 'events' object");
                                        if let Ok(mut events) = field.value().fields() {
                                            while let Some(Ok(event)) = events.next() {
                                                /*let event_key: heapless::String<10> =
                                                    event.key().filter_map(|k| k.ok()).collect();
                                                info!(" Processing event key: {}", event_key);*/
                                                if let Ok(mut event_fields) = event.value().fields()
                                                {
                                                    while let Some(Ok(mut event_field)) =
                                                        event_fields.next()
                                                    {
                                                        let event_field_key: heapless::String<100> =
                                                            event_field
                                                                .key()
                                                                .filter_map(|k| k.ok())
                                                                .collect();
                                                        match event_field_key.as_ref() {
                                                            "device" => {
                                                                let _ = json_process_device(
                                                                    event_field,
                                                                ); // TODO only if pushEventType is "DEVICE_CHANGED"? (assuming this comes always first...)
                                                            }
                                                            "home" => {
                                                                let _ =
                                                                    json_process_home(event_field);
                                                            }
                                                            "pushEventType" | "group" => {
                                                                // GROUP_CHANGED or DEVICE_CHANGED
                                                                // we derive indirectly by using only the "device"
                                                            }
                                                            _ => {
                                                                info!(
                                                                    "  Ignoring event field key: {}",
                                                                    event_field_key
                                                                );
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    "timestamp" => {
                                        let timestamp_ms =
                                            field.value().to_number().ok().and_then(|n| n.i64());
                                        if let Some(timestamp_ms) = timestamp_ms {
                                            //info!("  timestamp (ms): {}", timestamp_ms);
                                            let timestamp =
                                                Timestamp::from_millisecond(timestamp_ms).ok();
                                            // replace TIMESTAMP_LAST_HMIP_UPDATE
                                            TIMESTAMP_LAST_HMIP_UPDATE.lock(|tsp| {
                                                *tsp.borrow_mut() = timestamp;
                                            });
                                        }
                                        /*
                                        let time =
                                            Timestamp::from_millisecond(timestamp.unwrap_or(0))
                                                //.and_then(|t| Ok(t/* .to_zoned(TZ.clone())*/.strftime("%y%m%d %T"))).unwrap();
                                                .and_then(|t| {
                                                    Ok(t /* .to_zoned(TZ.clone())*/
                                                        .to_string())
                                                })
                                                .unwrap();
                                        info!("  timestamp: {}", &time.as_str());*/
                                    }
                                    "origin" | "accessPointId" => {}
                                    _ => {
                                        info!("Ignoring websocket field key: {}", key);
                                    }
                                }
                            }
                        } else {
                            warn!("Websocket data is not an object");
                        }
                    } else {
                        warn!("Failed to parse first JSON value from websocket data");
                    }
                } else {
                    warn!("Failed to create core_json deserializer for websocket data");
                }
            };

            // open websocket connection to url_websocket
            let _ws_res = websocket_connection(
                &url_websocket,
                &[
                    ("AUTHTOKEN", CONFIG_TOML.hmip_authtoken),
                    ("CLIENTAUTH", CONFIG_TOML.hmip_clientauth),
                    ("ACCESSPOINT-ID", CONFIG_TOML.hmip_accesspoint_id),
                ],
                process_binary_cb,
                &mut client,
            )
            .await;
        } else {
            warn!("No WebSocket URL obtained from getHost, cannot proceed");
        }
        // sleep a bit before restarting the loop
        Timer::after(Duration::from_secs(5)).await;
        info!("Restarting cloud connection task loop...");
    }
    /* in header_buf:
    HTTP/1.1 200 OK
    Server: Apache-Coyote/1.1
    Set-Cookie: JSESSIONID=FAD164160D2DFC19F18E2A2F2C5E90ED; Path=/; Secure; HttpOnly
    X-Application-Context: application
    Content-Type: application/json;charset=UTF-8
    Content-Length: 223
    Date: Sun, 26 Oct 2025 10:02:34 GMT
     */
    // info!("end cloud_connection_task");
}
