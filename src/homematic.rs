extern crate alloc;
use alloc::string::ToString;
use base64::prelude::*;
use core::{cell::RefCell, sync::atomic::AtomicBool};
use defmt::{info, warn};
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::{Duration, Timer};
use esp_hal::{rng::Rng, time::Instant};
use rand_core::RngCore;
use reqwless::{client::HttpClient, request::RequestBuilder};
use serde::Deserialize;

#[derive(Debug)]
pub struct HMIPHome {
    /// not the timestamp but the local uptime
    pub uptime_last_update: Instant,
    pub weather: WeatherData,
}

#[derive(Debug, Default)]
pub struct WeatherData {
    /// outside temperature
    pub temperature: f64,
    pub condition: heapless::String<50>,
    pub daytime: heapless::String<50>,
}

/// the devices we monitor:
#[derive(Debug)]
pub struct HeatingThermostat {
    pub id: heapless::String<50>,
    pub label: heapless::String<50>,
    pub last_status_update: i64,
    pub low_bat: bool,
    pub valve_position: f64,
    pub set_point_temp: f64,
    pub valve_act_temp: f64,
}

impl defmt::Format for HeatingThermostat {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "HeatingThermostat {{ id: {}, label: {}, last_status_update: {}, low_bat: {}, valve_position: {}, set_point_temp: {}, valve_act_temp: {} }}",
            self.id,
            self.label,
            self.last_status_update,
            self.low_bat,
            self.valve_position,
            self.set_point_temp,
            self.valve_act_temp
        )
    }
}

// stop any pending homematic connections, avoid creating new ones
pub static HMIP_CONNECTIONS_STOP: AtomicBool = AtomicBool::new(false);

// list of devices
pub static DEVICES: Mutex<CriticalSectionRawMutex, RefCell<heapless::Vec<HeatingThermostat, 6>>> =
    Mutex::new(RefCell::new(heapless::Vec::new()));

pub static HMIPHOME: Mutex<CriticalSectionRawMutex, RefCell<Option<HMIPHome>>> =
    Mutex::new(RefCell::new(None));

pub static TIMESTAMP_LAST_HMIP_UPDATE: Mutex<
    CriticalSectionRawMutex,
    RefCell<Option<jiff::Timestamp>>,
> = Mutex::new(RefCell::new(None));

pub static RTC: Mutex<CriticalSectionRawMutex, RefCell<Option<esp_hal::rtc_cntl::Rtc>>> =
    Mutex::new(RefCell::new(None));

#[derive(Deserialize, Debug)]
pub struct HmIpGetHostResponse<'a> {
    #[serde(rename = "urlREST")]
    pub url_rest: &'a str,
    #[serde(rename = "urlWebSocket")]
    pub url_websocket: &'a str,
}

#[allow(clippy::result_unit_err)]
pub fn json_process_weather<'read, 'parent, R, S>(
    weather: core_json::Field<'read, 'parent, R, S>,
    weather_data: &mut WeatherData,
) -> Result<(), ()>
where
    R: core_json::Read<'read>,
    S: core_json::Stack,
{
    if let Ok(mut weather_fields) = weather.value().fields() {
        while let Some(Ok(mut weather_field)) = weather_fields.next() {
            let weather_field_key: heapless::String<100> =
                weather_field.key().filter_map(|k| k.ok()).collect();

            match weather_field_key.as_ref() {
                "temperature" => {
                    if let Some(temp) = weather_field.value().to_number().ok().and_then(|n| n.f64())
                    {
                        //info!("  temperature: {}", temp);
                        weather_data.temperature = temp;
                    }
                }
                "weatherCondition" => {
                    if let Ok(cond_str) = weather_field.value().to_str() {
                        let cond: heapless::String<50> = cond_str.filter_map(|k| k.ok()).collect();
                        //info!("  weatherCondition: {}", cond);
                        weather_data.condition = cond;
                    }
                }
                "weatherDayTime" => {
                    if let Ok(daytime_str) = weather_field.value().to_str() {
                        let daytime: heapless::String<50> =
                            daytime_str.filter_map(|k| k.ok()).collect();
                        //info!("  weatherDayTime: {}", daytime);
                        weather_data.daytime = daytime;
                    }
                }
                _ => {}
            }
        }
        Ok(())
    } else {
        Err(())
    }
}

#[allow(clippy::result_unit_err)]
pub fn json_process_home<'read, 'parent, R, S>(
    home: core_json::Field<'read, 'parent, R, S>,
) -> Result<bool, ()>
where
    R: core_json::Read<'read>,
    S: core_json::Stack,
{
    if let Ok(mut home_fields) = home.value().fields() {
        let mut do_update = true;
        let mut hmip_home = HMIPHome {
            uptime_last_update: Instant::now(),
            weather: WeatherData::default(),
        };

        while let Some(Ok(mut home_field)) = home_fields.next() {
            let home_field_key: heapless::String<100> =
                home_field.key().filter_map(|k| k.ok()).collect();

            #[allow(clippy::single_match)]
            match home_field_key.as_ref() {
                "weather" => {
                    do_update = json_process_weather(home_field, &mut hmip_home.weather).is_ok()
                        && do_update;
                }
                _ => {}
            }
        }
        if do_update {
            HMIPHOME.lock(|home| {
                *home.borrow_mut() = Some(hmip_home);
            });
        }
        Ok(do_update)
    } else {
        Err(())
    }
}

/// Process a JSON device object and update the DEVICES list if it's a HEATING_THERMOSTAT
#[allow(clippy::result_unit_err)]
pub fn json_process_device<'read, 'parent, R, S>(
    mut device: core_json::Field<'read, 'parent, R, S>,
) -> Result<(), ()>
where
    R: core_json::Read<'read>,
    S: core_json::Stack,
{
    let mut dev_id: heapless::String<50> = device.key().filter_map(|k| k.ok()).collect();
    //info!("Processing 'device' with key: {}", dev_id);

    if let Ok(mut dev_fields) = device.value().fields() {
        let mut low_bat: Option<bool> = None;
        let mut last_status_update: Option<i64> = None;
        let mut label: Option<heapless::String<50>> = None;
        let mut valve_position: Option<f64> = None;
        let mut set_point_temp: Option<f64> = None;
        let mut valve_act_temp: Option<f64> = None;

        while let Some(Ok(mut dev_field)) = dev_fields.next() {
            let dev_field_key: heapless::String<100> =
                dev_field.key().filter_map(|k| k.ok()).collect();

            match dev_field_key.as_ref() {
                "id" => {
                    if let Ok(id_str) = dev_field.value().to_str() {
                        dev_id = id_str.filter_map(|k| k.ok()).collect();
                        //info!("  device id: {}", dev_id);
                    }
                }
                "type" => {
                    if let Ok(device_type) = dev_field.value().to_str() {
                        let dev_type: heapless::String<100> =
                            device_type.filter_map(|k| k.ok()).collect();
                        //info!("  deviceType: {}", dev_type);
                        if dev_type.as_str() != "HEATING_THERMOSTAT" {
                            return Err(());
                        }
                    }
                }
                "lastStatusUpdate" => {
                    last_status_update = dev_field.value().to_number().ok().and_then(|n| n.i64());
                    //info!("  lastStatusUpdate: {}", last_status_update);
                }
                "label" => {
                    if let Ok(label_str) = dev_field.value().to_str() {
                        label = Some(label_str.filter_map(|k| k.ok()).collect());
                        //info!("  label: {}", label);
                    }
                }
                "functionalChannels" => {
                    if let Ok(mut channels) = dev_field.value().fields() {
                        while let Some(Ok(mut channel)) = channels.next() {
                            let channel_idx: heapless::String<10> =
                                channel.key().filter_map(|k| k.ok()).collect();

                            if let Ok(mut channel_fields) = channel.value().fields() {
                                while let Some(Ok(mut channel_field)) = channel_fields.next() {
                                    let channel_field_key: heapless::String<100> =
                                        channel_field.key().filter_map(|k| k.ok()).collect();

                                    match (channel_idx.as_ref(), channel_field_key.as_ref()) {
                                        ("0", "lowBat") => {
                                            if let Ok(b) = channel_field.value().to_bool() {
                                                if b {
                                                    warn!("Device {} reports low battery!", dev_id);
                                                }
                                                low_bat = Some(b);
                                            }
                                        }
                                        ("1", "valvePosition") => {
                                            valve_position = channel_field
                                                .value()
                                                .to_number()
                                                .ok()
                                                .and_then(|n| n.f64());
                                            //info!("     valvePosition: {}", valve_position);
                                        }
                                        ("1", "setPointTemperature") => {
                                            set_point_temp = channel_field
                                                .value()
                                                .to_number()
                                                .ok()
                                                .and_then(|n| n.f64());
                                            //info!("     setPointTemperature: {}", set_point_temp);
                                        }
                                        ("1", "valveActualTemperature") => {
                                            valve_act_temp = channel_field
                                                .value()
                                                .to_number()
                                                .ok()
                                                .and_then(|n| n.f64());
                                            //info!(
                                            //    "     valveActualTemperature: {}",
                                            //    valve_act_temp
                                            //);
                                        }
                                        _ => {}
                                    }
                                }
                            }
                        }
                    }
                }
                _ => {}
            }
        }

        // Check if we have all mandatory fields for a heating thermostat device
        if let (
            Some(last_status_update),
            Some(low_bat),
            Some(label),
            Some(valve_position),
            Some(set_point_temp),
            Some(valve_act_temp),
        ) = (
            last_status_update,
            low_bat,
            label,
            valve_position,
            set_point_temp,
            valve_act_temp,
        ) {
            let heating_thermostat = HeatingThermostat {
                id: dev_id.clone(),
                label,
                last_status_update,
                low_bat,
                valve_position,
                set_point_temp,
                valve_act_temp,
            };

            // Store in global devices list
            DEVICES.lock(|devices| {
                let mut devices = devices.borrow_mut();
                if let Some(pos) = devices.iter().position(|d| d.id == dev_id) {
                    devices[pos] = heating_thermostat;
                    info!("  Updated existing device:  {}", devices[pos]);
                } else {
                    info!("  Adding HeatingThermostat device: {}", heating_thermostat);
                    if devices.push(heating_thermostat).is_ok() {
                        //info!("  Added new device to DEVICES list");
                    } else {
                        warn!("  DEVICES list full, cannot add new device");
                    }
                }
            });
            Ok(())
        } else {
            info!("  Missing mandatory fields device {}. ignoring.", dev_id);
            Err(())
        }
    } else {
        Err(())
    }
}

// allow unused
#[allow(dead_code)]
struct MyWSReadResult<'a>(&'a embedded_websocket::WebSocketReadResult);
impl defmt::Format for MyWSReadResult<'_> {
    fn format(&self, fmt: defmt::Formatter) {
        let frame = &self.0;
        defmt::write!(
            fmt,
            "WebSocketReadResult {{ len_from: {}, len_to: {}, end_of_message: {}, message_type: {:?} }}",
            frame.len_from,
            frame.len_to,
            frame.end_of_message,
            defmt::Debug2Format(&frame.message_type)
        )
    }
}

pub async fn websocket_connection<'a, T, D>(
    url: &str,
    headers: &[(&str, &str)],
    mut process_binary_cb: impl for<'cb> FnMut(&mut [u8]) + 'a,
    client: &mut HttpClient<'a, T, D>,
) -> Result<(), reqwless::Error>
where
    T: embedded_nal_async::TcpConnect,
    D: embedded_nal_async::Dns,
{
    info!(
        "Starting websocket connection to '{}'. Free heap: {} bytes",
        url,
        esp_alloc::HEAP.free()
    );
    //info!("Free heap: {} bytes", esp_alloc::HEAP.free());

    // perform a regular https request with Upgrade: websocket header
    // then use the HttpConnection to read/write websocket frames
    let resource = client.resource(url).await;
    if let Ok(mut resource) = resource {
        // conn = resource.conn;
        //info!("Websocket resource created");

        let rng = Rng::new();
        let mut key_as_base64: [u8; 24] = [0; 24];
        let mut key: [u8; 16] = [0; 16];
        Rng::new().fill_bytes(&mut key);
        base64::prelude::BASE64_STANDARD
            .encode_slice(key, &mut key_as_base64)
            .unwrap();
        let sec_websocket_key = str::from_utf8(&key_as_base64).unwrap();
        //info!("Generated Sec-WebSocket-Key: {}", sec_websocket_key);

        // create own headers:
        let mut full_headers: heapless::Vec<(&str, &str), 7> = heapless::Vec::new();
        full_headers.extend_from_slice(headers).unwrap_or_default();

        let mut ws_client = embedded_websocket::WebSocketClient::new_client(rng); // TODO use this key and not the static one!
        // but there is no api for it. so generate a differnt one for now. The lib doesn't seem to check it later...

        let ws_headers = [
            ("Upgrade", "websocket"),
            ("Connection", "Upgrade"),
            ("Sec-WebSocket-Version", "13"),
            ("Sec-WebSocket-Key", sec_websocket_key),
        ];
        full_headers
            .extend_from_slice(&ws_headers)
            .unwrap_or_default();

        let empty_body: &[u8] = &[];
        let request = resource
            .request(reqwless::request::Method::GET, "/")
            .headers(&full_headers)
            .body(empty_body);
        //info!("Websocket request prepared, sending...");
        {
            let mut rx_buffer = [0_u8; (16 * 1024)];
            let response = request.send(&mut rx_buffer).await.unwrap();
            {
                let now = Instant::now();
                let headers = &response
                    .headers()
                    .filter(|(k, _)| !k.is_empty())
                    .map(|(k, v)| (k, str::from_utf8(v).unwrap_or_default()))
                    .collect::<heapless::Vec<_, 16>>();
                info!(
                    "Websocket request sent, response status: {}, content_length: {}, headers: {:?}",
                    response.status,
                    response.content_length.unwrap_or(0),
                    headers
                );
                // use "Date" header to set our realtime clock:
                for (k, v) in headers.iter() {
                    if *k == "Date" {
                        // info!(" Websocket server Date header: {}", v);
                        // parse date string and set realtime clock, e.g 'Sun, 04 Jan 2026 17:27:35 GMT', seems to be RFC7131? (or RFS7231?)
                        // might fit for RFC2822 as well
                        if let Ok(realtime) = jiff::fmt::rfc2822::parse(v) {
                            // current time from Rtc::
                            let old_current_time_us = RTC.lock(|rtc| {
                                if let Some(rtc) = rtc.borrow_mut().as_mut() {
                                    let old_current_time_us = rtc.current_time_us();
                                    rtc.set_current_time_us(
                                        realtime.timestamp().as_microsecond() as u64
                                    );
                                    Some(old_current_time_us)
                                } else {
                                    None
                                }
                            });
                            info!(
                                "  Set current time '{}' @ monotonic clock {} (diff {} us)",
                                realtime.to_string().as_str(),
                                now,
                                // old_current_time_us,
                                realtime.timestamp().as_microsecond()
                                    - old_current_time_us.unwrap_or(0) as i64
                            );
                        }
                        break;
                    }
                }
            }
            // we expect a status code 101 (and no body)
            if let Ok(body) = response.body().read_to_end().await
                && let Ok(body_str) = core::str::from_utf8(body)
            {
                info!("Response body: {}", body_str);
            }
        }

        ws_client.state = embedded_websocket::WebSocketState::Open; // we did the handshake already...
        info!("Free heap with ws_client: {} bytes", esp_alloc::HEAP.free());
        // send a ping frame:
        let mut pings_sent = 0usize;
        let mut pongs_rcvd_last_ping = 0usize;

        Timer::after(Duration::from_secs(2)).await;
        let mut tx_buffer = heapless::Vec::<u8, 10>::new();
        let _ = tx_buffer.resize_default(10);
        let len = ws_client
            .write(
                embedded_websocket::WebSocketSendMessageType::Ping,
                true,
                b"", // .as_bytes(),
                &mut tx_buffer,
            )
            .unwrap();
        let written =
            esp_mbedtls::asynch::io::Write::write(&mut resource.conn, &tx_buffer[..len]).await?;
        esp_mbedtls::asynch::io::Write::flush(&mut resource.conn).await?;
        info!(
            "Sent ping frame len {}, wrote {} bytes to websocket connection",
            len, written,
        );
        pings_sent += 1;

        // loop until connection closed
        // TODO: most frames are a lot smaller (7kb, 10kb), ignore big ones...
        let mut frame_buffer = heapless::Vec::<u8, { 30 * 1024 }>::new(); // lets hope the events fit into 30kb
        frame_buffer.resize_default(30 * 1024).unwrap();
        let mut frame_buffer_used = 0usize;
        let frame_buffer_len = frame_buffer.len();

        let mut rx_buffer = heapless::Vec::<u8, 2048>::new();
        rx_buffer.resize_default(2048).unwrap();
        let mut rx_buffer_used = 0usize;
        loop {
            match select(
                esp_mbedtls::asynch::io::Read::read(
                    &mut resource.conn,
                    &mut rx_buffer[rx_buffer_used..],
                ),
                Timer::after(Duration::from_secs(15)),
            )
            .await
            {
                Either::First(Ok(read)) => {
                    // info!(
                    //     "Read {}+{} bytes from websocket connection",
                    //     rx_buffer_used, read
                    // );
                    if read > 0 || rx_buffer_used > 0 {
                        // todo better logic for fragmented frames! (loop before read from socket?)
                        rx_buffer_used += read;
                        match ws_client.read(
                            &rx_buffer[..rx_buffer_used],
                            &mut frame_buffer
                                [core::cmp::min(frame_buffer_used, frame_buffer_len)..],
                        ) {
                            Ok(frame) => {
                                rx_buffer_used = if frame.end_of_message {
                                    // need to move the rest of the buffer to the front
                                    rx_buffer.copy_within(frame.len_from..rx_buffer_used, 0);
                                    rx_buffer_used.saturating_sub(frame.len_from)
                                } else {
                                    0
                                };
                                // info!(
                                //     "Received websocket, new rx_buffer_used={} frame: {}",
                                //     rx_buffer_used,
                                //     MyWSReadResult(&frame)
                                // );
                                match frame.message_type {
                            embedded_websocket::WebSocketReceiveMessageType::Text => {
                                frame_buffer_used += frame.len_to;
                                if frame.end_of_message {
                                    if let Ok(text) = core::str::from_utf8(&frame_buffer
                                        [..core::cmp::min(frame_buffer_used, frame_buffer_len)])
                                    {
                                        info!(" Websocket Text message: {}", text);
                                    } else {
                                        warn!(" Websocket Text message not valid UTF-8");
                                    }
                                    frame_buffer_used = 0;
                                }
                            }
                            embedded_websocket::WebSocketReceiveMessageType::Binary => {
                                frame_buffer_used += frame.len_to;
                                if frame.end_of_message {
                                    // complete message received
                                    // DEVICE_CHANGED 7kb: {"events":{"0":{"pushEventType":"DEVICE_CHANGED","device":{"id":"3014F
                                    info!(
                                        " Complete binary message received, total length {} bytes",
                                        frame_buffer_used
                                    );
                                    if frame_buffer_used > frame_buffer_len {
                                        warn!(" Frame buffer overflow, message too large!");
                                    }else{
                                        process_binary_cb(&mut frame_buffer[..frame_buffer_used]);
                                    }
                                    // for now just reset buffer:
                                    frame_buffer_used = 0;
                                }
                            }
                            embedded_websocket::WebSocketReceiveMessageType::Ping => {
                                info!(" Websocket Ping message"); // should never come from the server...?
                                // respond with Pong
                                let len = ws_client
                                    .write(
                                        embedded_websocket::WebSocketSendMessageType::Pong,
                                        true,
                                        b"", // .as_bytes(),
                                        &mut tx_buffer,
                                    )
                                    .unwrap();
                                let _ = esp_mbedtls::asynch::io::Write::write(
                                    &mut resource.conn,
                                    &tx_buffer[..len],
                                ).await?;
                            }
                            embedded_websocket::WebSocketReceiveMessageType::Pong => {
                                // info!(" Websocket Pong message");
                                // could print here in case of gaps
                                pongs_rcvd_last_ping = pings_sent;
                                frame_buffer_used = 0;
                            }
                            embedded_websocket::WebSocketReceiveMessageType::CloseMustReply | embedded_websocket::WebSocketReceiveMessageType::CloseCompleted => {
                                info!(" Websocket Close message. Waiting 6s before retrying.");
                                Timer::after(Duration::from_secs(6)).await;
                                break;
                            }
                        }
                            }
                            Err(e) => {
                                warn!(
                                    "Failed to parse websocket frame: {:?} {:?}",
                                    defmt::Debug2Format(&e),
                                    &rx_buffer[..rx_buffer_used]
                                );
                                if read == 0 {
                                    // no more data to read, might have garbage? (or frame too large?)
                                    rx_buffer_used = 0;
                                }
                            }
                        }
                    } else {
                        // check whether conn is closed: HOW? assume on read 0?
                        // wait a bit before next read
                        info!(
                            "Errornous read {}+{} bytes from websocket connection",
                            rx_buffer_used, read
                        );
                        Timer::after(Duration::from_secs(2)).await;
                    }
                }
                Either::First(Err(e)) => {
                    warn!(
                        "Error reading from websocket connection: {:?}",
                        defmt::Debug2Format(&e)
                    );
                    break;
                }
                Either::Second(_) => {
                    // check whether we did receive pongs for our pings:
                    if pings_sent.saturating_sub(pongs_rcvd_last_ping) > 2 {
                        warn!(
                            "Did not receive pong for last pings (sent {}, received last {}), closing connection",
                            pings_sent, pongs_rcvd_last_ping
                        );
                        break;
                    }
                    if HMIP_CONNECTIONS_STOP.load(core::sync::atomic::Ordering::Relaxed) {
                        warn!("HMIP_CONNECTIONS_STOP set, closing websocket connection");
                        break;
                    }
                    // send a ping
                    let len = ws_client
                        .write(
                            embedded_websocket::WebSocketSendMessageType::Ping,
                            true,
                            b"", // .as_bytes(),
                            &mut tx_buffer,
                        )
                        .unwrap();
                    let written = esp_mbedtls::asynch::io::Write::write(
                        &mut resource.conn,
                        &tx_buffer[..len],
                    )
                    .await;
                    match written {
                        Ok(_written) => {
                            //esp_mbedtls::asynch::io::Write::flush(&mut resource.conn).await?;
                            pings_sent += 1;
                        }
                        Err(e) => {
                            warn!(
                                "Error writing ping to websocket connection: {:?}",
                                defmt::Debug2Format(&e)
                            );
                            break;
                        }
                    };
                }
            };
        }
        info!("Websocket connection handling done.");
    } else {
        warn!("Failed to create resource for websocket URL");
    }

    Ok(())
}

pub async fn single_https_request<'a, T, D>(
    method: reqwless::request::Method,
    url: &str,
    headers: &[(&str, &str)],
    body: &[u8],
    mut process_response_cb: impl for<'cb> FnMut(reqwless::response::StatusCode, &mut [u8]) + 'a,
    client: &mut HttpClient<'a, T, D>,
) -> Result<(), reqwless::Error>
where
    T: embedded_nal_async::TcpConnect,
    D: embedded_nal_async::Dns,
{
    info!(
        "Performing single HTTPS {} request to '{}'. Free heap: {} bytes",
        method,
        url,
        esp_alloc::HEAP.free()
    );

    // this will consume roughly 46kb of heap for esp_mbedtls::asynch::Session
    match client.request(method, url).await {
        Ok(request) => {
            info!(
                "HTTP request created. Free heap after request: {} bytes",
                esp_alloc::HEAP.free()
            );
            let mut request = request.headers(headers).body(body);
            //info!("HTTP request prepared, sending...");
            let mut rx_buffer = [0_u8; (90 * 1024)]; // TODO reduce size!!! But the json is typically 53kb... Should refactor to use core-json async and then process sequentially
            let response = request.send(&mut rx_buffer).await?;
            info!(
                "HTTP request sent, response status: {}, content_length: {}",
                response.status,
                response.content_length.unwrap_or(0)
            );
            // info!("HTTP response: {}", response);
            // we might want to set our clock based on the Date: header in the response!

            // todo process only if http 200, content length reasonably small (<90kb), ...
            let code = response.status;
            if let Ok(body) = response.body().read_to_end().await {
                process_response_cb(code, body);
            } else {
                warn!("Failed to read response body");
            }
        }
        Err(e) => {
            // todo handle errors properly, esp the ones that can occur during lifetime like cert changes
            // -9984 = -0x2700 MBEDTLS_ERR_X509_CERT_VERIFY_FAILED
            // -24960 = -0x6180 MBEDTLS_ERR_CIPHER_ALLOC_FAILED
            warn!("Failed to create HTTP request: {:?}", e);
            return Err(e);
        }
    }
    Ok(())
}
