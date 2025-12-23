// copied from https://github.com/probe-rs/rtt-target/blob/master/rtt-target/Cargo.toml
// MIT License
//
// extended to use fring as ringbuffer for defmt data

use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use defmt::{info, warn};
use embassy_net::{IpEndpoint, Stack, tcp::TcpSocket};
use embassy_time::{Duration, Timer};
use rtt_target::{UpChannel, rtt_init};

use embedded_storage::ReadStorage;
use esp_storage::FlashStorage;

use crate::ota::{EspImageHeader, EspImageSegmentHeader}; // TODO use core::sync::atomic::...
extern crate alloc;

// ringbuffer used for tcp transfer
const LOG_RINGBUF_SIZE: usize = 2 * 1024;
static mut LOG_RINGBUF: fring::Buffer<LOG_RINGBUF_SIZE> = fring::Buffer::new();

const RTT_BUFFER_SIZE: usize = 512;
static mut CHANNEL: Option<UpChannel> = None;

#[defmt::global_logger]
struct Logger;

/// Sets the channel to use for [`defmt`] macros.
fn set_defmt_channel(channel: UpChannel) {
    unsafe { CHANNEL = Some(channel) }
}

/// Global logger lock.
static TAKEN: AtomicBool = AtomicBool::new(false);
static mut CS_RESTORE: critical_section::RestoreState = critical_section::RestoreState::invalid();
static mut ENCODER: defmt::Encoder = defmt::Encoder::new(); // TODO ensure this is encoding-rzcobs

static mut PRODUCER: Option<fring::Producer<'static, LOG_RINGBUF_SIZE>> = None;

static mut WRITE_REGION: Option<
    fring::Region<'static, fring::Producer<'static, LOG_RINGBUF_SIZE>>,
> = None;
static LOST_BYTES: AtomicUsize = AtomicUsize::new(0);

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        // safety: Must be paired with corresponding call to release(), see below
        let restore = unsafe { critical_section::acquire() };

        if TAKEN.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly")
        }

        // no need for CAS because interrupts are disabled
        TAKEN.store(true, Ordering::Relaxed);

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        unsafe { CS_RESTORE = restore };

        unsafe {
            if let Some(producer) = &mut *core::ptr::addr_of_mut!(PRODUCER) {
                WRITE_REGION = Some(producer.write(usize::MAX));
            }
        };

        // safety: accessing the `static mut` is OK because we have disabled interrupts / are mutex protected.
        unsafe {
            let encoder = &mut *core::ptr::addr_of_mut!(ENCODER);
            encoder.start_frame(do_write)
        }
    }

    unsafe fn flush() {}

    unsafe fn release() {
        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        let encoder = unsafe { &mut *core::ptr::addr_of_mut!(ENCODER) };
        encoder.end_frame(do_write);

        // drop the write region to commit the data
        unsafe {
            let cur_region = (*core::ptr::addr_of_mut!(WRITE_REGION)).take();
            if let Some(region) = cur_region {
                region.partial_drop(0); // we dont need the rest
            }
            WRITE_REGION = None;
        };

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        TAKEN.store(false, Ordering::Relaxed);

        // safety: accessing the `static mut` is OK because we have acquired a critical section.
        let restore = unsafe { CS_RESTORE };

        // safety: Must be paired with corresponding call to acquire(), see above
        unsafe { critical_section::release(restore) };
    }

    unsafe fn write(bytes: &[u8]) {
        // safety: accessing the `static mut` is OK because we have disabled interrupts.
        let encoder = unsafe { &mut *core::ptr::addr_of_mut!(ENCODER) };
        encoder.write(bytes, do_write);
    }
}

fn do_write(bytes: &[u8]) {
    unsafe {
        //let channel = core::ptr::addr_of_mut!(CHANNEL);
        let channel = &raw mut CHANNEL;
        if let Some(Some(c)) = channel.as_mut() {
            c.write(bytes);
        }
        if let Some(region) = &mut *core::ptr::addr_of_mut!(WRITE_REGION) {
            // write to region:
            let to_copy = bytes.len().min(region.len());

            // did we loose/drop data?
            if to_copy < bytes.len() {
                let lost = bytes.len() - to_copy;
                LOST_BYTES.fetch_add(lost, Ordering::Relaxed);
            }

            if to_copy == 0 {
                return;
            }
            // TODO might be easier to copy into a local buf and on release copy that into the ringbuffer
            // would avoid partial frames as well!
            region[..to_copy].copy_from_slice(&bytes[..to_copy]);
            region.consume(to_copy);
        }
    }
}

/// our init function to be called once at program start
///
/// inits an RTT channel and sets it as defmt channel
///
pub fn init() -> fring::Consumer<'static, LOG_RINGBUF_SIZE> {
    let channels = rtt_init! {
        up: {
            0: {
                size: RTT_BUFFER_SIZE,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "defmt"
            }
        }
    };
    set_defmt_channel(channels.up.0);
    // UNSAFE: need to ensure init can only be called once!
    let log_ringbuffer = unsafe { &mut *core::ptr::addr_of_mut!(LOG_RINGBUF) };
    let (producer, consumer) = log_ringbuffer.split();
    unsafe {
        PRODUCER = Some(producer);
    }
    consumer
}

// ota update task
#[embassy_executor::task]
pub async fn log_serve_task(
    stack: Stack<'static>,
    mut log_consumer: fring::Consumer<'static, LOG_RINGBUF_SIZE>,
    log_server_endpoint: IpEndpoint,
) {
    loop {
        // wait until we do have a v4 ip addr: (but with short latency)
        if !stack.is_link_up() {
            Timer::after(Duration::from_millis(10)).await;
            continue;
        }
        if stack.config_v4().is_none() {
            Timer::after(Duration::from_millis(10)).await;
            continue;
        }

        let mut rx_buf = alloc::boxed::Box::new([0_u8; 16]);
        let mut tx_buf = alloc::boxed::Box::new([0_u8; 256]); // TODO which size to use?
        let mut client = TcpSocket::new(stack, rx_buf.as_mut_slice(), tx_buf.as_mut_slice());
        client.set_timeout(Some(Duration::from_secs(3))); // we dont expect any data from the server anyhow
        client.set_keep_alive(Some(Duration::from_secs(2))); // can this be larger than the timeout?
        //client.set_nagle_enabled(false);
        match client.connect(log_server_endpoint).await {
            Ok(()) => {
                defmt::info!("defmt_via_tcp: connected to log server");

                // send initial headers
                match send_initial_headers(&mut client).await {
                    Ok(()) => {
                        info!("defmt_via_tcp: sent initial headers");
                    }
                    Err(e) => {
                        warn!("defmt_via_tcp: could not send initial headers: {:?}", e);
                        // close socket and retry
                        client.abort();
                        Timer::after(Duration::from_secs(1)).await; // depending on the error code use a different delay?
                        continue;
                    }
                }
            }
            Err(e) => {
                warn!("defmt_via_tcp: could not connect to log server: {:?}", e);
                Timer::after(Duration::from_secs(1)).await; // depending on the error code use a different delay?
                continue;
            }
        }

        // forward log data:
        // we need to start with sending:
        // protocol version (our own)
        // build-id
        let mut last_lost_logged = 0;
        loop {
            if log_consumer.data_size() == 0 {
                // now we're idle check for lost bytes:
                let lost = LOST_BYTES.load(Ordering::Relaxed);
                if last_lost_logged != lost {
                    last_lost_logged = lost;
                    warn!("Lost {} bytes of log data", lost);
                }

                // todo: which minimal size to use?
                Timer::after(Duration::from_millis(50)).await;
                continue;
            }
            if client.can_send() {
                let read_region = log_consumer.read(256); // TODO socket size?
                let data = &*read_region;
                match client.write(data).await {
                    Ok(n) => {
                        read_region.partial_drop(n);
                    }
                    Err(e) => {
                        warn!(
                            "log_serve_task: write error: {:?}, restarting server loop",
                            e
                        );
                        read_region.partial_drop(0); // dont drop anything
                        break;
                    }
                }
            } else {
                if client.may_send() {
                    // info!("log_serve_task: socket not ready to send, waiting");
                    // would need to do that with timeout...client.wait_write_ready().await;
                    Timer::after(Duration::from_millis(50)).await;
                    continue;
                }
                warn!("log_serve_task: socket closed, restarting server loop");
                break;
                // todo if we cannot send, shall we flush data? do we prefer the last or the first data?
            }
        }
        // we should end here only if the socket is closed/had an error
        // warn!("log_serve_task: socket closed or had error, restarting server loop");
    }
}

unsafe extern "Rust" {
    // #[link_name = "esp_app_desc"]
    // static ESP_APP_DESC: esp_bootloader_esp_idf::EspAppDesc;
}

const PROTOCOL_VERSION: u8 = 1;

// sadly accessing the gnu build id doesn't seem to work. So using the env! macro here
const BUILD_ID: &str = env!("BUILD_ID");
const _: () = assert!(BUILD_ID.len() == 32);

async fn send_initial_headers(socket: &mut TcpSocket<'_>) -> Result<(), embassy_net::tcp::Error> {
    // send protocol version
    let proto_ver = PROTOCOL_VERSION;
    socket.write(&[proto_ver]).await?;

    // app version:
    // info!("app version: {}", unsafe { ESP_APP_DESC.version() });

    // send build-id (32 bytes hex string (from 16 bytes uuid))
    info!("build_id: {}", BUILD_ID);
    socket.write(BUILD_ID.as_bytes()).await?;

    Ok(())
}

/// can be used to read the app_sha256 of the currently running app
pub async fn get_current_app_sha256(flash: &mut FlashStorage<'static>) -> [u8; 32] {
    let mut buffer = [0u8; esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN];
    let boot_part = {
        let mut ota_upd =
            esp_bootloader_esp_idf::ota_updater::OtaUpdater::new(flash, &mut buffer).unwrap();
        ota_upd.selected_partition()
    };
    let ota_part_to_use = match boot_part {
        Ok(part) => part,
        _ => {
            warn!("get_current_app_sha256: no selected partition from ota_updater");
            return [0u8; 32];
        }
    };
    let pt = esp_bootloader_esp_idf::partitions::read_partition_table(flash, &mut buffer).unwrap();
    let boot_part = pt.find_partition(esp_bootloader_esp_idf::partitions::PartitionType::App(
        ota_part_to_use,
    ));

    if let Ok(Some(booted_partition)) = boot_part {
        let mut flash_region = booted_partition.as_embedded_storage(flash);
        // need to read the image header, evaluate number of segments and read all segments to get to the crc/sha256
        let mut esp_image_header = EspImageHeader::default();
        let header_bytes: &mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(
                &mut esp_image_header as *mut EspImageHeader as *mut u8,
                core::mem::size_of::<EspImageHeader>(),
            )
        };
        flash_region.read(0, header_bytes).unwrap();
        let num_segments = esp_image_header.segment_count;
        let mut offset = core::mem::size_of::<EspImageHeader>() as u32;
        for _ in 0..num_segments {
            let mut seg_header: EspImageSegmentHeader = EspImageSegmentHeader::default();
            let seg_header_bytes: &mut [u8] = unsafe {
                core::slice::from_raw_parts_mut(
                    &mut seg_header as *mut EspImageSegmentHeader as *mut u8,
                    core::mem::size_of::<EspImageSegmentHeader>(),
                )
            };
            flash_region.read(offset, seg_header_bytes).unwrap();
            offset += core::mem::size_of::<EspImageSegmentHeader>() as u32;
            offset += seg_header.data_len;
        }
        // skip the crc byte
        offset += 1; // formally the padding bytes come first, but we ignore it anyhow
        // the sha256 is at the next 16 bytes aligned address
        offset = offset + ((16 - (offset % 16)) % 16);

        if offset as usize + 32 > flash_region.capacity() {
            return [0u8; 32];
        }
        let mut sha256_buf = [0u8; 32];
        flash_region.read(offset, &mut sha256_buf).unwrap();
        return sha256_buf;
    }
    warn!("get_current_app_sha256: could not determine booted partition");
    [0u8; 32]
}
