use core::fmt::Debug;

use defmt::{debug, error, info, warn};
use embassy_net::{IpListenEndpoint, Stack, tcp::TcpSocket};
use embassy_time::{Duration, Timer};
use embedded_storage::Storage;
use esp_bootloader_esp_idf::partitions::FlashRegion;
use esp_storage::FlashStorage;

extern crate alloc;

/// OTA support.
///
/// We do a simple TCP server listening on port 65456. When a connection is established,
/// we expect a raw ESP32C6 binary image to be sent. The image is flashed to the inactive partition,
/// and after successful flashing and verification, the new partition is marked as active and the system reboots.
///
/// The image can be generated using:
/// ```sh
/// espflash save-image --chip esp32c6 -s 4mb target/riscv32imac-unknown-none-elf/release/esp-junkers-bm1 ota_img.bin
/// ```

// ota update task
#[embassy_executor::task]
pub async fn ota_task(
    stack: Stack<'static>,
    mut flash: FlashStorage<'static>,
    mut sha: esp_hal::sha::Sha<'static>,
) {
    Timer::after(Duration::from_secs(5)).await;
    let mut buffer =
        alloc::boxed::Box::new([0u8; esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN]); // TODO optimize to only load once needed!
    let ota = esp_bootloader_esp_idf::ota_updater::OtaUpdater::new(&mut flash, buffer.as_mut()); //.unwrap(); seems to fail with debugger attached?

    if ota.is_err() {
        error!("Failed to initialize OTA updater!");
        return;
    }
    let mut ota = ota.unwrap();

    current_ota_state(&mut ota);

    loop {
        // wait until we do have a v4 ip addr:
        if !stack.is_link_up() {
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }
        if stack.config_v4().is_none() {
            Timer::after(Duration::from_secs(2)).await;
            continue;
        }
        let ota_port = 65456u16;
        info!("Starting OTA server socket on port {}...", ota_port);

        let mut rx_buf = alloc::boxed::Box::new([0_u8; 1024]); // we want 1kb chunks
        let mut tx_buf = alloc::boxed::Box::new([0_u8; 128]);
        let mut server = TcpSocket::new(stack, rx_buf.as_mut_slice(), tx_buf.as_mut_slice());

        let acc = server
            .accept(IpListenEndpoint {
                addr: None,
                port: ota_port,
            })
            .await;
        match acc {
            Ok(()) => {
                info!("OTA connection established");
                let result = process_ota_img_push(&mut server, &mut ota, &mut sha).await;
                match result {
                    Ok(()) => {
                        info!("OTA image processed successfully - rebooting...");
                        let a = ota.activate_next_partition();
                        match a {
                            Ok(_) => {
                                info!("Activated new partition successfully");
                                match ota.set_current_ota_state(
                                    esp_bootloader_esp_idf::ota::OtaImageState::New,
                                ) {
                                    Ok(_) => {
                                        let msg = b"OK: OTA image processed successfully - rebooting...\n";
                                        let _ = server.write(msg).await;
                                        Timer::after(Duration::from_secs(1)).await;
                                        let _ = server.flush().await;
                                        server.abort();
                                        info!(
                                            "Set new image state to VALID successfully. Rebooting..."
                                        );
                                        Timer::after(Duration::from_secs(1)).await;
                                        esp_hal::system::software_reset();
                                    }
                                    Err(e) => {
                                        info!("Failed to set new image state to VALID: {:?}", e);
                                    }
                                }
                            }
                            Err(e) => {
                                info!("Failed to activate new partition: {:?}", e);
                            }
                        }
                    }
                    Err(e) => {
                        info!("OTA image processing failed: {:?}", e);
                        let msg = b"ERROR: OTA image processing failed!\n";
                        let _ = server.write(msg).await;
                        Timer::after(Duration::from_secs(1)).await;
                    }
                }
            }
            Err(e) => {
                info!("Failed to accept OTA connection: {:?}", e);
            }
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}

#[derive(Default)]
pub(crate) struct EspImageSegmentHeader {
    pub load_addr: u32,
    pub data_len: u32,
}

#[derive(Default)]
#[repr(C, packed(1))]
pub(crate) struct EspImageHeader {
    pub magic: u8,
    pub segment_count: u8,
    pub spi_mode: u8,
    pub spi_speed_size: u8, // upper 4 bits: spi_speed, lower 4 bits: spi_size
    // pub spi_size: u8,
    pub entry_addr: u32,
    pub wp_pin: u8,
    pub spi_pin_drv: [u8; 3],
    pub chip_id: [u8; 2],
    pub min_chip_rev: u8,
    pub min_chip_rev_full: u16,
    pub max_chip_rev_full: u16,
    pub reserved: [u8; 4],
    pub hash_appended: u8,
}

impl EspImageHeader {
    pub fn spi_size(&self) -> u8 {
        (self.spi_speed_size >> 4) & 0x0F
    }

    pub fn spi_speed(&self) -> u8 {
        self.spi_speed_size & 0x0F
    }
}

impl Debug for EspImageHeader {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // Copy packed fields to local variables to avoid unaligned references
        let entry_addr = self.entry_addr;
        let chip_id = self.chip_id;
        let min_chip_rev_full = self.min_chip_rev_full;
        let max_chip_rev_full = self.max_chip_rev_full;

        f.debug_struct("EspImageHeader")
            .field("magic", &self.magic)
            .field("segment_count", &self.segment_count)
            .field("spi_mode", &self.spi_mode)
            .field("spi_speed", &self.spi_speed())
            .field("spi_size", &self.spi_size())
            .field("entry_addr", &format_args!("0x{:08x}", entry_addr))
            .field("wp_pin", &self.wp_pin)
            .field("chip_id", &chip_id)
            // deprecated .field("min_chip_rev", &self.min_chip_rev)
            .field("min_chip_rev_full", &min_chip_rev_full)
            .field("max_chip_rev_full", &max_chip_rev_full)
            .field("hash_appended", &self.hash_appended)
            .finish()
    }
}

impl defmt::Format for EspImageHeader {
    fn format(&self, fmt: defmt::Formatter) {
        // Copy packed fields to local variables to avoid unaligned references
        let entry_addr = self.entry_addr;
        let chip_id = self.chip_id;
        let min_chip_rev_full = self.min_chip_rev_full;
        let max_chip_rev_full = self.max_chip_rev_full;

        defmt::write!(
            fmt,
            "EspImageHeader {{ magic: {}, segment_count: {}, spi_mode: {}, spi_speed: {}, spi_size: {}, entry_addr: 0x{:08x}, wp_pin: {}, chip_id: {:?}, min_chip_rev_full: {}, max_chip_rev_full: {}, hash_appended: {} }}",
            self.magic,
            self.segment_count,
            self.spi_mode,
            self.spi_speed(),
            self.spi_size(),
            entry_addr,
            self.wp_pin,
            chip_id,
            min_chip_rev_full,
            max_chip_rev_full,
            self.hash_appended
        )
    }
}

const _: () = assert!(core::mem::size_of::<EspImageHeader>() == 24);
const _: () = assert!(core::mem::size_of::<EspImageSegmentHeader>() == 8);

async fn process_ota_img_push<F>(
    socket: &mut TcpSocket<'_>,
    ota: &mut esp_bootloader_esp_idf::ota_updater::OtaUpdater<'_, F>,
    sha: &mut esp_hal::sha::Sha<'static>,
) -> Result<(), embassy_net::tcp::Error>
where
    F: embedded_storage::Storage,
{
    info!("Processing OTA image push - checking header...");
    let mut esp_image_header: EspImageHeader = unsafe { core::mem::zeroed() };
    let header_bytes: &mut [u8] = unsafe {
        core::slice::from_raw_parts_mut(
            &mut esp_image_header as *mut EspImageHeader as *mut u8,
            core::mem::size_of::<EspImageHeader>(),
        )
    };
    assert!(header_bytes.len() == 24);
    let read = read_chunk(socket, header_bytes).await?;
    if read != core::mem::size_of::<EspImageHeader>() {
        warn!("Invalid image header size");
        return Err(embassy_net::tcp::Error::ConnectionReset);
    }
    info!("Image header read: {}", esp_image_header);
    if esp_image_header.magic != 0xE9 {
        warn!("Invalid image magic");
        return Err(embassy_net::tcp::Error::ConnectionReset);
    }
    // image for 4mb flash?
    if esp_image_header.spi_size() != 0x02 {
        warn!("Invalid image spi size (!= 4MB)");
        return Err(embassy_net::tcp::Error::ConnectionReset);
    }
    // image for ESP32C6?
    if esp_image_header.chip_id != [0x0d, 0x0] {
        // from https://github.com/espressif/esp-idf/blob/ff97953b32a32e44f507593320b50d728eea3f06/components/bootloader_support/include/esp_app_format.h#L25
        warn!("Invalid image chip id (!= ESP32C6)");
        return Err(embassy_net::tcp::Error::ConnectionReset);
    }

    if esp_image_header.segment_count == 0 || esp_image_header.segment_count > 16 {
        warn!("Invalid segment count {}", esp_image_header.segment_count);
        return Err(embassy_net::tcp::Error::ConnectionReset);
    }
    if esp_image_header.hash_appended == 0 {
        warn!("Image does not have (sha256) hash appended. We're not flashing non-hashed images.");
        return Err(embassy_net::tcp::Error::ConnectionReset);
    }

    // start updating the inactive partition
    info!("Starting OTA update to inactive partition...");
    let _ = socket
        .write(b"INFO: Starting OTA update to inactive partition...\n")
        .await;
    if let Ok((mut next_app_partition, part_type)) = ota.next_partition() {
        let part_size = next_app_partition.partition_size();
        info!(
            "Flashing new image to {:?} with part size {}",
            part_type, part_size
        );

        if part_size < 0x1e0000 {
            warn!("Partition size too small ({} < 0x1e0000) for image", part_size);
            return Err(embassy_net::tcp::Error::ConnectionReset);
        }

        let mut sha_hasher = sha.start::<esp_hal::sha::Sha256>();

        let mut flash_offset = 0usize;
        match next_app_partition.write(flash_offset as u32, header_bytes) {
            Ok(()) => {
                info!("Wrote image header to flash at offset {}", flash_offset);
            }
            Err(e) => {
                info!("Failed to write image header to flash: {:?}", e);
                return Err(embassy_net::tcp::Error::ConnectionReset);
            }
        }
        flash_offset += header_bytes.len();
        update_sha256(&mut sha_hasher, header_bytes);

        debug!(
            "Reading {} image segments...",
            esp_image_header.segment_count
        );

        let mut data_buf = alloc::boxed::Box::new([0u8; 4096]);
        for seg_idx in 0..esp_image_header.segment_count {
            info!(
                "Reading segment {}/{} header...",
                seg_idx + 1,
                esp_image_header.segment_count
            );
            let mut seg_header: EspImageSegmentHeader = EspImageSegmentHeader::default();
            let seg_header_bytes: &mut [u8] = unsafe {
                core::slice::from_raw_parts_mut(
                    &mut seg_header as *mut EspImageSegmentHeader as *mut u8,
                    core::mem::size_of::<EspImageSegmentHeader>(),
                )
            };
            let read = read_chunk(socket, seg_header_bytes).await?;
            if read != core::mem::size_of::<EspImageSegmentHeader>() {
                warn!("Invalid segment header size: read {} bytes", read);
                return Err(embassy_net::tcp::Error::ConnectionReset);
            }
            let _ = socket.write(b"INFO: Processing segment ").await;
            let _ = socket
                .write(alloc::format!("{} len: {}\n", seg_idx + 1, seg_header.data_len).as_bytes())
                .await;
            info!(
                "Segment header: load_addr=0x{:08x}, data_len={}",
                seg_header.load_addr, seg_header.data_len
            );
            // read segment data, update crc and write directly to flash
            // read in chunks of 4kb // need to align offsets...
            let mut remaining = seg_header.data_len as usize;

            // write segment header to flash
            write_flash(&mut next_app_partition, seg_header_bytes, flash_offset).await?;
            flash_offset += seg_header_bytes.len();
            update_sha256(&mut sha_hasher, seg_header_bytes);
            // write data chunks, starting with the less than 4kb, then 4kb chunks, then the last less than 4kb
            // let first chunk be less than 4kb
            let bytes_to_next_page = if !flash_offset.is_multiple_of(4096) {
                4096 - (flash_offset % 4096)
            } else {
                4096
            };
            let first_chunk_size = remaining.min(4096).min(bytes_to_next_page);
            assert!(first_chunk_size > 0);
            let read =
                read_chunk(socket, &mut (data_buf.as_mut_slice()[..first_chunk_size])).await?;
            if read != first_chunk_size && read != remaining {
                warn!(
                    "Unexpected EOF read {} bytes, expected {}",
                    read, first_chunk_size
                );
                return Err(embassy_net::tcp::Error::ConnectionReset);
            }
            write_flash(
                &mut next_app_partition,
                &data_buf.as_mut_slice()[..first_chunk_size],
                flash_offset,
            )
            .await?;
            flash_offset += first_chunk_size;
            update_sha256(&mut sha_hasher, &data_buf.as_slice()[..first_chunk_size]);
            remaining -= first_chunk_size;

            while remaining > 0 {
                assert!(flash_offset.is_multiple_of(4096));
                let to_read = remaining.min(4096);
                let read = read_chunk(socket, &mut (data_buf.as_mut_slice()[..to_read])).await?;

                if read != to_read {
                    warn!("Unexpected EOF read {} bytes, expected {}", read, to_read);
                    return Err(embassy_net::tcp::Error::ConnectionReset);
                }
                write_flash(
                    &mut next_app_partition,
                    &data_buf.as_mut_slice()[..to_read],
                    flash_offset,
                )
                .await?;
                flash_offset += to_read;
                update_sha256(&mut sha_hasher, &data_buf.as_slice()[..to_read]);
                remaining -= to_read;
            }
        }
        let _ = socket.write(b"INFO: Processing SHA...\n").await;
        debug!("Reading post-segment data crc and sha...");
        let read = read_chunk(socket, data_buf.as_mut_slice()).await?;
        info!("Read {} bytes of post-segment data", read);
        if read > 32 && read <= 32 + 16 {
            // 1 byte checksum follows that is 16 byte padded
            // todo now verify crc? sanity checks?
            write_flash(
                &mut next_app_partition,
                &data_buf.as_mut_slice()[..read],
                flash_offset,
            )
            .await?;
            flash_offset += read;
            info!(
                "OTA image upload complete, total flashed size {} bytes",
                flash_offset
            );
            update_sha256(&mut sha_hasher, &data_buf.as_slice()[..read - 32]);
            let mut sha256_from_recvd = [0u8; 32];
            nb::block!(sha_hasher.finish(sha256_from_recvd.as_mut_slice()));
            if sha256_from_recvd == data_buf[read - 32..read] {
                info!("SHA256 verification successful");
                // info!("Received SHA256:    {:?}", &sha256_from_recvd);
            } else {
                warn!("SHA256 verification failed");
                //info!("Received SHA256:    {:?}", &data_buf[read - 32..read]);
                //info!("Computed SHA256:    {:?}", &sha256_from_recvd);
                return Err(embassy_net::tcp::Error::ConnectionReset);
            }
        } else {
            warn!("No SHA256 hash received ({} != 32), treat as failure", read);
            return Err(embassy_net::tcp::Error::ConnectionReset);
        }

        Ok(())
    } else {
        Err(embassy_net::tcp::Error::ConnectionReset)
    }
}

async fn write_flash<F>(
    next_app_partition: &mut FlashRegion<'_, F>,
    data: &[u8],
    flash_offset: usize,
) -> Result<(), embassy_net::tcp::Error>
where
    F: embedded_storage::Storage,
{
    if data.is_empty() {
        return Ok(());
    }

    match next_app_partition.write(flash_offset as u32, data) {
        Ok(()) => {
            debug!(
                "Wrote {} bytes of data to flash at offset {}",
                data.len(),
                flash_offset
            );
        }
        Err(e) => {
            info!("Failed to write segment data to flash: {:?}", e);
            return Err(embassy_net::tcp::Error::ConnectionReset);
        }
    }
    Ok(())
}

async fn read_chunk(
    socket: &mut TcpSocket<'_>,
    buf: &mut [u8],
) -> Result<usize, embassy_net::tcp::Error> {
    let mut total_read = 0;
    while total_read < buf.len() {
        let read = socket.read(&mut buf[total_read..]).await?;
        //info!("Read {} bytes from socket", read);
        if read == 0 {
            break;
        }
        total_read += read;
    }
    debug!("Read {}/{} bytes from socket", total_read, buf.len());
    Ok(total_read)
}

pub fn current_ota_state<F>(ota: &mut esp_bootloader_esp_idf::ota_updater::OtaUpdater<'_, F>)
where
    F: embedded_storage::Storage,
{
    info!("OTA: Reading OTA state from flash storage...");
    /*let mut buffer = [0u8; esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN];
      let pt =
          esp_bootloader_esp_idf::partitions::read_partition_table(&mut flash, &mut buffer).unwrap();
      // List all partitions - this is just FYI
      for part in pt.iter() {
          info!("Partition: {}", part);
      }
      info!("Currently booted partition {:?}", pt.booted_partition());
    */
    //let mut ota =
    //    esp_bootloader_esp_idf::ota_updater::OtaUpdater::new(&mut flash, &mut buffer).unwrap();

    info!(
        "Build version: built on {}, at {}",
        //esp_bootloader_esp_idf::ESP_APP_DESC.version,
        esp_bootloader_esp_idf::BUILD_DATE,
        esp_bootloader_esp_idf::BUILD_TIME
    );

    let current = ota.selected_partition().unwrap();
    info!(
        "OTA :Current image state {:?} (only relevant if the bootloader was built with auto-rollback support)",
        ota.current_ota_state()
    );

    // TODO when to mark new one as valid? (should do only after e.g. a min of uptime)
    if let Ok(state) = ota.current_ota_state()
        && (state == esp_bootloader_esp_idf::ota::OtaImageState::New
            || state == esp_bootloader_esp_idf::ota::OtaImageState::PendingVerify)
    {
        info!("Changed state to VALID");
        ota.set_current_ota_state(esp_bootloader_esp_idf::ota::OtaImageState::Valid)
            .unwrap();
    }

    info!("OTA :Currently selected partition {:?}", current);

    let (mut _next_app_partition, part_type) = ota.next_partition().unwrap();

    info!("OTA: Would flash new image to {:?}", part_type);
}

fn update_sha256<'a>(
    sha: &mut esp_hal::sha::ShaDigest<'a, esp_hal::sha::Sha256, &mut esp_hal::sha::Sha<'a>>,
    data: &[u8],
) {
    let mut data = data;
    while !data.is_empty() {
        data = nb::block!(sha.update(data)).unwrap();
    }
}

/*
struct Sha256 <'b> {
    sha_hasher: esp_hal::sha::ShaDigest<'static, esp_hal::sha::Sha256, &'b mut esp_hal::sha::Sha<'static>>,
}

impl<'b> Sha256<'b> {
    pub fn new(sha_hasher: esp_hal::sha::ShaDigest<'static, esp_hal::sha::Sha256, &'b mut esp_hal::sha::Sha<'static>>) -> Self {
      //let sha_hasher = sha.start::<esp_hal::sha::Sha256>();
        Sha256 {
          sha_hasher
        }
    }

    pub fn update(&mut self, data: &[u8]) {
      let mut data = data;
      while !data.is_empty() {
          data = nb::block!(self.sha_hasher.update(data)).unwrap();
      }
    }

    pub fn finish(&mut self) -> [u8; 32] {
        let mut sha256_res = [0u8; 32];
        nb::block!(self.sha_hasher.finish(sha256_res.as_mut_slice()));
        sha256_res
    }
}

impl Drop for Sha256<'_> {
    fn drop(&mut self) {
        // make sure we finish the sha when going out of scope
        // TODO? self.sha_hasher.cancel();
    }
}
*/
