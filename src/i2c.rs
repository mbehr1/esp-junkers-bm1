use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU8};
use critical_section::Mutex;
use defmt::{error, info};
use embassy_time::{Duration, Timer};
use esp_hal::i2c::slave::SlaveEvent;
use esp_hal::time::Instant;
use heapless::spsc::{Consumer, Producer, Queue};

const SLAVE_ADDR: u8 = 0x55; // i2c address to simulate Junkers BM1 PCF8570 ram chip (todo use 0x50...)

/// our interface to main logic:

/// output enable for level shifter to boiler side
pub static OUTPUT_ENABLE: AtomicBool = AtomicBool::new(false);

///
/// status from boiler: (boiler state and uptime we did receive it)
pub static BOILER_STATE: Mutex<RefCell<Option<(BoilerState, Instant)>>> =
    Mutex::new(RefCell::new(None));

/// power setting from remote control
pub static REMOTE_POWER: AtomicU8 = AtomicU8::new(0xff); // ff = full?
/// VL_SOLL temperature setting in 0.5 degree C steps
pub static REMOTE_VL_SOLL2: AtomicU8 = AtomicU8::new(10 * 2); // 10C
/// WW_SOLL temperature setting in 0.5 degree C steps
pub static REMOTE_WW_SOLL2: AtomicU8 = AtomicU8::new(10 * 2); // 10C
/// stop pump command from remote control
pub static REMOTE_STOP_PUMP: AtomicU8 = AtomicU8::new(1); // 0 = pump stopped
/// error code to set in remote state
pub static REMOTE_ERROR: AtomicU8 = AtomicU8::new(0); // no error

// the real PCF8570 seems quite fast:
// after write addr before read it keeps (only) SCL low for 135-150us.

// atomic counter for received data:
// static RECEIVED_DATA_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
// static RECEIVED_CALL_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
// static REQUEST_CALL_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

static RAM: Mutex<RefCell<BMRam>> = Mutex::new(RefCell::new(BMRam::new()));
static RAM_NEXT_ADDR: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

struct ReceiveData {
    data: heapless::Vec<u8, 32>,
    reason: SlaveEvent,
}

impl defmt::Format for ReceiveData {
    fn format(&self, f: defmt::Formatter) {
        let (reason_str, arg) = match self.reason {
            SlaveEvent::StretchRxFull(a) => ("StretchRxFull", a),
            SlaveEvent::StretchAddrMatch(a) => ("AddressMatch", a),
            SlaveEvent::StretchTxEmpty => ("StretchTxEmpty", 0),
            SlaveEvent::TransComplete(a) => ("TransComplete", a),
            SlaveEvent::RxFifoWatermark(a) => ("RxFifoWatermark", a),
        };
        defmt::write!(
            f,
            "ReceiveData {{ reason: {}({}), data: {:02x} }}",
            reason_str,
            arg,
            self.data
        );
    }
}

static PC: (
    Mutex<RefCell<Option<Producer<'_, ReceiveData>>>>,
    Mutex<RefCell<Option<Consumer<'_, ReceiveData>>>>,
) = {
    static mut Q: Queue<ReceiveData, 16> = Queue::new();
    // SAFETY: `Q` is only accessible in this scope.
    #[allow(static_mut_refs)]
    let (p, c) = unsafe { Q.split_const() };

    (
        Mutex::new(RefCell::new(Some(p))),
        Mutex::new(RefCell::new(Some(c))),
    )
};

#[esp_hal::ram]
fn on_i2c_slave_event(reason: SlaveEvent, data: &[u8], data_to_write: &mut [u8]) -> usize {
    //info!("I2C on_i2c_slave_event callback called with data: {:02x}", data);
    // RECEIVED_CALL_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    // RECEIVED_DATA_COUNT.fetch_add(data.len() as u32, core::sync::atomic::Ordering::Relaxed);

    let producer = {
        static mut P: Option<Producer<'_, ReceiveData>> = None;
        // SAFETY: Mutable access to `P` is allowed exclusively in this scope
        // and `interrupt` cannot be called directly or preempt itself.
        //unsafe { &mut P }
        unsafe {
            let p = &mut *(&raw mut P);
            p.get_or_insert_with(|| {
                critical_section::with(|cs| PC.0.borrow_ref_mut(cs).take().unwrap())
            })
        }
    };

    match reason {
        SlaveEvent::StretchAddrMatch(offset) => {
            if offset == 0 && data.len() >= 1 {
                // todo error handling, only 1 byte expected
                let addr = data[0];
                // set RAM next address
                RAM_NEXT_ADDR.store(addr as u8, core::sync::atomic::Ordering::Relaxed); // TODO which memory ordering?
            }
        }
        SlaveEvent::RxFifoWatermark(offset)
        | SlaveEvent::StretchRxFull(offset)
        | SlaveEvent::TransComplete(offset) => {
            if data.len() >= 1 {
                let (addr, data) = if offset == 0 {
                    (data[0] as usize, &data[1..])
                } else {
                    // continuing write, so use current RAM_NEXT_ADDR
                    (
                        RAM_NEXT_ADDR.load(core::sync::atomic::Ordering::Relaxed) as usize,
                        data,
                    )
                };
                critical_section::with(|cs| {
                    let mut ram = RAM.borrow_ref_mut(cs);
                    let max_addr = core::cmp::min(ram.ram.len(), addr + data.len());
                    let ram_slice = &mut ram.ram[addr..max_addr];
                    ram_slice.copy_from_slice(&data[..ram_slice.len()]); // we dont wrap here...
                });
                // TODO does write to set the next read address as well? Check PCF spec/behavior. If so, set RAM_NEXT_ADDR here.
                let next_addr = addr + data.len();
                let next_addr = if next_addr >= 256 { 0 } else { next_addr };
                RAM_NEXT_ADDR.store(
                    (next_addr % 256) as u8,
                    core::sync::atomic::Ordering::Relaxed,
                );
            }
        }
        _ => {}
    }

    // we write if the data_to_write is non empty
    // logically we should only do it on: StretchAddrMatch and StretchTxEmpty
    let written = if !data_to_write.is_empty() {
        assert!(matches!(
            reason,
            SlaveEvent::StretchAddrMatch(_) | SlaveEvent::StretchTxEmpty
        ));

        let addr = RAM_NEXT_ADDR.load(core::sync::atomic::Ordering::SeqCst) as usize;
        let written = critical_section::with(|cs| {
            let ram = RAM.borrow_ref(cs);
            let max_addr = core::cmp::min(ram.ram.len(), addr + data_to_write.len());
            let ram_data = &ram.ram[addr..max_addr];
            let to_write_len = core::cmp::min(data_to_write.len(), ram_data.len());
            // E.g. limit to 4 bytes for testing the stretch tx fifo behavior! Leads to a 35us SCL stretch low every 4 bytes
            data_to_write[..to_write_len].copy_from_slice(&ram_data[..to_write_len]);
            to_write_len
        });
        // this assumes that all bytes are written. But the master might NAK earlier...
        RAM_NEXT_ADDR.store((addr + written) as u8, core::sync::atomic::Ordering::SeqCst);
        // REQUEST_CALL_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        written
    } else {
        0
    };

    // we enqueue received data only for debugging purposes, not needed for functionality
    let _ = producer.enqueue(ReceiveData {
        data: {
            let mut v = heapless::Vec::<u8, 32>::new();
            let _ = v.extend_from_slice(data); // ignore error if data too long
            v
        },
        reason,
    });
    written
}

// MARK: i2c task
#[embassy_executor::task]
pub async fn i2c_task(
    i2c_peripheral: esp_hal::peripherals::I2C0<'static>,
    gpio_sda: esp_hal::peripherals::GPIO4<'static>,
    gpio_scl: esp_hal::peripherals::GPIO5<'static>,
    gpio_oe: esp_hal::peripherals::GPIO0<'static>,
) {
    info!("task 'i2c_task' running...");

    // the level shifter has the OE (output enable) ping tied to GPIO0. It has a pull up to vcc (3.3V)
    // but we can turn it off by pulling GPIO0 low
    let mut gpio0 = esp_hal::gpio::Output::new(
        gpio_oe,
        esp_hal::gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );

    let config = esp_hal::i2c::slave::Config::default()
        .with_address(SLAVE_ADDR.into())
        .with_rx_fifo_wm_threshold_none()
        //.with_rx_fifo_wm_threshold(2) // only for testing. leads to RxFifoWatermark events
        ;

    // bugs to investigate from i2c_slave from other branch from esp-hal
    // - on i2c.write the first byte seems to be still the last NAK'd byte from previous write! (added to https://github.com/esp-rs/esp-hal/issues/1909#issuecomment-3592556119 )
    // - is it normal that a write ends with a NAK from master side? The problem is that the slave doesn't
    //  know how many bytes the master wants to read, so it "offers" more bytes with the write request.

    let mut i2c = esp_hal::i2c::slave::I2c::new(i2c_peripheral, config)
        .expect("Failed to create I2C slave")
        .with_sda(gpio_sda)
        .with_scl(gpio_scl);

    i2c.register_callbacks(&on_i2c_slave_event); // enables interrupts internally

    info!("I2C slave initialized with address 0x{:02X}", SLAVE_ADDR);
    // let mut count = 0u32;
    gpio0.set_high(); // enable level shifter output to boiler side
    OUTPUT_ENABLE.store(true, core::sync::atomic::Ordering::Relaxed);

    let mut consumer = critical_section::with(|cs| PC.1.borrow_ref_mut(cs).take().unwrap());
    loop {
        // count += 1;
        // info!(
        //     "I2C stats: received {} calls, {} bytes; request calls: {}",
        //     RECEIVED_CALL_COUNT.load(core::sync::atomic::Ordering::Relaxed),
        //     RECEIVED_DATA_COUNT.load(core::sync::atomic::Ordering::Relaxed),
        //     REQUEST_CALL_COUNT.load(core::sync::atomic::Ordering::Relaxed),
        // );
        while let Some(rcvd_data) = consumer.dequeue() {
            //info!("I2C: Data received {}", rcvd_data);
            if let SlaveEvent::StretchTxEmpty = rcvd_data.reason {
                error!("I2C: StretchTxEmpty event received!");
            } else {
                //info!("I2C: Data received {}", rcvd_data);
            }
        }

        // from i2c.txt the sequence is:
        // boiler reads from 0xfe 2 bytes (expects 0xfc 0x03)
        // boild reads from 0x10 9 bytes (0x00 0x01 0x00 0x00 0x00 0x00 0x00 0x00 0x00)
        // (reads from 0x90 16 bytes)
        // boiler writes to 0x20 (BoilerState) 13 bytes and then
        // writes to 0x10 two bytes: 0x01 0x00
        // then

        // remote reads 0x10 two bytes: 0x01 0x00
        // reads from 0x20 (BoilerState) (but 32 bytes...)
        // remote writes to 0x90 (RemoteState) 16 bytes and then
        // then writes to 0x12 0x00 0x00 0x00 (unclear which side actually writes, assuming remote (bm1 e.g. data for bm2,3,4?))
        // then writes to 0x10 0x00 0x01
        critical_section::with(|cs| {
            let mut ram = RAM.borrow_ref_mut(cs);
            if let Some(boiler_state) = ram.get_boiler_state() {
                // info!("Got new boiler state: {:?}", boiler_state);
                // put current remote_state into ram:
                let remote_state = RemoteState {
                    power: REMOTE_POWER.load(core::sync::atomic::Ordering::Relaxed),
                    vl_soll2: REMOTE_VL_SOLL2.load(core::sync::atomic::Ordering::Relaxed),
                    ww_soll2: REMOTE_WW_SOLL2.load(core::sync::atomic::Ordering::Relaxed),
                    dummy: 0x01, // (count / 10) as u8,
                    stop_pump: REMOTE_STOP_PUMP.load(core::sync::atomic::Ordering::Relaxed),
                    dummy2: 1,
                    error: REMOTE_ERROR.load(core::sync::atomic::Ordering::Relaxed),
                    dummy3: [0; 7],
                    dummy4: 0xff,
                    checksum: 0xff, // checksum will be done via update_remote_state
                };
                ram.update_remote_state(&remote_state);
                ram.ram[0x12] = 0;
                ram.ram[0x13] = 0;
                ram.ram[0x14] = 0;
                ram.ram[RAM_BOILER_AVAILABLE_OFFSET] = 0; // mark as read
                ram.ram[RAM_REMOTE_AVAILABLE_OFFSET] = 1; // mark as available
                // todo the c code sets             ram[0x18] = 0x10; as well???
                let mut bs_ref = BOILER_STATE.borrow_ref_mut(cs);
                *bs_ref = Some((boiler_state, Instant::now()));
            }
        });

        let oe = OUTPUT_ENABLE.load(core::sync::atomic::Ordering::Relaxed);
        if oe && gpio0.is_set_low() {
            // todo optimize with previous state?
            gpio0.set_high();
        } else if !oe && gpio0.is_set_high() {
            gpio0.set_low();
        }
        Timer::after(Duration::from_millis(250)).await;
    }
}

struct BMRam {
    ram: [u8; 256],
}

const RAM_BOILER_AVAILABLE_OFFSET: usize = 0x10;
const RAM_REMOTE_AVAILABLE_OFFSET: usize = 0x11;
const RAM_BOILER_STATE_OFFSET: usize = 0x20;
const RAM_REMOTE_STATE_OFFSET: usize = 0x90;
const RAM_INIT_OFFSET: usize = 0xe0;
const RAM_MAGIC_OFFSET: usize = 0xfe;

impl Default for BMRam {
    fn default() -> Self {
        Self::new()
    }
}

impl BMRam {
    const fn new() -> Self {
        let mut ram = [0u8; 256];
        ram[RAM_MAGIC_OFFSET] = 0xFC;
        ram[RAM_MAGIC_OFFSET + 1] = 0x03;
        // Manual copy instead of copy_from_slice (which is not const)
        // "IF 04.00"
        ram[RAM_INIT_OFFSET] = 0x49;
        ram[RAM_INIT_OFFSET + 1] = 0x46;
        ram[RAM_INIT_OFFSET + 2] = 0x20;
        ram[RAM_INIT_OFFSET + 3] = 0x30;
        ram[RAM_INIT_OFFSET + 4] = 0x34;
        ram[RAM_INIT_OFFSET + 5] = 0x2E;
        ram[RAM_INIT_OFFSET + 6] = 0x30;
        ram[RAM_INIT_OFFSET + 7] = 0x30;
        BMRam { ram }
    }
}

impl BMRam {
    fn update_remote_state(&mut self, state: &RemoteState) {
        let offset = RAM_REMOTE_STATE_OFFSET;
        let bytes: &[u8; 16] = unsafe { core::mem::transmute(state) };
        self.ram[offset..offset + 16].copy_from_slice(bytes);

        // calculate checksum
        let mut checksum: u8 = 0;
        for b in &self.ram[offset..offset + 15] {
            crc8(&mut checksum, *b);
        }
        self.ram[offset + 15] = checksum;

        // to be done by the caller:
        //self.ram[RAM_REMOTE_AVAILABLE_OFFSET] = 1;
    }

    fn get_boiler_state(&mut self) -> Option<BoilerState> {
        if self.ram[RAM_BOILER_AVAILABLE_OFFSET] == 0 {
            return None;
        }
        let offset = RAM_BOILER_STATE_OFFSET;
        let bytes: &[u8; core::mem::size_of::<BoilerState>()] = &self.ram
            [offset..offset + core::mem::size_of::<BoilerState>()]
            .try_into()
            .unwrap();
        let boilerstate = unsafe { core::mem::transmute(*bytes) };
        // mark as read, should be done by the caller
        // self.ram[RAM_BOILER_AVAILABLE_OFFSET] = 0;

        Some(boilerstate)
    }
}

#[derive(defmt::Format, Copy, Clone)]
pub struct BoilerState {
    // e.g. 0x89 0x40 0x78 0x00 0x8A 0x4A 0x00 0x89 0x01 0x01 0x01 0x63 0x0F
    vl_max2: u8,      // 0x20 e.g. 0x89 = 137 = 68.5C
    vl_temp2: u8,     // 0x21 e.g. 0x40 = 64 = 32.0C
    dl_max2: u8,      // 0x22 e.g. 0x78 = 120 = 60.0C
    dl_max_temp2: u8, // 0x23 e.g. 0x00 = 0 = 0.0C
    ww_max2: u8,      // 0x24 e.g. 0x8a = 138 = 69.0C
    ww_temp2: u8,     // 0x25 e.g. 0x4a = 74 = 37.0C
    error: u8,        // 0x26 e.g. 0x00 = no error
    dummy1: u8,       // 0x27 e.g. 0x89 = 137 = ? (looks like a temp as well)
    dummy2: u8,       // 0x28 e.g. 0x01 = 1 = on/off?
    flame: u8,        // 0x29 e.g. 0x01 = 1 = flame on
    pump: u8,         // 0x2a e.g. 0x01 = 1 = pump on
    flags: u8,        // 0x2b BOILER_FLAG e.g. 0x63 = 0110_0011 ???
    dummy3: u8,
}
// assert size of BoilerState
const _: () = assert!(core::mem::size_of::<BoilerState>() == 13);

impl core::fmt::Debug for BoilerState {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("BoilerState")
            .field(
                "vl_max2",
                &format_args!("{} ({}°C)", self.vl_max2, self.vl_max2 as f32 * 0.5),
            )
            .field(
                "vl_temp2",
                &format_args!("{} ({}°C)", self.vl_temp2, self.vl_temp2 as f32 * 0.5),
            )
            .field(
                "dl_max2",
                &format_args!("{} ({}°C)", self.dl_max2, self.dl_max2 as f32 * 0.5),
            )
            .field(
                "dl_max_temp2",
                &format_args!(
                    "{} ({}°C)",
                    self.dl_max_temp2,
                    self.dl_max_temp2 as f32 * 0.5
                ),
            )
            .field(
                "ww_max2",
                &format_args!("{} ({}°C)", self.ww_max2, self.ww_max2 as f32 * 0.5),
            )
            .field(
                "ww_temp2",
                &format_args!("{} ({}°C)", self.ww_temp2, self.ww_temp2 as f32 * 0.5),
            )
            .field("error", &self.error)
            .field("dummy1", &self.dummy1)
            .field("dummy2", &self.dummy2)
            .field("flame", &self.flame)
            .field("pump", &self.pump)
            .field("flags", &self.flags)
            .finish()
    }
}

#[allow(unused)]
struct RemoteState {
    power: u8,     // 0x90
    vl_soll2: u8,  // 0x91
    ww_soll2: u8,  // 0x92
    dummy: u8,     // 0x93 - 1
    stop_pump: u8, // 0x94
    dummy2: u8,    // 0x95 - 1
    error: u8,
    dummy3: [u8; 7],
    dummy4: u8,
    checksum: u8, // 0x2c
}
// assert size of RemoteState
const _: () = assert!(core::mem::size_of::<RemoteState>() == 16);

// could as well use crate crc
// width=8  poly=0x39  init=0x00  refin=false  refout=false  xorout=0x00 check=0x8f  residue=0x00
const CRC8_TABLE: [u8; 256] = {
    let mut table = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = i as u8;
        let mut j = 0;
        while j < 8 {
            crc = (crc << 1) ^ (if crc & 0x80 != 0 { 0x39 } else { 0 });
            j += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
};

fn crc8(crc: &mut u8, m: u8) {
    *crc = CRC8_TABLE[(*crc ^ m) as usize];
}

// from https://www.mikrocontroller.net/attachment/highlight/590667
/*
#define BOILER_STATE_OFFSET 0x20
#define REMOTE_STATE_OFFSET 0x90
#define BOILER_AVAILABLE_OFFSET 0x10
#define REMOTE_AVAILABLE_OFFSET 0x11
// OFFSET 0x12 unknown. from i2c.txt: write to 0x50 ack data: 0x12 0x00 0x00 0x00 (unclear which side)
#define I2C_RAM_MAGIC_OFFSET 0xFE
#define I2C_INIT_OFFSET     0xe0

#define BOILER_FLAG_BUFFEROPERATION 8
#define BOILER_SIZE 13 (13 written, 32 read...?)
struct BoilerState
{
    uint8_t vlMax2;       // 0x20
    uint8_t vlTemp2;      // 0x21
    uint8_t dlMax2;       // 0x22
    uint8_t dlMaxTemp2;   // 0x23
    uint8_t wwMax2;       // 0x24
    uint8_t wwTemp2;      // 0x25
    uint8_t error;        // 0x26
    uint8_t dummy1;       // 0x27
    uint8_t dummy2;       // 0x28
    uint8_t flame;        // 0x29
    uint8_t pump;         // 0x2a
    uint8_t flags;        // 0x2b BOILER_FLAG
    uint8_t dummy3;
};
static BoilerState* pBoiler = (BoilerState*)(ram + BOILER_STATE_OFFSET);
#define REMOTE_SIZE 16
struct RemoteState
{
    uint8_t power; // 0x90
    uint8_t vlSoll2; // 0x91
    uint8_t wwSoll2; // 0x92
    uint8_t dummy;  // 0x93 - 1
    uint8_t stopPump; // 0x94
    uint8_t dummy2; // 0x95 - 1
    uint8_t error;
    uint8_t dummy3[7];
    uint8_t dummy4;
    uint8_t checksum;     // 0x2c
};
static RemoteState* pRemote = (RemoteState*)(ram + REMOTE_STATE_OFFSET);
static RemoteState rxRemoteState;


static void initRam()
{
    memset(ram, 0, sizeof(ram));
    ram[I2C_RAM_MAGIC_OFFSET] = 0xfc;
    ram[I2C_RAM_MAGIC_OFFSET + 1] = 0x03;
    byte initSequence[] = { 0x49, 0x46, 0x20, 0x30, 0x34, 0x2e, 0x30, 0x30 }; // "IF 04.00"
    memcpy(ram + I2C_INIT_OFFSET, initSequence, sizeof(initSequence));
}
static unsigned char crc8_table[256];
static void init_crc8()
{
    int i,j;
    unsigned char crc;
    for (i=0; i<256; i++) {
        crc = i;
        for (j=0; j<8; j++)
            crc = (crc << 1) ^ ((crc & 0x80) ? 0x39 : 0);
        crc8_table[i] = crc & 0xFF;
    }
}
void crc8(unsigned char *crc, unsigned char m)
{
    *crc = crc8_table[(*crc) ^ m];
    *crc &= 0xFF;
}

void processI2c()
{
    uint32_t m = millis();
    if (!ram[REMOTE_AVAILABLE_OFFSET] && rxRemoteState.power != 0) {
        static uint32_t lastAvailableMillis;
        if (!lastAvailableMillis) {
            lastAvailableMillis  = m;
        }
        if (m - lastAvailableMillis > 5) {
            lastAvailableMillis = 0;
            memcpy(pRemote, &rxRemoteState, sizeof(RemoteState));

            if (ram[BOILER_AVAILABLE_OFFSET]) {
                static BoilerState oldBoilerState;
                BoilerState avgBoilerState;
                static uint16_t vlTempX;
                static uint16_t wwTempX;
                if (!vlTempX) vlTempX = pBoiler->vlTemp2 << 5;
                if (!wwTempX) wwTempX = pBoiler->wwTemp2 << 9;
                vlTempX -= (vlTempX + 16) >> 5;
                vlTempX += pBoiler->vlTemp2;
                wwTempX -= (wwTempX + 256) >> 9;
                wwTempX += pBoiler->wwTemp2;
                avgBoilerState = *pBoiler;
                avgBoilerState.vlTemp2 = (vlTempX + 16) >> 5;
                avgBoilerState.wwTemp2 = (wwTempX + 256) >> 9;

                static uint32_t lastUpdateMillis;
                if (m - lastUpdateMillis > 5000 ||
                    m - lastUpdateMillis > 500 && memcmp(&avgBoilerState, &oldBoilerState, sizeof(BoilerState))) {
                    lastUpdateMillis = m;
                    oldBoilerState = avgBoilerState;

                    uint8_t buf[sizeof(BoilerState) + 1] = {0};
                    buf[0] = 0xf0;
                    memcpy(buf+1, &avgBoilerState, BOILER_SIZE);
                    radio.sendWithAck( GATEWAY, buf, BOILER_SIZE + 1);
                    radio.setMode(SX1231_OPMODE_RECEIVER);
                }
            }

            ram[REMOTE_AVAILABLE_OFFSET] = 1;
            ram[BOILER_AVAILABLE_OFFSET] = 0;
            ram[0x12] = ram[0x13] = ram[0x14] = 0;
            ram[0x18] = 0x10;
        }
    }
}
void JunkersSensor::loop()
{
    processI2c();

    uint8_t len = radio.receive( dataBuffer, SX1231_MAX_DATA_LEN );
    if( len > 0) {
        if ((uint8_t)dataBuffer[0] == 0xf0) {
            memcpy(&rxRemoteState, dataBuffer+1, sizeof(RemoteState));

            rxRemoteState.checksum = 0;
            for (uint8_t* p = (uint8_t*)&rxRemoteState; p != &rxRemoteState.checksum; p++) {
                crc8(&rxRemoteState.checksum,*p);
            }
        } else {
            checkForCommands( dataBuffer, len );
        }
    }
    Sensor::loop();
}
volatile uint8_t ram[256];

14:32:22 - junkers/remote: FFB4000100010000000000000000FF00
14:32:38 - junkers/boiler: 8C627400784D008C010101630F
14:32:52 - junkers/remote: FFB4000100010000000000000000FF00
14:33:09 - junkers/boiler: 8C627400784D008C010101630F
14:33:16 - junkers/boiler: 8C637400784D008C010101630F
14:33:22 - junkers/remote: FFB4000100010000000000000000FF00
14:33:36 - junkers/boiler: 8C647400784D008C010101630F
14:33:52 - junkers/remote: FFB4000100010000000000000000FF00
14:34:06 - junkers/boiler: 8C647400784D008C010101630F
14:34:09 - junkers/boiler: 8C657400784D008C010101630F
14:34:22 - junkers/remote: FF28000100010000000000000000FF00
14:34:40 - junkers/boiler: 8C657400784D008C010101630F
14:34:50 - junkers/boiler: 8C667400784D008C010101630F
14:34:52 - junkers/remote: FF28000100010000000000000000FF00
14:35:21 - junkers/boiler: 8C667400784D008C010101630F
14:35:22 - junkers/remote: FF28000100010000000000000000FF00
14:35:23 - junkers/boiler: 8C667400784D00280101014207
14:35:25 - junkers/boiler: 8C667400784D00280100014207
14:35:45 - junkers/boiler: 8C657400784D00280100014207
14:35:48 - junkers/boiler: 8C647400784D00280100014207
14:35:50 - junkers/boiler: 8C637400784D00280100014207
14:35:52 - junkers/remote: FF28000100010000000000000000FF00
14:35:56 - junkers/boiler: 8C627400784D00280100014207
14:35:58 - junkers/boiler: 8C617400784D00280100014207
14:36:01 - junkers/boiler: 8C607400784D00280100014207
14:36:04 - junkers/boiler: 8C5F7400784D00280100014207
14:36:08 - junkers/boiler: 8C5E7400784D00280100014207
14:36:11 - junkers/boiler: 8C5D7400784D00280100014207
14:36:16 - junkers/boiler: 8C5C7400784D00280100014207
14:36:22 - junkers/remote: FF28000100010000000000000000FF00
14:36:28 - junkers/boiler: 8C5B7400784D00280100014207
14:36:38 - junkers/boiler: 8C5A7400784D00280100014207
14:36:52 - junkers/remote: FF28000100010000000000000000FF00
14:36:55 - junkers/boiler: 8C597400784D00280100014207
14:37:22 - junkers/remote: FF28000100010000000000000000FF00
14:37:26 - junkers/boiler: 8C597400784D00280100014207
14:37:52 - junkers/remote: FF28000100010000000000000000FF00
14:37:56 - junkers/boiler: 8C597400784D00280100014207
14:38:22 - junkers/remote: FF28000100010000000000000000FF00
14:38:28 - junkers/boiler: 8C597400784D00280100014207
14:38:52 - junkers/remote: FF28000100010000000000000000FF00

junkers/remote = read/write to addr 0x90 RemoteState 16 bytes
19:09:40 - junkers/remote: FF 28 00 01 00 01 00 00 00 00 00 00 00 00 FF 00 // VL Soll 20C

junkers/boiler = write to addr 0x20 BoilerState 13 bytes
19:10:14 - junkers/boiler: 8C 3E 74 00 78 44 00 28 01 00 01 42 07   // VL Soll 20C = 40dec = 0x28hex
19:10:31 - junkers/remote: FF B4 000100010000000000000000FF00 // VL Soll 90C
19:10:32 - junkers/boiler: 8C3E7400784400 8C 01 00 01 42 07 // VL Soll 70C (8C) nach Max VL Limitierung
19:10:32 - junkers/boiler: 8C3E7400784400 8C 01 00 01 62 07 // Gas?
19:10:32 - junkers/boiler: 8C3E7400784400 8C 01 00 01 63 07 // Zündung?
19:10:36 - junkers/boiler: 8C3E7400784400 8C 01 01 01 63 0F // Flamme

Wenn die Therme anspringt dann probed sie auf den i2c Leitungen rum -
erst versucht sie 0x48, dann 0x50. Und so wie sie da einen Speicher
findet, der auf 0xfe - 0xff die richtigen Daten hat versucht sie den
Handshake zu beginnen. Dann denkt die Therme sie hätte einen BM1 - auch
wenn es statt dessen der Moteino ist.

PCF I2c addresses according to PCF8570 datasheet:
1 0 1 0 A2 A1 A0
2^6 + 2^4 + (0..=7) = 0x50 ..= 0x57
--> we simulate 0x50



*/
