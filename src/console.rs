use defmt::{info, warn};
use embassy_net::{IpListenEndpoint, Stack, tcp::TcpSocket};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use nostd_interactive_terminal::prelude::*;

use crate::{homematic::RTC, regulator::MANUAL_OVERRIDE};

extern crate alloc;

// console task
#[embassy_executor::task]
pub async fn console_task(stack: Stack<'static>) {
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

        let console_port = 65457u16;
        info!("Starting console server socket on port {}...", console_port);

        let mut rx_buf = alloc::boxed::Box::new([0_u8; 128]);
        let mut tx_buf = alloc::boxed::Box::new([0_u8; 128]);
        let mut server = TcpSocket::new(stack, rx_buf.as_mut_slice(), tx_buf.as_mut_slice());

        let acc = server
            .accept(IpListenEndpoint {
                addr: None,
                port: console_port,
            })
            .await;
        match acc {
            Ok(()) => {
                info!("Console client connected!");
                let exit_reason = handle_client(&mut server).await;
                let _ = server.flush().await;
                server.abort();
                match exit_reason {
                    Ok(ExitReason::Restart) => {
                        info!("Console requested system restart!");
                        Timer::after(Duration::from_secs(1)).await;
                        esp_hal::system::software_reset();
                    }
                    Ok(ExitReason::Disconnected) => {
                        info!("Console client disconnected!");
                    }
                    _ => {}
                }
            }
            Err(e) => {
                info!("Console server accept error: {:?}", e);
            }
        }
        Timer::after(Duration::from_secs(5)).await;
    }
}

// MARK: nostd_interactive_terminal client handler

enum ExitReason {
    Disconnected,
    Exit,
    Restart,
}

async fn handle_client(socket: &mut TcpSocket<'_>) -> Result<ExitReason, embassy_net::tcp::Error> {
    let (mut rx, mut tx) = socket.split();

    let config = TerminalConfig {
        buffer_size: 128,
        prompt: "espbm1> ",
        echo: true,
        ansi_enabled: true,
    };
    let history = History::new(nostd_interactive_terminal::HistoryConfig::default());
    let mut reader =
        nostd_interactive_terminal::terminal::TerminalReader::<128>::new(config, Some(history));
    let mut writer = TerminalWriter::new(&mut tx, true);

    writer.clear_screen().await?;
    writer.writeln("=== esp-junkers-bm1 console ===").await?;
    writer
        .writeln("Type 'help' for available commands\n")
        .await?;
    // Main command loop
    loop {
        match reader
            .read_line(
                &mut rx,
                &mut writer,
                Option::<&Signal<NoopRawMutex, ()>>::None,
            )
            .await
        {
            Ok(command) => {
                // Parse command
                let parsed = match CommandParser::parse_simple::<4, 128>(&command) {
                    Ok(p) => p,
                    Err(_) => {
                        writer.write_error("Failed to parse command\n").await?;
                        continue;
                    }
                };
                match parsed.name() {
                    "help" => {
                        writer.writeln("Available commands:").await?;
                        writer.writeln("  help   - Show this message").await?;
                        writer.writeln("  exit   - Exit console").await?;
                        writer.writeln("  info   - Show system information").await?;
                        writer.writeln("  restart- Restart the system").await?;
                        writer
                            .writeln("  toggle-output - Toggle level shifter output")
                            .await?;
                        writer
                            .writeln("  toggle-homematic - Toggle homematic connections on/off")
                            .await?;
                        writer.writeln("  override - Set manual override. Args: <mins> (0 = off), [vl_soll (<0 to ignore)] [pump_onoff]").await?;
                        writer.writeln("  set_remote - Set static remote values. Args: <power> <dummy1> <dummy2>").await?;
                        writer
                            .writeln("  set_ww_soll2 - Set remote WW_SOLL2 value. Args: <ww_soll2>")
                            .await?;
                        writer.writeln("  STACK_TEST - Dangerous! Use amount of stack to be able to determine how much is available. Args: [amount in 1kb]. If no arg is used, it will consume all in 1k increments.").await?;
                    }
                    "exit" => {
                        writer.writeln("Exiting console...").await?;
                        break;
                    }
                    "restart" => {
                        writer.writeln("Restarting system...").await?;
                        return Ok(ExitReason::Restart); // caller will handle restart to ensure proper connection close
                    }
                    "toggle-output" => {
                        use crate::i2c::OUTPUT_ENABLE;
                        let old_oe = OUTPUT_ENABLE.fetch_not(core::sync::atomic::Ordering::Relaxed);
                        if !old_oe {
                            writer.writeln("Level shifter output ENABLED").await?;
                        } else {
                            writer.writeln("Level shifter output DISABLED").await?;
                        }
                    }
                    "toggle-homematic" => {
                        use crate::homematic::HMIP_CONNECTIONS_STOP;
                        let old =
                            HMIP_CONNECTIONS_STOP.fetch_not(core::sync::atomic::Ordering::Relaxed);
                        if old {
                            writer.writeln("Homematic connections ENABLED").await?;
                        } else {
                            writer.writeln("Homematic connections DISABLED").await?;
                        }
                    }
                    "set_ww_soll2" => {
                        use crate::i2c::REMOTE_WW_SOLL2;
                        let args = parsed.args;
                        if args.len() != 1 {
                            writer
                                .write_error("set_ww_soll2 command requires 1 argument\n")
                                .await?;
                            continue;
                        }
                        let ww_soll2: u8 = match args[0].parse() {
                            Ok(v) => v,
                            Err(_) => {
                                writer
                                    .write_error("Failed to parse ww_soll2 argument\n")
                                    .await?;
                                continue;
                            }
                        };
                        REMOTE_WW_SOLL2.store(ww_soll2, core::sync::atomic::Ordering::Relaxed);
                        writer.writeln("Remote WW_SOLL2 value updated").await?;
                    }
                    "set_remote" => {
                        use crate::i2c::{
                            REMOTE_DUMMY1,
                            REMOTE_DUMMY2,
                            REMOTE_POWER, // , REMOTE_WW_SOLL2,
                        };
                        // we expect 3 arguments: power, dummy1, dummy2
                        let args = parsed.args;
                        if args.len() < 3 {
                            writer
                                .write_error("set_remote command requires 3 arguments\n")
                                .await?;
                            continue;
                        }
                        let power: u8 = match args[0].parse() {
                            Ok(v) => v,
                            Err(_) => {
                                writer
                                    .write_error("Failed to parse power argument\n")
                                    .await?;
                                continue;
                            }
                        };
                        let dummy1: u8 = match args[1].parse() {
                            Ok(v) => v,
                            Err(_) => {
                                writer
                                    .write_error("Failed to parse dummy1 argument\n")
                                    .await?;
                                continue;
                            }
                        };
                        let dummy2: u8 = match args[2].parse() {
                            Ok(v) => v,
                            Err(_) => {
                                writer
                                    .write_error("Failed to parse dummy2 argument\n")
                                    .await?;
                                continue;
                            }
                        };
                        // set the REMOTE_... values
                        REMOTE_POWER.store(power, core::sync::atomic::Ordering::Relaxed);
                        //REMOTE_WW_SOLL2.store(ww_soll2, core::sync::atomic::Ordering::Relaxed);
                        REMOTE_DUMMY1.store(dummy1, core::sync::atomic::Ordering::Relaxed);
                        REMOTE_DUMMY2.store(dummy2, core::sync::atomic::Ordering::Relaxed);
                        writer.writeln("Remote values updated").await?;
                    }
                    "override" => {
                        // we expect at least 1 argument: minutes to activate override
                        let args = parsed.args;
                        if args.is_empty() {
                            writer
                                .write_error("override command requires at least 1 argument (duration in minutes)\n")
                                .await?;
                            continue;
                        }
                        let duration_mins: u32 = match args[0].parse() {
                            Ok(v) => v,
                            Err(_) => {
                                writer
                                    .write_error("Failed to parse duration argument\n")
                                    .await?;
                                continue;
                            }
                        };
                        let mut vl_soll2: Option<u8> = None;
                        let mut pump_onoff: Option<bool> = None;
                        if args.len() >= 2 {
                            // try parse as temperature
                            match args[1].parse::<i8>() {
                                Ok(t) if t >= 0 => {
                                    vl_soll2 = Some((t as u8) * 2);
                                }
                                Ok(_) => { // weirdly using if t < 0 doesn't work here... (non exhaustive patterns)
                                    // ignore
                                }
                                Err(_) => {
                                    writer
                                        .write_error("Failed to parse vl_soll argument\n")
                                        .await?;
                                    continue;
                                }
                            }
                        }
                        if args.len() >= 3 {
                            // try parse as bool
                            match args[2].to_lowercase().as_str() {
                                "on" | "1" | "true" => {
                                    pump_onoff = Some(true);
                                }
                                "off" | "0" | "false" => {
                                    pump_onoff = Some(false);
                                }
                                _ => {
                                    writer
                                        .write_error("Failed to parse pump_onoff argument\n")
                                        .await?;
                                    continue;
                                }
                            }
                        }
                        // set the MANUAL_OVERRIDE
                        let now = Instant::now();
                        MANUAL_OVERRIDE.lock(|mo| {
                            let mut mo = mo.borrow_mut();
                            if duration_mins == 0 {
                                // disable
                                mo.active_till = Instant::MIN;
                            } else {
                                mo.active_till =
                                    now + Duration::from_secs(duration_mins as u64 * 60);
                            }
                            mo.vl_soll2 = vl_soll2;
                            mo.pump_onoff = pump_onoff;
                        });
                        writer
                            .writeln(
                                &heapless::format!(
                                    128;
                                    "Manual override set for {} minutes",
                                    if duration_mins == 0 {
                                        0
                                    } else {
                                        duration_mins
                                    }
                                )
                                .unwrap_or_default(),
                            )
                            .await?;
                    }
                    "info" => {
                        let now = esp_hal::time::Instant::now();
                        writer.writeln("System Information:").await?;
                        writer.writeln(" Device: esp-junkers-bm1").await?;
                        // print up-time and current time from RTC:
                        {
                            let up_secs = now.duration_since_epoch().as_secs();
                            let hours = up_secs / 3600;
                            let mins = (up_secs % 3600) / 60;
                            let secs = up_secs % 60;
                            let current_time = RTC
                                .lock(|rtc| rtc.borrow().as_ref().map(|rtc| rtc.current_time_us()));
                            let current_time_str = if let Some(ct_us) = current_time {
                                if let Ok(ct) = jiff::Timestamp::from_microsecond(ct_us as i64) {
                                    alloc::string::ToString::to_string(&ct)
                                } else {
                                    alloc::string::ToString::to_string(&"N/A")
                                }
                            } else {
                                alloc::string::ToString::to_string(&"N/A")
                            };
                            writer
                                .write_info(
                                    &heapless::format!(
                                        128;
                                        " Uptime: {}h {}m {}s, current time: {}\n",
                                        hours, mins, secs, current_time_str.as_str()
                                    )
                                    .unwrap_or_default(),
                                )
                                .await?;
                        }
                        // add heap info:
                        {
                            let free_heap = esp_alloc::HEAP.free();

                            writer
                                .write_info(
                                    &heapless::format!(128; " Free heap: {} bytes\n", free_heap)
                                        .unwrap_or_default(),
                                )
                                .await?;
                            let heap_stats = MyHeapStats(esp_alloc::HEAP.stats());
                            writer
                                .write_info(
                                    &heapless::format!(512; " Heap stats: {}\n", heap_stats)
                                        .unwrap_or(
                                            heapless::format!(512;"failed to convert\n").unwrap(),
                                        ),
                                )
                                .await?;
                        }
                        // add stack info
                        {
                            // show stack size as distance of _stack_end_cpu0 and start_start_cpu0
                            // and compare it to current stack pointer
                            let (stack_size, stack_used) = stack_info();
                            writer
                                .write_info(
                                    &heapless::format!(
                                        256;
                                        " Stack size: {} bytes, used: {} bytes\n",
                                        stack_size, stack_used
                                    )
                                    .unwrap_or_default(),
                                )
                                .await?;
                        }
                        // homematic stopped?
                        {
                            use crate::homematic::HMIP_CONNECTIONS_STOP;
                            let stopped =
                                HMIP_CONNECTIONS_STOP.load(core::sync::atomic::Ordering::Relaxed);
                            if stopped {
                                writer
                                    .write_warning(" WARNING: Homematic connections stopped!\n")
                                    .await?;
                            }
                        }
                        // level shifter output enabled:
                        {
                            use crate::i2c::OUTPUT_ENABLE;
                            let oe = OUTPUT_ENABLE.load(core::sync::atomic::Ordering::Relaxed);
                            if oe {
                                writer
                                    .write_warning(" WARNING: Level shifter output is ENABLED!\n")
                                    .await?;
                            }
                        }
                        // last boiler state via i2c::BOILER_STATE:
                        {
                            use crate::i2c::BOILER_STATE;
                            let bs = critical_section::with(|cs| {
                                let ram = BOILER_STATE.borrow_ref(cs);
                                if let Some((boiler_state, update_instant)) = ram.as_ref() {
                                    Some((*boiler_state, *update_instant))
                                } else {
                                    None
                                }
                            });
                            if let Some((boiler_state, timestamp)) = bs {
                                let age = now
                                    .duration_since_epoch()
                                    .saturating_sub(timestamp.duration_since_epoch());
                                writer
                                    .write_info(
                                        &heapless::format!(
                                            512;
                                            " Last boiler state update: {}s ago\n  {:?}\n",
                                            age.as_secs(),
                                            boiler_state
                                        )
                                        .unwrap_or_default(),
                                    )
                                    .await?;
                            } else {
                                writer.write_warning(" No boiler state available\n").await?;
                            }
                        }
                        // current homematic DEVICES:
                        {
                            use crate::homematic::DEVICES;
                            let mut device_info_str: heapless::String<64> = heapless::String::new();
                            let mut device_infos: alloc::vec::Vec<heapless::String<128>> =
                                alloc::vec::Vec::new();
                            DEVICES.lock(|devices| {
                                let devices = devices.borrow();
                                device_info_str = heapless::format!(64; " Homematic devices: {}\n", devices.len())
                                        .unwrap_or_default();
                                for device in devices.iter() {
                                    let info_str = heapless::format!(128;
                                        "   '{}': {}/{}C, valve: {}, {}\n",
                                        device.label,
                                        device.valve_act_temp,
                                        device.set_point_temp,
                                        device.valve_position,
                                        alloc::string::ToString::to_string(&jiff::Timestamp::from_millisecond(device.last_status_update).unwrap_or_default())
                                    )
                                    .unwrap_or_default();
                                    device_infos.push(info_str);
                                }
                            });
                            writer.write_info(&device_info_str).await?;
                            for device in device_infos.iter() {
                                writer.write_info(device).await?;
                            }
                        }
                        // current remote values:
                        {
                            use crate::i2c::{
                                REMOTE_DUMMY1, REMOTE_DUMMY2, REMOTE_POWER, REMOTE_STOP_PUMP,
                                REMOTE_VL_SOLL2, REMOTE_WW_SOLL2,
                            };
                            let power = REMOTE_POWER.load(core::sync::atomic::Ordering::Relaxed);
                            let vl_soll2 =
                                REMOTE_VL_SOLL2.load(core::sync::atomic::Ordering::Relaxed);
                            let ww_soll2 =
                                REMOTE_WW_SOLL2.load(core::sync::atomic::Ordering::Relaxed);
                            let stop_pump =
                                REMOTE_STOP_PUMP.load(core::sync::atomic::Ordering::Relaxed);
                            let dummy1 = REMOTE_DUMMY1.load(core::sync::atomic::Ordering::Relaxed);
                            let dummy2 = REMOTE_DUMMY2.load(core::sync::atomic::Ordering::Relaxed);
                            writer
                                .write_info(
                                    &heapless::format!(256;
                                        " Remote settings:\n  Power: {}\n  VL_SOLL2: {} ({}°C)\n  WW_SOLL2: {} ({}°C)\n  STOP_PUMP: {}\n  DUMMY1: {}\n  DUMMY2: {}\n",
                                        power, vl_soll2, vl_soll2 as f32 * 0.5, ww_soll2, ww_soll2 as f32 * 0.5, stop_pump, dummy1, dummy2
                                    )
                                    .unwrap_or_default(),
                                )
                                .await?;
                        }
                    }
                    "STACK_TEST" => {
                        let args = &parsed.args;
                        let mut amount: usize = usize::MAX;
                        if !args.is_empty() {
                            match args[0].parse::<usize>() {
                                Ok(v) => {
                                    amount = v;
                                }
                                Err(_) => {
                                    writer
                                        .write_error("Failed to parse amount argument\n")
                                        .await?;
                                    continue;
                                }
                            }
                        }
                        writer
                            .writeln("Starting STACK_TEST (this may take a while)...")
                            .await?;
                        consume_stack(amount);
                        writer.writeln("STACK_TEST completed").await?;
                    }
                    _ => {
                        {
                            let mut msg: heapless::String<64> = heapless::String::new();
                            msg.push_str("Unknown command: '").unwrap();
                            msg.push_str(parsed.name()).unwrap(); // TODO!
                            msg.push_str("'\n").unwrap();
                            writer.write_error(&msg).await?;
                        }
                        writer.writeln("Type 'help' for available commands").await?;
                    }
                }
            }
            Err(_) => {
                // TODO seems to lead to busy loop in read_line / WDG reset! -> fix/provide PR!
                info!("Console client disconnected (read error)");
                Timer::after(Duration::from_secs(5)).await;
                // writer.write_error("Error reading line\n").await?;
                return Ok(ExitReason::Disconnected);
            }
        }
    }
    Ok(ExitReason::Exit)
}

fn consume_stack(chunks: usize) -> usize {
    // create a large array on the stack
    // do something with it to avoid optimization
    if chunks == 0 {
        0
    } else {
        const CHUNK_SIZE: usize = 1024; // 1k, we do seem to consume 1088 bytes per iteration, so 1088-1024 = 64 bytes overhead (stack frame?)
        let stack_info = stack_info();
        warn!(
            "Consuming {}/{} bytes of stack {}/{}...",
            CHUNK_SIZE, chunks, stack_info.0, stack_info.1
        );
        // delay a bit to allow defmt to flush
        // we need to do it here by busy looping as we are on stack already
        let wait_till = esp_hal::time::Instant::now() + esp_hal::time::Duration::from_millis(10);
        while esp_hal::time::Instant::now() < wait_till {}

        let buffer: [u8; CHUNK_SIZE] = [0u8; CHUNK_SIZE];
        let used = CHUNK_SIZE + consume_stack(chunks.saturating_sub(1));
        let sum: u8 = buffer.iter().sum();
        core::hint::black_box(sum);
        used
    }
}

struct MyHeapStats(esp_alloc::HeapStats);

impl core::fmt::Display for MyHeapStats {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let hs = &self.0;
        writeln!(f, "  Current usage: {} / {}", hs.current_usage, hs.size)?;
        for region in hs.region_stats.iter() {
            if let Some(region) = region.as_ref() {
                write!(f, "  ")?;
                region.fmt(f)?;
                writeln!(f)?;
            }
        }
        Ok(())
    }
}

fn stack_info() -> (usize, usize) {
    unsafe extern "C" {
        static mut _stack_start_cpu0: u8; // ptr to start of stack
        static mut _stack_end_cpu0: u8; // ptr to end of stack
    }
    let sp: usize;
    unsafe {
        core::arch::asm!("mv {0}, sp", out(reg) sp, options(nomem, nostack),);
    }
    let stack_start = core::ptr::addr_of!(_stack_start_cpu0) as usize;
    let stack_end = core::ptr::addr_of!(_stack_end_cpu0) as usize;
    let stack_size = stack_start.saturating_sub(stack_end);
    let stack_used = stack_start.saturating_sub(sp);
    (stack_size, stack_used)
}
