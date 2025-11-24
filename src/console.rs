use defmt::info;
use embassy_net::{IpListenEndpoint, Stack, tcp::TcpSocket};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use nostd_interactive_terminal::prelude::*;

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
                let _ = handle_client(&mut server).await;
                let _ = server.flush().await;
                server.abort();
                info!("Console client disconnected!");
            }
            Err(e) => {
                info!("Console server accept error: {:?}", e);
            }
        }
        Timer::after(Duration::from_secs(5)).await;
    }
}

// MARK: nostd_interactive_terminal client handler

async fn handle_client(socket: &mut TcpSocket<'_>) -> Result<(), embassy_net::tcp::Error> {
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
    writer.writeln("=== esp-junkers-bm1 console ===\r").await?;
    writer
        .writeln("Type 'help' for available commands\r")
        .await?;
    writer.writeln("\r").await?;

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
                        writer.write_error("Failed to parse command\r\n").await?;
                        continue;
                    }
                };
                match parsed.name() {
                    "help" => {
                        writer.writeln("Available commands:\r").await?;
                        writer.writeln("  help   - Show this message\r").await?;
                        writer.writeln("  exit   - Exit console\r").await?;
                        writer
                            .writeln("  info   - Show system information\r")
                            .await?;
                    }
                    "exit" => {
                        writer.writeln("Exiting console...\r").await?;
                        break;
                    }
                    "info" => {
                        writer.writeln("System Information:\r").await?;
                        writer.writeln(" Device: esp-junkers-bm1\r").await?;
                        // add heap info:
                        {
                            let free_heap = esp_alloc::HEAP.free();

                            writer
                                .write_info(
                                    &heapless::format!(128; " Free heap: {} bytes\r\n", free_heap)
                                        .unwrap_or_default(),
                                )
                                .await?;
                            let heap_stats = MyHeapStats(esp_alloc::HEAP.stats());
                            writer
                                .write_info(
                                    &heapless::format!(512; " Heap stats: {}\r\n", heap_stats)
                                        .unwrap_or(
                                            heapless::format!(512;"failed to convert\r\n").unwrap(),
                                        ),
                                )
                                .await?;
                        }
                    }
                    _ => {
                        {
                            let mut msg: heapless::String<64> = heapless::String::new();
                            msg.push_str("Unknown command: '").unwrap();
                            msg.push_str(parsed.name()).unwrap(); // TODO!
                            msg.push_str("'\r\n").unwrap();
                            writer.write_error(&msg).await?;
                        }
                        writer
                            .writeln("Type 'help' for available commands\r")
                            .await?;
                    }
                }
            }
            Err(_) => {
                // TODO seems to lead to busy loop in read_line / WDG reset! -> fix/provide PR!
                info!("Console client disconnected (read error)");
                Timer::after(Duration::from_secs(5)).await;
                // writer.write_error("Error reading line\r\n").await?;
                break; // TODO exist on error?
            }
        }
    }
    Ok(())
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
