#![no_std]
#![no_main]
#![deny(warnings)]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate kvs;
extern crate panic_halt;
extern crate rtic;
extern crate stm32g0xx_hal as hal;

use core::fmt::Write;
use core::str::from_utf8;
use hal::gpio::{gpioa::*, gpioc::*, *};
use hal::prelude::*;
use hal::serial::*;
use hal::spi;
use hal::stm32;
use hal::timer::{stopwatch::Stopwatch, *};
use hal::watchdog::*;
use kvs::adapters::paged::PagedAdapter;
use kvs::adapters::spi::{SpiAdapterConfig, SpiStoreAdapter};
use kvs::*;
use ushell::autocomplete::StaticAutocomplete;
use ushell::control;
use ushell::history::LRUHistory;
use ushell::{Input, ShellError, UShell};

pub use defmt_rtt as _;

const SHELL_PROMPT: &str = "$> ";
const CR: &str = "\r\n";
const HELP: &str = "\r\n\
uShell\r\n\r\n\
USAGE:\r\n\
\tcommand [arg]\r\n\r\n\
COMMANDS:\r\n\
\tls           List keys\r\n\
\tget key      Get key value\r\n\
\tdel key      Delete key\r\n\
\tset key val  Set key value\r\n\
\ton           Start animation\r\n\
\toff          Stop animation\r\n\
\tstatus       Get status\r\n\
\tclear        Clear screen\r\n\
\thelp         Print this message\r\n\r\n
CONTROL KEYS:\r\n\
\tCtrl+C       Reboot\r\n\
";

pub const KVS_MAGIC: u32 = 0x0d0d;
pub const KVS_BUCKETS: usize = 256;
pub const KVS_SLOTS: usize = 8;
pub const KVS_MAX_HOPS: usize = 64;

pub type ShellLink = Serial<stm32::USART2, BasicConfig>;
pub type Shell = UShell<ShellLink, StaticAutocomplete<9>, LRUHistory<32, 4>, 32>;
pub type StoreSPI = spi::Spi<hal::stm32::SPI1, (PA1<Analog>, PA6<Analog>, PA7<Analog>)>;
pub type StoreCS = PA5<hal::gpio::Output<PushPull>>;
pub type Store =
    KVStore<PagedAdapter<SpiStoreAdapter<StoreSPI, StoreCS, 3>, 256>, KVS_BUCKETS, KVS_SLOTS>;

#[rtic::app(device = hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        led: PC15<Output<OpenDrain>>,
        timer: Timer<stm32::TIM3>,
        store: Store,
        shell: Shell,
        blink_enabled: bool,
        watchdog: IndependedWatchdog,
        stopwatch: Stopwatch<stm32::TIM2>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        let mut rcc = ctx.device.RCC.freeze(hal::rcc::Config::pll());
        defmt::info!("booting...");

        let port_a = ctx.device.GPIOA.split(&mut rcc);
        let port_c = ctx.device.GPIOC.split(&mut rcc);

        let led = port_c.pc15.into_open_drain_output();
        let stopwatch = ctx.device.TIM2.stopwatch(&mut rcc);
        let watchdog = ctx.device.IWDG.constrain();

        let mut serial = ctx
            .device
            .USART2
            .usart(
                port_a.pa2,
                port_a.pa3,
                BasicConfig::default().baudrate(115_200.bps()),
                &mut rcc,
            )
            .expect("Failed to init serial port");
        serial.listen(Event::Rxne);
        write!(serial, "{0:}Booting...{0:}", CR).ok();

        let mut fram_spi_cs = port_a.pa5.into_push_pull_output();
        fram_spi_cs
            .set_high()
            .expect("Failed to control ChipSelect");
        let fram_spi = ctx.device.SPI1.spi(
            (port_a.pa1, port_a.pa6, port_a.pa7),
            spi::MODE_0,
            8.mhz(),
            &mut rcc,
        );

        let adapter_cfg = SpiAdapterConfig::new(131_072);
        let adapter = PagedAdapter::new(SpiStoreAdapter::new(fram_spi, fram_spi_cs, adapter_cfg));
        let store_cfg = StoreConfig::new(KVS_MAGIC, KVS_MAX_HOPS);
        let store = Store::open(adapter, store_cfg, true).expect("Failed to open store");

        let blink_enabled = true;
        let mut timer = ctx.device.TIM3.timer(&mut rcc);
        timer.start(2.hz());
        timer.listen();

        let autocomplete = StaticAutocomplete([
            "clear", "del", "get", "help", "ls", "off", "on", "set ", "status",
        ]);
        let history = LRUHistory::default();
        let mut shell = UShell::new(serial, autocomplete, history);
        shell.write_str(SHELL_PROMPT).ok();

        defmt::info!("starting...");
        init::LateResources {
            timer,
            led,
            shell,
            store,
            blink_enabled,
            watchdog,
            stopwatch,
        }
    }

    #[task(binds = TIM3, priority = 2, resources = [timer, led, blink_enabled])]
    fn timer_tick(ctx: timer_tick::Context) {
        let timer_tick::Resources {
            led,
            timer,
            blink_enabled,
        } = ctx.resources;
        if *blink_enabled {
            led.toggle().ok();
        } else {
            led.set_low().ok();
        }
        timer.clear_irq();
    }

    #[task(binds = USART2, priority = 1, resources = [shell, blink_enabled, store, stopwatch, watchdog])]
    fn serial_data(ctx: serial_data::Context) {
        let shell = ctx.resources.shell;
        let store = ctx.resources.store;
        let stopwatch = ctx.resources.stopwatch;
        let watchdog = ctx.resources.watchdog;
        let mut blink_enabled = ctx.resources.blink_enabled;

        loop {
            match shell.poll() {
                Ok(Some(Input::Command((cmd, args)))) => {
                    match cmd {
                        "help" => {
                            shell.write_str(HELP).ok();
                        }
                        "clear" => {
                            shell.clear().ok();
                        }
                        "on" => {
                            blink_enabled.lock(|e| *e = true);
                            shell.write_str(CR).ok();
                        }
                        "off" => {
                            blink_enabled.lock(|e| *e = false);
                            shell.write_str(CR).ok();
                        }
                        "status" => {
                            let on = blink_enabled.lock(|e| *e);
                            let status = if on { "on" } else { "off" };
                            write!(shell, "{0:}Animation: {1:}{0:}", CR, status).ok();
                        }
                        "ls" => {
                            shell.write_str(CR).ok();
                            for key_ref in store.keys() {
                                let key = core::str::from_utf8(key_ref.key()).unwrap();
                                write!(shell, "{} - {} bytes{}", key, key_ref.val_len(), CR).ok();
                            }
                        }
                        "reset-store" => {
                            store.reset().ok();
                            shell.write_str(CR).ok();
                        }
                        "fill-store" => {
                            for x in 0..20 {
                                for y in 0..10 {
                                    store.insert(&[b'a' + x, b'a' + y], b"lorem ipsum dolor sit amet, consectetur adipiscing elit").unwrap();
                                }
                            }
                            shell.write_str(CR).ok();
                        }
                        "get" | "cat" if !args.is_empty() => {
                            stopwatch.reset();
                            let now = stopwatch.now();
                            let mut scratch = [0_u8; 64];
                            match store.load(args.as_bytes(), &mut scratch) {
                                Ok(bucket) => {
                                    let val = &scratch[..bucket.val_len()];
                                    let val = from_utf8(val).unwrap_or("--mailformed--");
                                    let lookup_time_us = stopwatch.elapsed(now);
                                    write!(
                                        shell,
                                        "{0:}OK; lookup time: {1:} us{0:}{2:}{0:}",
                                        CR, lookup_time_us.0, val
                                    )
                                    .ok();
                                }
                                Err(err) => {
                                    let lookup_time_us = stopwatch.elapsed(now);
                                    write!(
                                        shell,
                                        "{0:}{1:?}; lookup time: {2:} us{0:}",
                                        CR, err, lookup_time_us.0
                                    )
                                    .ok();
                                }
                            }
                        }
                        "del" if !args.is_empty() => match store.remove(args.as_bytes()) {
                            Err(err) => {
                                write!(shell, "{0:}{1:?}{0:}", CR, err).ok();
                            }
                            _ => {
                                shell.write_str(CR).ok();
                            }
                        },
                        "set" => match args.split_once(" ") {
                            Some((key, val)) if !key.is_empty() && !val.is_empty() => {
                                stopwatch.reset();
                                let now = stopwatch.now();

                                match store.insert(key.as_bytes(), val.as_bytes()) {
                                    Ok(_) => {
                                        let insert_time_us = stopwatch.elapsed(now);
                                        write!(
                                            shell,
                                            "{0:}OK; insert time: {1:} us{0:}",
                                            CR, insert_time_us.0
                                        )
                                        .ok();
                                    }
                                    Err(err) => {
                                        write!(shell, "{0:}{1:?}{0:}", CR, err).ok();
                                    }
                                }
                            }
                            _ => {
                                write!(shell, "{0:}unsupported command{0:}", CR).ok();
                            }
                        },
                        "" => {
                            shell.write_str(CR).ok();
                        }
                        _ => {
                            write!(shell, "{0:}unsupported command{0:}", CR).ok();
                        }
                    }
                    shell.write_str(SHELL_PROMPT).ok();
                }
                Ok(Some(Input::Control(control::CTRL_C))) => {
                    defmt::info!("rebooting...");
                    watchdog.start(1.us());
                }
                Err(ShellError::WouldBlock) => break,
                _ => {}
            }
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }
};
