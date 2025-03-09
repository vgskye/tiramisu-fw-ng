#![no_std]
#![no_main]

mod app;
mod fmt;
mod handlers;

use app::AppTx;
use embassy_usb::UsbDevice;
use esb_embassy::{Packet, Radio};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
use postcard_rpc::{
    sender_fmt,
    server::{Dispatch, Sender, Server},
};
use static_cell::StaticCell;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts,
    pac::{self, radio::vals::Txpower},
    peripherals, rng, saadc, twim, usb,
};
use embassy_nrf::{
    config::{DcdcConfig, HfclkSource, Reg0Voltage},
    usb::vbus_detect::HardwareVbusDetect,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use fmt::info;

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
    RNG => rng::InterruptHandler<peripherals::RNG>;
    RADIO => esb_embassy::InterruptHandler<peripherals::RADIO>;
});

fn get_unique_id() -> u64 {
    let lower = pac::FICR.deviceid(0).read() as u64;
    let upper = pac::FICR.deviceid(1).read() as u64;
    (upper << 32) | lower
}

fn usb_config(serial: &'static str) -> embassy_usb::Config<'static> {
    let mut config = embassy_usb::Config::new(0x1209, 0x0002);
    config.manufacturer = Some("SkyeLabs");
    config.product = Some("Mascarpone");
    config.serial_number = Some(serial);

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    config
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut c = embassy_nrf::config::Config::default();
    c.hfclk_source = HfclkSource::ExternalXtal;
    c.dcdc = DcdcConfig {
        reg0: false,
        reg0_voltage: Some(Reg0Voltage::_3V0),
        reg1: true,
    };
    let p = embassy_nrf::init(c);
    #[allow(clippy::drop_non_drop)]
    drop(p.TIMER0);
    info!(
        "{:08x}{:08x}",
        pac::FICR.deviceid(1).read(),
        pac::FICR.deviceid(0).read()
    );

    let mut radio = Radio::new(
        p.RADIO,
        Irqs,
        esb_embassy::Config {
            use_fast_ramp_up: false,
            tx_output_power: Txpower::POS8_DBM,
            ..esb_embassy::Config::default()
        },
    );

    radio.set_base_address_0(*b"Cake");
    radio.set_prefixes(*b"12345678");

    // Obtain the device ID
    let unique_id = get_unique_id();

    let mut rand = embassy_nrf::rng::Rng::new(p.RNG, Irqs);
    rand.set_bias_correction(true);

    let mut devid = [0u8; 4];

    rand.fill_bytes(&mut devid).await;

    radio.set_base_address_1(devid);

    static SERIAL_STRING: StaticCell<[u8; 16]> = StaticCell::new();
    let mut ser_buf = [b' '; 16];
    // This is a simple number-to-hex formatting
    unique_id
        .to_be_bytes()
        .iter()
        .zip(ser_buf.chunks_exact_mut(2))
        .for_each(|(b, chs)| {
            let mut b = *b;
            for c in chs {
                *c = match b >> 4 {
                    v @ 0..10 => b'0' + v,
                    v @ 10..16 => b'A' + (v - 10),
                    _ => b'X',
                };
                b <<= 4;
            }
        });
    let ser_buf = SERIAL_STRING.init(ser_buf);
    let ser_buf = core::str::from_utf8(ser_buf.as_slice()).unwrap();

    // USB/RPC INIT
    let driver = usb::Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));
    let pbufs = app::PBUFS.take();
    let config = usb_config(ser_buf);

    let context = app::Context { unique_id };

    let (device, tx_impl, rx_impl) =
        app::STORAGE.init_poststation(driver, config, pbufs.tx_buf.as_mut_slice());
    let dispatcher = app::MyApp::new(context, spawner.into());
    let vkk = dispatcher.min_key_len();
    let mut server: app::AppServer = Server::new(
        tx_impl,
        rx_impl,
        pbufs.rx_buf.as_mut_slice(),
        dispatcher,
        vkk,
    );
    let sender = server.sender();
    // We need to spawn the USB task so that USB messages are handled by
    // embassy-usb
    spawner.must_spawn(usb_task(device));
    spawner.must_spawn(esb_recv_task(radio, sender.clone(), devid));

    // Begin running!
    loop {
        // If the host disconnects, we'll return an error here.
        // If this happens, just wait until the host reconnects
        let err = server.run().await;
        #[cfg(feature = "defmt")]
        match err {
            postcard_rpc::server::ServerError::TxFatal(err) => {
                info!("TX error: {}", defmt::Debug2Format(&err))
            }
            postcard_rpc::server::ServerError::RxFatal(err) => {
                info!("RX error: {}", defmt::Debug2Format(&err))
            }
        }
        Timer::after_millis(100).await;
    }
}

/// This handles the low level USB management
#[embassy_executor::task]
pub async fn esb_recv_task(
    mut radio: Radio<'static, embassy_nrf::peripherals::RADIO>,
    sender: Sender<AppTx>,
    base_addr: [u8; 4],
) {
    let mut trackers: [Option<[u8; 8]>; 256] = [None; 256];
    let mut first_available_id = 0;
    let mut seq: u32 = 0;
    loop {
        let mut pkt = [Packet::new(); 8];
        pkt[0] = if let Ok(packet) = postcard::serialize_with_flavor(
            &icd::PairResponse {
                base_addr,
                assigned_id: first_available_id,
                protocol_key: icd::CURRENT_PROTOCOL_KEY,
            },
            PacketPostcardWriter(Packet::new()),
        ) {
            packet
        } else {
            info!("Failed to serialize pair resp to packet");
            continue;
        };
        info!("entering recv");
        let (recvd, channel, crc, rssi) = radio.try_recv(3, &mut pkt).await;
        info!("packet recv: {:?} {} {}", &recvd as &[u8], channel, crc);
        if channel == 0 {
            if let Ok(result) = postcard::from_bytes::<icd::PairRequest>(&recvd) {
                if result.protocol_key == icd::CURRENT_PROTOCOL_KEY {
                    trackers[first_available_id as usize] = Some(result.device_id);
                    first_available_id += 1;
                    info!("Paired: {:016X}", u64::from_le_bytes(result.device_id));
                } else {
                    info!("Tracker has different protocol version!");
                }
            } else {
                info!("Failed to deserialize pair req");
            }
        } else if channel == 1 {
            if let Ok(result) = postcard::from_bytes::<icd::DataFrame>(&recvd) {
                if let Some(device_id) = trackers[result.device_id as usize] {
                    _ = sender
                        .publish::<icd::dongle::DongleData>(
                            seq.into(),
                            &icd::TrackerDataFrame {
                                device_id,
                                battery_level: result.battery_level,
                                rssi,
                                accel: [
                                    (result.accel[0] << 6) as i16,
                                    (result.accel[1] << 6) as i16,
                                    (result.accel[2] << 6) as i16,
                                ],
                                quat: result.quat,
                                temp: result.temp,
                            },
                        )
                        .await;
                    seq += 1;
                } else {
                    info!("unpaired device acting paired?");
                }
            } else {
                info!("failed to decipher data");
            }
        }
    }
}

/// This handles the low level USB management
#[embassy_executor::task]
pub async fn usb_task(mut usb: UsbDevice<'static, app::AppDriver>) {
    usb.run().await;
}

struct PacketPostcardWriter(Packet);

impl postcard::ser_flavors::Flavor for PacketPostcardWriter {
    type Output = Packet;

    fn try_push(&mut self, data: u8) -> postcard::Result<()> {
        self.try_extend(&[data])
    }

    fn finalize(self) -> postcard::Result<Self::Output> {
        Ok(self.0)
    }

    fn try_extend(&mut self, data: &[u8]) -> postcard::Result<()> {
        let off = self.0.len() as usize;
        let new_len = self.0.len() as usize + data.len();
        if new_len > 255 {
            return Err(postcard::Error::SerializeBufferFull);
        }
        self.0.set_len(new_len as u8);
        self.0[off..].copy_from_slice(data);
        Ok(())
    }
}
