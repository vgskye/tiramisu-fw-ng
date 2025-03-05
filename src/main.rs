#![no_std]
#![no_main]

mod app;
mod fmt;
mod handlers;

use core::{convert::Infallible, f32::consts::PI};

use app::AppTx;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_usb::UsbDevice;
use esb_embassy::{Packet, Radio};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
use postcard_rpc::{
    sender_fmt,
    server::{Dispatch, Sender, Server},
};
use static_cell::StaticCell;
use vqf_rs::{Params, VQF};
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_nrf::{
    bind_interrupts, gpio::{self, Pin}, pac, peripherals,
    ppi::ConfigurableChannel,
    saadc, twim,
    usb::{self, vbus_detect::VbusDetect},
};
use embassy_nrf::{
    config::{DcdcConfig, HfclkSource, Reg0Voltage},
    usb::vbus_detect::HardwareVbusDetect,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use fmt::{info, unwrap};

const ADDRESS: u8 = 0b1101010;

const GYRO_SENSITIVITY: f32 = 35.0 / 1000.0;
const GSCALE: f32 = GYRO_SENSITIVITY * (PI / 180.0);

const ACCEL_SENSITIVITY: f32 = 0.244 / 1000.0;

const UNPAIR_TIMEOUT: Duration = Duration::from_secs(5);
const PAIR_RETRY_INTERVAL: Duration = Duration::from_secs(1);
const SLEEP_TIMEOUT: Duration = Duration::from_secs(30);

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
    SAADC => saadc::InterruptHandler;
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
    RADIO => esb_embassy::InterruptHandler<peripherals::RADIO>;
});

fn get_unique_id() -> u64 {
    let lower = pac::FICR.deviceid(0).read() as u64;
    let upper = pac::FICR.deviceid(1).read() as u64;
    (upper << 32) | lower
}

fn usb_config(serial: &'static str) -> embassy_usb::Config<'static> {
    let mut config = embassy_usb::Config::new(0x1209, 0x0001);
    config.manufacturer = Some("SkyeLabs");
    config.product = Some("Tiramisu");
    config.serial_number = Some(serial);

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    config
}

static USB_SIG: Signal<CriticalSectionRawMutex, ()> = Signal::new();

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

    let mut radio = Radio::new(p.RADIO, Irqs, esb_embassy::Config {
        tx_output_power: embassy_nrf::radio::TxPower::POS8_DBM,
        ..esb_embassy::Config::default()
    });

    radio.set_base_address_0(*b"Cake");
    radio.set_base_address_1(*b"Cafe");
    radio.set_prefixes(*b"12345678");

    // Obtain the device ID
    let unique_id = get_unique_id();

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
    let ser_buf = unsafe { core::str::from_utf8_unchecked(ser_buf.as_slice()) };

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
    // spawner.must_spawn(esb_recv_task(radio, sender.clone()));
    spawner.must_spawn(vqf_task(
        p.TWISPI0, p.P0_13, p.P0_15, p.SAADC, p.P0_17, sender, radio, p.PPI_CH0, p.PPI_CH1,
        p.PPI_CH2, Duration::from_millis(unique_id & 0b1111)
    ));

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
        if !HardwareVbusDetect::new(Irqs).is_usb_detected() {
            USB_SIG.wait().await;
        }
        Timer::after_millis(500).await;
    }
}

trait Imu {
    type Error;
    async fn init(&mut self) -> Result<(), Self::Error>;
    async fn bulk_read(
        &mut self,
        gyro_sample: &mut impl FnMut([f32; 3]),
        accel_sample: &mut impl FnMut([f32; 3])
    ) -> Result<(), Self::Error>;
    async fn prepare_wake(&mut self) -> Result<(), Self::Error>;
}

struct LSM6DSV<'d, T: twim::Instance> {
    twim: twim::Twim<'d, T>,
    int1: gpio::AnyPin
}

impl<T: twim::Instance> Imu for LSM6DSV<'_, T> {
    type Error = twim::Error;

    async fn init(&mut self) -> Result<(), Self::Error> {
        self.twim.write(ADDRESS, &[0x62, 0b00]).await?; // HAODR_CFG to set 00
        self.twim.write(ADDRESS, &[0x08, 0]).await?; // FIFO_CTRL2
        self.twim.write(ADDRESS, &[0x10, 0b0010110]).await?; // accel ODR: OP_MODE_XL to HAODR, ODR_XL to 120Hz
        self.twim.write(ADDRESS, &[0x11, 0b0011000]).await?; // gyro ODR: OP_MODE_G to HAODR, ODR_G to 480Hz
        self.twim.write(ADDRESS, &[0x12, (1 << 6) | (1 << 2)]).await?; // BDU to 1, IF_INC to 1
        self.twim.write(ADDRESS, &[0x15, 0b0011]).await?; // gyro range: FS_G to ±1000 dps
        self.twim.write(ADDRESS, &[0x17, 0b10]).await?; // accel range: FS_XL to ±8 g
        self.twim.write(ADDRESS, &[0x09, (0b1000) | (0b1000 << 4)]).await?;
        self.twim.write(ADDRESS, &[0x0a, 0b110]).await?; // FIFO mode: continuous mode
        self.twim.write(ADDRESS, &[0x13, 0b10]).await?; // DRDY_PULSED to 1
        self.twim.write(ADDRESS, &[0x0d, 0b1]).await?; // INT1_CTRL: INT1_DRDY_XL to 1
        Ok(())
    }

    async fn bulk_read(
        &mut self,
        gyro_sample: &mut impl FnMut([f32; 3]),
        accel_sample: &mut impl FnMut([f32; 3])
    ) -> Result<(), Self::Error> {
        #[unsafe(link_section = "data")]
        static HEX_1B: [u8; 1] = [0x1b];
        #[unsafe(link_section = "data")]
        static HEX_78: [u8; 1] = [0x78];
        let mut size_buf = [0u8; 2];
        let mut buf = [0u8; 7 * 16];
        loop {
            self.twim.write_read(ADDRESS, &HEX_1B, &mut size_buf).await?;
            let len = (size_buf[0] as usize) | (((size_buf[1] & 0b1) as usize) << 8);
            let read_size = (7 * (len + 1)).clamp(0, 16 * 7);
            self.twim.write_read(ADDRESS, &HEX_78, &mut buf[..read_size]).await?;
            let buf: &[[u8; 7]; 16] = zerocopy::transmute_ref!(&buf);
            for entry in &buf[..(len + 1)] {
                let tag = entry[0] >> 3;
                match tag {
                    0x00 => {
                        return Ok(())
                    },
                    0x01 => {
                        let x = i16::from_le_bytes([entry[1], entry[2]]);
                        let y = i16::from_le_bytes([entry[3], entry[4]]);
                        let z = i16::from_le_bytes([entry[5], entry[6]]);
                        gyro_sample([GSCALE * x as f32, GSCALE * y as f32, GSCALE * z as f32])
                    }
                    0x02 => {
                        let x = i16::from_le_bytes([entry[1], entry[2]]);
                        let y = i16::from_le_bytes([entry[3], entry[4]]);
                        let z = i16::from_le_bytes([entry[5], entry[6]]);
                        accel_sample([
                            ACCEL_SENSITIVITY * x as f32,
                            ACCEL_SENSITIVITY * y as f32,
                            ACCEL_SENSITIVITY * z as f32,
                        ])
                    }
                    x => info!("unknown tag: {=u8:02X}", x),
                }
            }
        }
    }

    async fn prepare_wake(&mut self) -> Result<(), Self::Error> {
        self.twim.write(ADDRESS, &[0x0d, 0]).await?; // INT1_CTRL: all cleared
        self.twim.write(ADDRESS, &[0x10, 0b1000110]).await?; // accel ODR: OP_MODE_XL to low power 1, ODR_XL to 120Hz
        self.twim.write(ADDRESS, &[0x11, 0b0000000]).await?; // gyro ODR: power down
        self.twim.write(ADDRESS, &[0x17, 0b10]).await?; // accel range: FS_XL to ±8 g
        self.twim.write(ADDRESS, &[0x18, 0b10]).await?; // USR_OFF_W to 2^-6 g/LSB
        self.twim.write(ADDRESS, &[0x56, 0b10000]).await?; // SLOPE_FDS to 1
        self.twim.write(ADDRESS, &[0x5b, 0b010000]).await?; // WK_THS to 4, USR_OFF_ON_WU to 0
        self.twim.write(ADDRESS, &[0x5c, 0b1100000]).await?; // WAKE_DUR to 3
    
        embassy_time::Timer::after_millis(11).await; // wait for accel to settle

        let port = match self.int1.port() {
            gpio::Port::Port0 => pac::P0,
            gpio::Port::Port1 => pac::P1,
        };

        port.pin_cnf(self.int1.pin() as usize).write(|w| {
            w.set_dir(pac::gpio::vals::Dir::INPUT);
            w.set_input(pac::gpio::vals::Input::CONNECT);
            w.set_pull(pac::gpio::vals::Pull::DISABLED);
            w.set_sense(pac::gpio::vals::Sense::HIGH);
        });
    
        info!("gn!");
        self.twim.write(ADDRESS, &[0x50, 0b10000000]).await?; // FUNCTIONS_ENABLE: INTERRUPTS_ENABLE to 1
        self.twim.write(ADDRESS, &[0x5e, 0b100000]).await?; // MD1_CFG: INT1_WU to 1
        Ok(())
    }
}

#[allow(clippy::too_many_arguments)]
#[embassy_executor::task]
pub async fn vqf_task(
    twispi0: embassy_nrf::peripherals::TWISPI0,
    p0_13: embassy_nrf::peripherals::P0_13,
    p0_15: embassy_nrf::peripherals::P0_15,
    saadc: embassy_nrf::peripherals::SAADC,
    p0_17: embassy_nrf::peripherals::P0_17,
    sender: Sender<AppTx>,
    mut radio: Radio<'static, embassy_nrf::peripherals::RADIO>,
    mut ppi_ch0: embassy_nrf::peripherals::PPI_CH0,
    mut ppi_ch1: embassy_nrf::peripherals::PPI_CH1,
    mut ppi_ch2: embassy_nrf::peripherals::PPI_CH2,
    desync_factor: Duration,
) {
    info!("Initializing TWI...");
    let mut config = twim::Config::default();
    config.frequency = twim::Frequency::K400;
    let mut twi = twim::Twim::new(twispi0, Irqs, p0_13, p0_15, config);

    let mut config = saadc::Config::default();
    config.oversample = saadc::Oversample::OVER8X;
    let mut channel = saadc::ChannelConfig::single_ended(saadc::VddhDiv5Input);
    channel.gain = saadc::Gain::GAIN1_2;
    let mut saadc = saadc::Saadc::new(saadc, Irqs, config, [channel]);
    saadc.calibrate().await;

    let mut pair_data = attempt_pair(&mut radio, (&mut ppi_ch0, &mut ppi_ch1, &mut ppi_ch2)).await;

    let mut last_pair_attempt = Instant::now();

    let mut last_successful_tx = Instant::now();

    info!("Resetting IMU...");
    unwrap!(twi.write(ADDRESS, &[0x12, 0b1]).await);
    info!("Waiting for reset...");
    Timer::after(Duration::from_millis(30)).await;

    info!("Configuring...");
    unwrap!(twi.write(ADDRESS, &[0x62, 0b00]).await); // HAODR_CFG to set 00
    unwrap!(twi.write(ADDRESS, &[0x08, 0]).await); // FIFO_CTRL2
    unwrap!(twi.write(ADDRESS, &[0x10, 0b0010110]).await); // accel ODR: OP_MODE_XL to HAODR, ODR_XL to 120Hz
    unwrap!(twi.write(ADDRESS, &[0x11, 0b0011000]).await); // gyro ODR: OP_MODE_G to HAODR, ODR_G to 480Hz
    unwrap!(twi.write(ADDRESS, &[0x12, (1 << 6) | (1 << 2)]).await); // BDU to 1, IF_INC to 1
    unwrap!(twi.write(ADDRESS, &[0x15, 0b0011]).await); // gyro range: FS_G to ±1000 dps
    unwrap!(twi.write(ADDRESS, &[0x17, 0b10]).await); // accel range: FS_XL to ±8 g
    unwrap!(twi.write(ADDRESS, &[0x09, (0b1000) | (0b1000 << 4)]).await);
    unwrap!(twi.write(ADDRESS, &[0x0a, 0b110]).await); // FIFO mode: continuous mode
    unwrap!(twi.write(ADDRESS, &[0x13, 0b10]).await); // DRDY_PULSED to 1
    unwrap!(twi.write(ADDRESS, &[0x0d, 0b1]).await); // INT1_CTRL: INT1_DRDY_XL to 1

    let mut vqf = VQF::new(
        1.0 / 480.0,
        Some(1.0 / 120.0),
        None,
        Some(Params {
            motion_bias_est_enabled: true,
            bias_sigma_init: 1.0,
            bias_clip: 2.0,
            rest_th_gyr: 1.0,
            rest_th_acc: 0.192,
            ..Default::default()
        }),
    );

    let cal_end = Instant::now() + Duration::from_secs(5);

    let mut x_acc = 0;
    let mut y_acc = 0;
    let mut z_acc = 0;
    let mut cnt_acc = -1;

    let mut x_accel_last = 0;
    let mut y_accel_last = 0;
    let mut z_accel_last = 0;

    let mut x_off = 0;
    let mut y_off = 0;
    let mut z_off = 0;

    let mut int = gpio::Input::new(p0_17, embassy_nrf::gpio::Pull::None);

    let hex_1b = [0x1b];
    let hex_78 = [0x78];

    let mut seq: u32 = 0;

    loop {
        let mut size_buf = [0u8; 2];
        unwrap!(twi.write_read(ADDRESS, &hex_1b, &mut size_buf).await);
        let len = size_buf[0] as usize;
        let read_size = (7 * (len + 1)).clamp(0, 16 * 7);
        let mut buf = [0u8; 7 * 16];
        unwrap!(
            twi.write_read(ADDRESS, &hex_78, &mut buf[..read_size])
                .await
        );
        let buf: [[u8; 7]; 16] = zerocopy::transmute!(buf);
        let mut fifo_fully_read = false;
        for entry in &buf[..(len + 1)] {
            let tag = entry[0] >> 3;
            match tag {
                0x00 => {
                    fifo_fully_read = true;    
                    break
                },
                0x01 => {
                    let x = i16::from_le_bytes([entry[1], entry[2]]) + x_off;
                    let y = i16::from_le_bytes([entry[3], entry[4]]) + y_off;
                    let z = i16::from_le_bytes([entry[5], entry[6]]) + z_off;
                    if cnt_acc != -1 {
                        x_acc += x as i32;
                        y_acc += y as i32;
                        z_acc += z as i32;
                        cnt_acc += 1;
                    }
                    vqf.update_gyr([GSCALE * x as f32, GSCALE * y as f32, GSCALE * z as f32])
                }
                0x02 => {
                    let x = i16::from_le_bytes([entry[1], entry[2]]);
                    let y = i16::from_le_bytes([entry[3], entry[4]]);
                    let z = i16::from_le_bytes([entry[5], entry[6]]);
                    x_accel_last = x;
                    y_accel_last = y;
                    z_accel_last = z;
                    vqf.update_acc([
                        ACCEL_SENSITIVITY * x as f32,
                        ACCEL_SENSITIVITY * y as f32,
                        ACCEL_SENSITIVITY * z as f32,
                    ])
                }
                x => info!("unknown tag: {=u8:02X}", x),
            }
        }

        if !fifo_fully_read {
            info!("FIFO was not fully read!");
        }

        if cnt_acc != -1 && Instant::now() > cal_end {
            x_off = (x_acc / cnt_acc) as i16;
            y_off = (y_acc / cnt_acc) as i16;
            z_off = (z_acc / cnt_acc) as i16;
            info!(
                "calibration using {} samples: {} {} {}",
                cnt_acc,
                x_off as f32 * GSCALE,
                y_off as f32 * GSCALE,
                z_off as f32 * GSCALE
            );
            let ([vqf_x_off, vqf_y_off, vqf_z_off], vqf_off_sigma) = vqf.bias_estimate();
            info!(
                "vqf bias estimate: {} {} {} {}",
                vqf_x_off / GSCALE,
                vqf_y_off / GSCALE,
                vqf_z_off / GSCALE,
                vqf_off_sigma / GSCALE
            );
            cnt_acc = -1;
        }

        let mut saadc_buf = [0i16; 1];
        saadc.sample(&mut saadc_buf).await;

        let voltage = saadc_buf[0] as f32 * (6.0 / 4096.0);

        let quat = vqf.quat_6d();
        let quat = icd::Quaternion(quat.0, quat.1, quat.2, quat.3);
        _ = sender
            .publish::<icd::tracker::VqfData>(seq.into(), &quat)
            .await;
        seq += 1;
        if let Some(pair_data_real) = &pair_data {
            if let Ok(mut packet) = postcard::serialize_with_flavor(
                &icd::DataFrame {
                    device_id: pair_data_real.assigned_id,
                    battery_level: ((voltage / 5.0) * 256.0) as u8,
                    accel: [x_accel_last, y_accel_last, z_accel_last],
                    quat: quat.into(),
                },
                PacketPostcardWriter(Packet::new()),
            ) {
                if let Ok(packet) = radio
                    .try_send_with_ack(
                        pac::TIMER0,
                        &mut packet,
                        1,
                        (&mut ppi_ch0, &mut ppi_ch1, &mut ppi_ch2),
                    )
                    .await
                {
                    // info!("recv'd ack: {}", &packet as &[u8]);
                    if packet.is_empty() {
                        last_successful_tx = Instant::now();
                    }
                } else if (last_successful_tx + UNPAIR_TIMEOUT + desync_factor) <= Instant::now() {
                    info!("Unpairing!");
                    pair_data = None;
                }
            }
        } else if (last_successful_tx + SLEEP_TIMEOUT) <= Instant::now()
            && (!pac::POWER.usbregstatus().read().vbusdetect())
        {
            unsafe {
                let Err(e) = enter_wake_on_movement(&mut twi).await;
                info!("WOM enter error: {}", e);
            }
        } else if (last_pair_attempt + PAIR_RETRY_INTERVAL + desync_factor) <= Instant::now() {
            pair_data = attempt_pair(&mut radio, (&mut ppi_ch0, &mut ppi_ch1, &mut ppi_ch2)).await;
            last_pair_attempt = Instant::now();
        }

        // info!(
        //     "samp: {} quat: {=f32} {=f32} {=f32} {=f32}, rest: {=bool}, volt: {=f32}, last tx: {}, last pair: {}, pair data: {}",
        //     size_buf[0],
        //     quat.0,
        //     quat.1,
        //     quat.2,
        //     quat.3,
        //     vqf.rest_detected(),
        //     voltage,
        //     last_successful_tx,
        //     last_pair_attempt,
        //     pair_data.is_some()
        // );
        int.wait_for_rising_edge().await;
    }
}

async unsafe fn enter_wake_on_movement(twim: &mut twim::Twim<'_, peripherals::TWISPI0>) -> Result<Infallible, twim::Error> {
    info!("getting ready to eep");
    twim.write(ADDRESS, &[0x0d, 0]).await?; // INT1_CTRL: all cleared
    twim.write(ADDRESS, &[0x10, 0b1000110]).await?; // accel ODR: OP_MODE_XL to low power 1, ODR_XL to 120Hz
    twim.write(ADDRESS, &[0x11, 0b0000000]).await?; // gyro ODR: power down
    twim.write(ADDRESS, &[0x17, 0b10]).await?; // accel range: FS_XL to ±8 g
    twim.write(ADDRESS, &[0x18, 0b10]).await?; // USR_OFF_W to 2^-6 g/LSB
    twim.write(ADDRESS, &[0x56, 0b10000]).await?; // SLOPE_FDS to 1
    twim.write(ADDRESS, &[0x5b, 0b010000]).await?; // WK_THS to 4, USR_OFF_ON_WU to 0
    twim.write(ADDRESS, &[0x5c, 0b1100000]).await?; // WAKE_DUR to 3

    embassy_time::Timer::after_millis(11).await; // wait for accel to settle

    pac::P0.pin_cnf(17).write(|w| {
        w.set_dir(pac::gpio::vals::Dir::INPUT);
        w.set_input(pac::gpio::vals::Input::CONNECT);
        w.set_pull(pac::gpio::vals::Pull::DISABLED);
        w.set_sense(pac::gpio::vals::Sense::HIGH);
    });

    info!("gn!");
    twim.write(ADDRESS, &[0x50, 0b10000000]).await?; // FUNCTIONS_ENABLE: INTERRUPTS_ENABLE to 1
    twim.write(ADDRESS, &[0x5e, 0b100000]).await?; // MD1_CFG: INT1_WU to 1
    enter_deep_sleep()
}

fn enter_deep_sleep() -> ! {
    embassy_nrf::power::set_system_off();
    #[allow(clippy::empty_loop)]
    loop {} // only reachable when in debug
}

async fn attempt_pair(
    radio: &mut Radio<'static, embassy_nrf::peripherals::RADIO>,
    ppi_channels: (
        &mut impl ConfigurableChannel,
        &mut impl ConfigurableChannel,
        &mut impl ConfigurableChannel,
    ),
) -> Option<icd::PairResponse> {
    info!("attempting pair");
    let mut packet = postcard::serialize_with_flavor(
        &icd::PairRequest {
            device_id: get_unique_id().to_le_bytes(),
            protocol_key: icd::CURRENT_PROTOCOL_KEY,
        },
        PacketPostcardWriter(Packet::new()),
    )
    .ok()?;
    info!("sending radio");
    let result = radio
        .try_send_with_ack(pac::TIMER0, &mut packet, 0, ppi_channels)
        .await
        .ok()?;
    info!("ack gotten: {}", &result as &[u8]);
    let result: icd::PairResponse = postcard::from_bytes(&result).ok()?;
    info!("ack parse ok");
    if result.protocol_key != icd::CURRENT_PROTOCOL_KEY {
        info!("protocol mismatch");
        return None;
    }
    radio.set_base_address_1(result.base_addr);
    Some(result)
}

/// This handles the low level USB management
#[embassy_executor::task]
pub async fn usb_task(mut usb: UsbDevice<'static, app::AppDriver>) {
    loop {
        usb.run_until_suspend().await;
        usb.wait_resume().await;
        USB_SIG.signal(());
    }
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
