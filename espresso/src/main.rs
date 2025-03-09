use std::{collections::HashMap, time::Duration};

use color_eyre::eyre::eyre;
use postcard_rpc::{
    header::VarSeqKind,
    host_client::{HostClient, Subscription},
    standard_icd::{WireError, ERROR_PATH},
};
use tokio::{net::UdpSocket, time::timeout};

const ACCEL_SENSITIVITY: f32 = 0.244 / 1000.0;

async fn try_discover_slimevr_server(id: u64) -> color_eyre::Result<UdpSocket> {
    println!("Connecting tracker {id:016X}");
    let sock = UdpSocket::bind("0.0.0.0:0").await?;
    sock.set_broadcast(true)?;
    loop {
        let id_bytes = id.to_be_bytes();
        let buf = [
            &[
                0u8, 0, 0, 3, // pkt type
                0, 0, 0, 0, 0, 0, 0, 0, // pkt number
                0, 0, 0, 250, // board type ("Prototype")
                0, 0, 0, 13, // IMU type ("LSM6DSV")
                0, 0, 0, 250, // mcu type ("DEV_RESERVED")
                0, 0, 0, 0, // IMU info (unused)
                0, 0, 0, 0, // IMU info (unused)
                0, 0, 0, 0, // IMU info (unused)
                0, 0, 0, 19, // protocol version (19)
                14,
            ] as &[u8],
            b"tiramisu-fw-ng",
            &id_bytes[2..],
        ]
        .into_iter()
        .flatten()
        .copied()
        .collect::<Vec<_>>();
        sock.send_to(&buf, "255.255.255.255:6969").await?;
        let mut reply = [0u8; 13];
        let (read, addr) = timeout(Duration::from_secs(1), sock.recv_from(&mut reply)).await??;
        if read == 13 && &reply == b"\x03Hey OVR =D 5" {
            sock.connect(addr).await?;
            println!("connected {:016X} to {}", id, addr);
            return Ok(sock);
        }
    }
}

#[tokio::main]
async fn main() -> color_eyre::Result<()> {
    let client: HostClient<WireError> = HostClient::try_new_raw_nusb(
        |d| d.product_string() == Some("Mascarpone"),
        ERROR_PATH,
        8,
        VarSeqKind::Seq1,
    )
    .map_err(|e| eyre!("{e}"))?;
    let mut sub = client
        .subscribe_exclusive::<icd::dongle::DongleData>(4)
        .await
        .map_err(|e| eyre!("{e:?}"))?;

    let mut pairmap: HashMap<u64, UdpSocket> = HashMap::new();
    while let Some(data) = sub.recv().await {
        let did = u64::from_le_bytes(data.device_id);
        let tid = if let Some(tid) = pairmap.get(&did) {
            tid
        } else {
            let tid = try_discover_slimevr_server(did).await?;
            pairmap.insert(did, tid);
            let tid = pairmap.get(&did).unwrap();
            tid.send(&[0, 0, 0, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 0, 0])
                .await?;
            tid
        };
        let quat: icd::Quaternion = data.quat.into();
        let quatbuf = [
            &[
                0u8, 0, 0, 17, // pkt type
                // 0, 0, 0, 0, 0, 0, 0, 0, // pkt number
                0, // sensor id
                1, // data type (DATA_TYPE_NORMAL)
            ] as &[u8],
            &quat.1.to_be_bytes(),
            &quat.2.to_be_bytes(),
            &quat.3.to_be_bytes(),
            &quat.0.to_be_bytes(),
            &[0], // cal info (unused)
        ]
        .into_iter()
        .flatten()
        .copied()
        .collect::<Vec<_>>();
        // tid.send(&quatbuf).await?;
        let accbuf = [
            &[
                0u8, 0, 0, 4, // pkt type
                // 0, 0, 0, 0, 0, 0, 0, 0, // pkt id
            ] as &[u8],
            &(data.accel[0] as f32 * ACCEL_SENSITIVITY).to_be_bytes(),
            &(data.accel[1] as f32 * ACCEL_SENSITIVITY).to_be_bytes(),
            &(data.accel[2] as f32 * ACCEL_SENSITIVITY).to_be_bytes(),
            &[0], // sensor id
        ]
        .into_iter()
        .flatten()
        .copied()
        .collect::<Vec<_>>();
        // tid.send(&accbuf).await?;
        let voltage = (data.battery_level as f32 / 256.0) * 5.0;
        let level;
        if voltage > 4.5 {
            level = f32::MAX;
        } else if voltage > 3.975 {
            level = (voltage - 2.920) * 0.8;
        } else if voltage > 3.678 {
            level = (voltage - 3.300) * 1.25;
        } else if voltage > 3.489 {
            level = (voltage - 3.400) * 1.7;
        } else if voltage > 3.360 {
            level = (voltage - 3.300) * 0.8;
        } else {
            level = (voltage - 3.200) * 0.3;
        }
        let batbuf = [
            &[
                0u8, 0, 0, 12, // pkt type
                // 0, 0, 0, 0, 0, 0, 0, 0, // pkt id
            ] as &[u8],
            &(voltage).to_be_bytes(),
            &(level).to_be_bytes(),
        ]
        .into_iter()
        .flatten()
        .copied()
        .collect::<Vec<_>>();
        // tid.send(&batbuf).await?;
        let rssibuf = [
            0u8, 0, 0, 19, // pkt type
            // 0, 0, 0, 0, 0, 0, 0, 0, // pkt id
            0, // sensor id
            (-(data.rssi as i8) as u8),
        ];
        let temp = (data.temp as f32 / 2.0) + 25.0;
        let tempbuf = [
            &[
                0u8, 0, 0, 20, // pkt type
                // 0, 0, 0, 0, 0, 0, 0, 0, // pkt id
                0, // sensor id
            ] as &[u8],
            &(temp).to_be_bytes(),
        ]
        .into_iter()
        .flatten()
        .copied()
        .collect::<Vec<_>>();
        // tid.send(&rssibuf).await?;
        let bundlebuf = [
            &[
                0u8, 0, 0, 100, // pkt type
                0, 0, 0, 0, 0, 0, 0, 0, // pkt id
            ] as &[u8],
            &(quatbuf.len() as u16).to_be_bytes(),
            &quatbuf,
            &(accbuf.len() as u16).to_be_bytes(),
            &accbuf,
            &(batbuf.len() as u16).to_be_bytes(),
            &batbuf,
            &(rssibuf.len() as u16).to_be_bytes(),
            &rssibuf,
            &(tempbuf.len() as u16).to_be_bytes(),
            &tempbuf,
        ]
        .into_iter()
        .flatten()
        .copied()
        .collect::<Vec<_>>();
        tid.send(&bundlebuf).await?;
    }
    Ok(())
}