#![no_std]
use postcard_schema::{key::Key, Schema};
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Schema, Clone, Copy)]
pub struct Quaternion(pub f32, pub f32, pub f32, pub f32);

#[derive(Debug, Serialize, Deserialize, Schema, Clone, Copy)]
pub struct QuaternionQ15(pub [u8; 2], pub [u8; 2], pub [u8; 2], pub [u8; 2]);

fn from_q15(value: [u8; 2]) -> f32 {
    ((i16::from_le_bytes(value)) as f32) / 32768.0
}

fn to_q15(value: f32) -> [u8; 2] {
    ((value * 32768.0).clamp(i16::MIN as _, i16::MAX as _) as i16).to_le_bytes()
}

impl From<Quaternion> for QuaternionQ15 {
    fn from(value: Quaternion) -> Self {
        QuaternionQ15(
            to_q15(value.0),
            to_q15(value.1),
            to_q15(value.2),
            to_q15(value.3),
        )
    }
}

impl From<QuaternionQ15> for Quaternion {
    fn from(value: QuaternionQ15) -> Self {
        Quaternion(
            from_q15(value.0),
            from_q15(value.1),
            from_q15(value.2),
            from_q15(value.3),
        )
    }
}

#[derive(Debug, Serialize, Deserialize, Schema)]
pub struct DataFrame {
    pub device_id: u8,
    pub battery_level: u8,
    pub accel: [i8; 3],
    pub temp: i8,
    pub quat: QuaternionQ15,
}

#[derive(Debug, Serialize, Deserialize, Schema)]
pub struct PairRequest {
    pub device_id: [u8; 8],
    pub protocol_key: Key,
}

#[derive(Debug, Serialize, Deserialize, Schema)]
pub struct PairResponse {
    pub assigned_id: u8,
    pub base_addr: [u8; 4],
    pub protocol_key: Key,
}

#[derive(Debug, Serialize, Deserialize, Schema)]
pub struct TrackerDataFrame {
    pub device_id: [u8; 8],
    pub battery_level: u8,
    pub rssi: u8,
    pub temp: i8,
    pub accel: [i16; 3],
    pub quat: QuaternionQ15,
}

pub const CURRENT_PROTOCOL_KEY: Key = Key::for_path::<DataFrame>("tiramisu/air");

pub mod tracker {
    use crate::Quaternion;
    use postcard_rpc::{endpoints, topics, TopicDirection};

    type Str = &'static str;
    // Endpoints spoken by our device
    //
    // GetUniqueIdEndpoint is mandatory, the others are examples
    endpoints! {
        list = ENDPOINT_LIST;
        | EndpointTy                | RequestTy     | ResponseTy            | Path                          |
        | ----------                | ---------     | ----------            | ----                          |
        | GetUniqueIdEndpoint       | ()            | u64                   | "poststation/unique_id/get"   |
        | MeowEndpoint              | ()            | Str                   | "meow"                        |
        | DfuEndpoint               | ()            | ()                    | "enter_dfu"                   |
    }

    // incoming topics handled by our device
    topics! {
        list = TOPICS_IN_LIST;
        direction = TopicDirection::ToServer;
        | TopicTy                   | MessageTy     | Path              |
        | -------                   | ---------     | ----              |
    }

    // outgoing topics handled by our device
    topics! {
        list = TOPICS_OUT_LIST;
        direction = TopicDirection::ToClient;
        | TopicTy                   | MessageTy     | Path              | Cfg                           |
        | -------                   | ---------     | ----              | ---                           |
        | VqfData                   | Quaternion    | "tiramisu/quat"   |                               |
    }
}

pub mod dongle {
    use crate::TrackerDataFrame;
    use postcard_rpc::{endpoints, topics, TopicDirection};

    type Str = &'static str;
    // Endpoints spoken by our device
    //
    // GetUniqueIdEndpoint is mandatory, the others are examples
    endpoints! {
        list = ENDPOINT_LIST;
        | EndpointTy                | RequestTy     | ResponseTy            | Path                          |
        | ----------                | ---------     | ----------            | ----                          |
        | GetUniqueIdEndpoint       | ()            | u64                   | "poststation/unique_id/get"   |
        | MeowEndpoint              | ()            | Str                   | "meow"                        |
    }

    // incoming topics handled by our device
    topics! {
        list = TOPICS_IN_LIST;
        direction = TopicDirection::ToServer;
        | TopicTy                   | MessageTy     | Path              |
        | -------                   | ---------     | ----              |
    }

    // outgoing topics handled by our device
    topics! {
        list = TOPICS_OUT_LIST;
        direction = TopicDirection::ToClient;
        | TopicTy                   | MessageTy        | Path              | Cfg                           |
        | -------                   | ---------        | ----              | ---                           |
        | DongleData                | TrackerDataFrame | "tiramisu/data"   |                               |
    }
}
