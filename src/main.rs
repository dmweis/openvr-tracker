mod multicast;
mod openvr_adaptor;

use anyhow::Result;
use serde::Serialize;
use std::thread::sleep;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

#[derive(Debug, Serialize)]
struct TrackedObjects {
    ts: u128,
    trackers: Vec<openvr_adaptor::VrDevice>,
}

impl TrackedObjects {
    pub fn new(ts: u128, trackers: Vec<openvr_adaptor::VrDevice>) -> Self {
        Self { ts, trackers }
    }
}

fn main() -> Result<()> {
    let mut openvr = openvr_adaptor::VrDeviceManager::new()?;
    let (socket, address) = multicast::create_socket()?;
    loop {
        openvr.update();
        let devices = openvr.device_list();
        let time = SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis();
        let objects = TrackedObjects::new(time, devices);
        let json = serde_json::to_string(&objects)?;
        let message = json.as_bytes();
        socket.send_to(message, address)?;
        sleep(Duration::from_millis(20));
    }
}
