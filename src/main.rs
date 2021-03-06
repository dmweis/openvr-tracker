mod multicast;
mod openvr_adaptor;
mod tracking_messages;

use anyhow::Result;
use clap::Clap;
use std::net::SocketAddrV4;
use std::thread::sleep;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

#[derive(Clap)]
#[clap(version = "0.0.1", author = "David M. W. <dweis7@gmail.com>")]
struct Args {
    #[clap(short, long, default_value = "239.0.0.22:7070")]
    address: SocketAddrV4,
}

fn main() -> Result<()> {
    let args: Args = Args::parse();
    let mut openvr = openvr_adaptor::VrDeviceManager::new()?;
    let messenger = multicast::MessageSender::new(args.address)?;
    loop {
        openvr.update();
        let devices = openvr
            .device_list()
            .into_iter()
            .filter(|object| object.seen())
            .collect();
        let time = SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis();
        let objects = tracking_messages::TrackedObjects::new(time, devices);
        let json = serde_json::to_string(&objects)?;
        println!("{}", &json);
        messenger.send(&json)?;
        sleep(Duration::from_millis(20));
    }
}
