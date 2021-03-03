use anyhow::Result;
use socket2::{Domain, Protocol, Socket, Type};
use std::net::SocketAddrV4;
use std::net::UdpSocket;

fn bind_multicast(addr: &SocketAddrV4, multi_addr: &SocketAddrV4) -> Result<UdpSocket> {
    assert!(multi_addr.ip().is_multicast(), "Address must be multicast");

    let socket = Socket::new(Domain::ipv4(), Type::dgram(), Some(Protocol::udp()))?;

    socket.set_reuse_address(true)?;
    socket.set_nonblocking(true)?;
    socket.bind(&socket2::SockAddr::from(*addr))?;
    socket.set_multicast_loop_v4(true)?;
    socket.join_multicast_v4(multi_addr.ip(), addr.ip())?;
    Ok(socket.into_udp_socket())
}

const ALL_INTERFACES: [u8; 4] = [0, 0, 0, 0];

pub struct MessageSender {
    socket: UdpSocket,
    multicast_address: SocketAddrV4,
}

impl MessageSender {
    pub fn new(multicast_address: SocketAddrV4) -> Result<Self> {
        let addr = SocketAddrV4::new(ALL_INTERFACES.into(), multicast_address.port());
        let socket = bind_multicast(&addr, &multicast_address)?;
        Ok(Self {
            socket,
            multicast_address,
        })
    }

    pub fn send(&self, message: &str) -> Result<()> {
        self.socket
            .send_to(message.as_bytes(), self.multicast_address)?;
        Ok(())
    }
}
