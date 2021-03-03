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

const DEFAULT_PORT: u16 = 50692;
const DEFAULT_MULTICAST: [u8; 4] = [239, 255, 42, 98];
const IP_ALL: [u8; 4] = [0, 0, 0, 0];

pub fn create_socket() -> Result<(std::net::UdpSocket, SocketAddrV4)> {
    let addr = SocketAddrV4::new(IP_ALL.into(), DEFAULT_PORT);
    let multi_addr = SocketAddrV4::new(DEFAULT_MULTICAST.into(), DEFAULT_PORT);
    let socket = bind_multicast(&addr, &multi_addr)?;
    Ok((socket, multi_addr))
}
