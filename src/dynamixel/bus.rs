use super::protocol::{build_packet, parse_status, Instruction, ProtocolError, BROADCAST_ID};
use parking_lot::Mutex;
use serialport::SerialPort;
use std::collections::HashMap;
use std::io::{Read, Write};
use std::sync::Arc;
use std::time::Duration;

#[derive(Clone)]
pub struct Bus {
    inner: Arc<Mutex<BusInner>>,
}

struct BusInner {
    port: Box<dyn SerialPort>,
}

impl Bus {
    pub fn open(port_name: &str, baud_rate: u32) -> Result<Self, ProtocolError> {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(50))
            .open()?;
        Ok(Self {
            inner: Arc::new(Mutex::new(BusInner { port })),
        })
    }

    pub fn open_shared(
        cache: &Mutex<HashMap<String, Bus>>,
        port_name: &str,
        baud_rate: u32,
    ) -> Result<Self, ProtocolError> {
        let mut guard = cache.lock();
        if let Some(bus) = guard.get(port_name) {
            return Ok(bus.clone());
        }
        let bus = Self::open(port_name, baud_rate)?;
        guard.insert(port_name.to_string(), bus.clone());
        Ok(bus)
    }

    pub fn ping(&self, id: u8) -> Result<u16, ProtocolError> {
        let params = self.tx_rx(id, Instruction::Ping, &[])?;
        if params.len() < 2 {
            return Err(ProtocolError::InvalidPacket);
        }
        Ok(u16::from_le_bytes([params[0], params[1]]))
    }

    pub fn reboot(&self, id: u8) -> Result<(), ProtocolError> {
        let _ = self.tx_rx(id, Instruction::Reboot, &[])?;
        Ok(())
    }

    pub fn read(&self, id: u8, address: u16, length: u16) -> Result<Vec<u8>, ProtocolError> {
        let mut params = Vec::with_capacity(4);
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(&length.to_le_bytes());
        self.tx_rx(id, Instruction::Read, &params)
    }

    pub fn write(&self, id: u8, address: u16, data: &[u8]) -> Result<(), ProtocolError> {
        let mut params = Vec::with_capacity(2 + data.len());
        params.extend_from_slice(&address.to_le_bytes());
        params.extend_from_slice(data);
        let _ = self.tx_rx(id, Instruction::Write, &params)?;
        Ok(())
    }

    pub fn bulk_write(&self, entries: &[(u8, u16, Vec<u8>)]) -> Result<(), ProtocolError> {
        let mut params = Vec::new();
        for (id, address, data) in entries {
            params.push(*id);
            params.extend_from_slice(&address.to_le_bytes());
            params.extend_from_slice(&(data.len() as u16).to_le_bytes());
            params.extend_from_slice(data);
        }
        let packet = build_packet(BROADCAST_ID, Instruction::BulkWrite, &params);
        let mut inner = self.inner.lock();
        inner.port.clear(serialport::ClearBuffer::All)?;
        inner.port.write_all(&packet)?;
        inner.port.flush()?;
        Ok(())
    }

    fn tx_rx(
        &self,
        id: u8,
        instruction: Instruction,
        params: &[u8],
    ) -> Result<Vec<u8>, ProtocolError> {
        let packet = build_packet(id, instruction, params);
        let mut inner = self.inner.lock();
        inner.port.clear(serialport::ClearBuffer::All)?;
        inner.port.write_all(&packet)?;
        inner.port.flush()?;

        let mut buffer = Vec::new();
        let mut chunk = [0u8; 256];
        loop {
            match inner.port.read(&mut chunk) {
                Ok(0) => return Err(ProtocolError::Timeout),
                Ok(n) => {
                    buffer.extend_from_slice(&chunk[..n]);
                    if buffer.len() >= 11 {
                        if let Ok((_rid, error, params)) = parse_status(&buffer) {
                            if error != 0 {
                                return Err(ProtocolError::DeviceError(error));
                            }
                            return Ok(params.to_vec());
                        }
                    }
                    if buffer.len() > 1024 {
                        return Err(ProtocolError::InvalidPacket);
                    }
                }
                Err(err) if err.kind() == std::io::ErrorKind::TimedOut => {
                    return Err(ProtocolError::Timeout);
                }
                Err(err) => return Err(ProtocolError::Io(err)),
            }
        }
    }
}

pub struct BulkWrite {
    bus: Bus,
    entries: Vec<(u8, u16, Vec<u8>)>,
}

impl BulkWrite {
    pub fn new(bus: Bus) -> Self {
        Self {
            bus,
            entries: Vec::new(),
        }
    }

    pub fn add_param(&mut self, id: u8, address: u16, data: Vec<u8>) {
        self.entries.push((id, address, data));
    }

    pub fn tx_packet(self) -> Result<(), ProtocolError> {
        self.bus.bulk_write(&self.entries)
    }
}
