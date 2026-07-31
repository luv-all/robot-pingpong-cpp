use super::bus::{BulkWrite, Bus};
use super::control_table::{ControlTable, UNIT_SCALE};
use super::protocol::ProtocolError;
use once_cell::sync::Lazy;
use parking_lot::Mutex;
use std::collections::HashMap;

static BUSES: Lazy<Mutex<HashMap<String, Bus>>> = Lazy::new(|| Mutex::new(HashMap::new()));

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum Torque {
    Disable = 0,
    Enable = 1,
}

pub struct DynamixelMotor<M: ControlTable> {
    bus: Bus,
    id: u8,
    _marker: std::marker::PhantomData<M>,
}

impl<M: ControlTable> DynamixelMotor<M> {
    pub fn new(port_name: &str, id: u8) -> Result<Self, ProtocolError> {
        let bus = Bus::open_shared(&BUSES, port_name, 4_500_000)?;
        let motor = Self {
            bus,
            id,
            _marker: std::marker::PhantomData,
        };
        match motor.ping() {
            Ok(()) => Ok(motor),
            Err(err) => {
                eprintln!("ping failed for id {id}: {err}");
                let _ = motor.reboot();
                std::thread::sleep(std::time::Duration::from_millis(200));
                motor.ping()?;
                Ok(motor)
            }
        }
    }

    pub fn id(&self) -> u8 {
        self.id
    }

    pub fn ping(&self) -> Result<(), ProtocolError> {
        let model = self.bus.ping(self.id)?;
        if model != M::MODEL_NUMBER {
            return Err(ProtocolError::ModelMismatch {
                got: model,
                expected: M::MODEL_NUMBER,
            });
        }
        Ok(())
    }

    pub fn reboot(&self) -> Result<(), ProtocolError> {
        self.bus.reboot(self.id)
    }

    pub fn read_hardware_error_status(&self) -> Result<u8, ProtocolError> {
        let data = self.bus.read(self.id, M::HARDWARE_ERROR_STATUS, 1)?;
        Ok(data.first().copied().unwrap_or(0))
    }

    pub fn set_torque_enable(&self, torque: Torque) -> Result<(), ProtocolError> {
        self.bus
            .write(self.id, M::TORQUE_ENABLE, &[torque as u8])
    }

    pub fn set_profile_velocity(&self, velocity: u32) -> Result<(), ProtocolError> {
        self.bus
            .write(self.id, M::PROFILE_VELOCITY, &velocity.to_le_bytes())
    }

    pub fn set_profile_acceleration(&self, acceleration: u32) -> Result<(), ProtocolError> {
        self.bus
            .write(self.id, M::PROFILE_ACCELERATION, &acceleration.to_le_bytes())
    }

    pub fn get_angle(&self) -> Result<f64, ProtocolError> {
        let data = self.bus.read(self.id, M::PRESENT_POSITION, 4)?;
        if data.len() < 4 {
            return Err(ProtocolError::InvalidPacket);
        }
        let ticks = i32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        Ok(ticks as f64 * UNIT_SCALE)
    }

    pub fn set_angle(&self, angle: f64) -> Result<(), ProtocolError> {
        let ticks = (angle / UNIT_SCALE) as u32;
        self.bus
            .write(self.id, M::GOAL_POSITION, &ticks.to_le_bytes())
    }

    pub fn set_angle_bulk(&self, writer: &mut BulkWrite, angle: f64) {
        let ticks = (angle / UNIT_SCALE) as u32;
        writer.add_param(self.id, M::GOAL_POSITION, ticks.to_le_bytes().to_vec());
    }

    pub fn bulk_writer(&self) -> BulkWrite {
        BulkWrite::new(self.bus.clone())
    }
}
