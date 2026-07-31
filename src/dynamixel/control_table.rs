pub const UNIT_SCALE: f64 = 0.087891;

pub trait ControlTable {
    const MODEL_NUMBER: u16;
    const TORQUE_ENABLE: u16 = 64;
    const HARDWARE_ERROR_STATUS: u16 = 70;
    const PROFILE_ACCELERATION: u16 = 108;
    const PROFILE_VELOCITY: u16 = 112;
    const GOAL_POSITION: u16 = 116;
    const PRESENT_POSITION: u16 = 132;
}

#[derive(Debug, Clone, Copy)]
pub struct Mx64;

impl ControlTable for Mx64 {
    const MODEL_NUMBER: u16 = 311;
}

#[derive(Debug, Clone, Copy)]
pub struct Mx28;

impl ControlTable for Mx28 {
    const MODEL_NUMBER: u16 = 30;
}
