use crate::constants::{
    ELBOW_ID, PORT_NAME, SHOULDER_ID, SHOULDER_PRIME_ID, SHOULDER_YAW_ID, WRIST_ID,
};
use crate::dynamixel::{DynamixelMotor, Mx28, Mx64, Torque};
use anyhow::{Context, Result};
use parking_lot::Mutex;
use std::f64::consts::PI;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

pub struct Arm {
    shoulder_pitch: Arc<DynamixelMotor<Mx64>>,
    shoulder_pitch_rev: Arc<DynamixelMotor<Mx64>>,
    shoulder_yaw: Arc<DynamixelMotor<Mx64>>,
    elbow: Arc<DynamixelMotor<Mx28>>,
    wrist: Arc<DynamixelMotor<Mx28>>,
    resetted: Arc<AtomicBool>,
    busy: Arc<Mutex<()>>,
}

impl Arm {
    pub fn new() -> Result<Self> {
        Ok(Self {
            shoulder_pitch: Arc::new(
                DynamixelMotor::<Mx64>::new(PORT_NAME, SHOULDER_ID)
                    .context("open shoulder pitch")?,
            ),
            shoulder_pitch_rev: Arc::new(
                DynamixelMotor::<Mx64>::new(PORT_NAME, SHOULDER_PRIME_ID)
                    .context("open shoulder pitch rev")?,
            ),
            shoulder_yaw: Arc::new(
                DynamixelMotor::<Mx64>::new(PORT_NAME, SHOULDER_YAW_ID)
                    .context("open shoulder yaw")?,
            ),
            elbow: Arc::new(
                DynamixelMotor::<Mx28>::new(PORT_NAME, ELBOW_ID).context("open elbow")?,
            ),
            wrist: Arc::new(
                DynamixelMotor::<Mx28>::new(PORT_NAME, WRIST_ID).context("open wrist")?,
            ),
            resetted: Arc::new(AtomicBool::new(false)),
            busy: Arc::new(Mutex::new(())),
        })
    }

    pub fn init(&self) -> Result<()> {
        let motors: [&dyn MotorOps; 5] = [
            &*self.shoulder_pitch,
            &*self.shoulder_pitch_rev,
            &*self.shoulder_yaw,
            &*self.elbow,
            &*self.wrist,
        ];
        for motor in motors {
            if motor.read_hardware_error_status()? != 0 {
                motor.reboot()?;
            }
            motor.set_profile_velocity(0)?;
            motor.set_profile_acceleration(0)?;
            motor.set_torque_enable(Torque::Enable)?;
        }

        let this = ArmHandles {
            shoulder_pitch: Arc::clone(&self.shoulder_pitch),
            shoulder_pitch_rev: Arc::clone(&self.shoulder_pitch_rev),
            shoulder_yaw: Arc::clone(&self.shoulder_yaw),
            elbow: Arc::clone(&self.elbow),
            wrist: Arc::clone(&self.wrist),
            resetted: Arc::clone(&self.resetted),
            busy: Arc::clone(&self.busy),
        };
        thread::spawn(move || {
            let _ = this.reset_by_z_blocking(50.0);
            this.resetted.store(false, Ordering::SeqCst);
            thread::sleep(Duration::from_secs(1));
            let _ = this.reset_by_z_blocking(400.0);
            this.resetted.store(false, Ordering::SeqCst);
        });
        Ok(())
    }

    pub fn move_to(&self, _y: f64, z: f64, hit_target: bool) {
        let handles = self.handles();
        thread::spawn(move || {
            let Some(_guard) = handles.busy.try_lock() else {
                return;
            };
            if let Err(err) = handles.move_blocking(z, hit_target) {
                eprintln!("{err:#}");
            }
            handles.resetted.store(false, Ordering::SeqCst);
        });
    }

    pub fn reset_by_z(&self, z: f64) {
        if self.resetted.load(Ordering::SeqCst) {
            return;
        }
        let handles = self.handles();
        thread::spawn(move || {
            let Some(_guard) = handles.busy.try_lock() else {
                return;
            };
            if let Err(err) = handles.reset_by_z_blocking(z) {
                eprintln!("{err:#}");
            }
        });
    }

    fn handles(&self) -> ArmHandles {
        ArmHandles {
            shoulder_pitch: Arc::clone(&self.shoulder_pitch),
            shoulder_pitch_rev: Arc::clone(&self.shoulder_pitch_rev),
            shoulder_yaw: Arc::clone(&self.shoulder_yaw),
            elbow: Arc::clone(&self.elbow),
            wrist: Arc::clone(&self.wrist),
            resetted: Arc::clone(&self.resetted),
            busy: Arc::clone(&self.busy),
        }
    }

    /// Inverse kinematics for the 3R planar arm (millimetres / radians).
    pub fn inverse_kinematics(
        x: f64,
        _y: f64,
        z: f64,
        pitch: f64,
        _yaw: f64,
    ) -> Option<(f64, f64, f64, f64, f64)> {
        let l1 = 223.602;
        let l2 = 151.80;
        let l3 = 103.333;

        let x2 = x - l3 * pitch.cos();
        let z2 = z - l3 * pitch.sin();
        let r_square = x2 * x2 + z2 * z2;
        let cos_theta2 = (r_square - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
        if !(-1.0..=1.0).contains(&cos_theta2) {
            return None;
        }
        let sin_theta2 = (1.0 - cos_theta2 * cos_theta2).sqrt();
        let theta2 = sin_theta2.atan2(cos_theta2);
        let theta1 = z2.atan2(x2) - (l2 * sin_theta2).atan2(l1 + l2 * cos_theta2);
        let theta3 = pitch - theta1 - theta2;
        if theta1.is_nan() || theta2.is_nan() || theta3.is_nan() {
            return None;
        }

        let q1 = 180.0 + theta1 / PI * 180.0;
        let q1_rev = 180.0 - theta1 / PI * 180.0;
        let q2 = 0.0;
        let q3 = 180.0 + theta2 / PI * 180.0;
        let q4 = 180.0 + theta3 / PI * 180.0;
        Some((q1, q1_rev, q2, q3, q4))
    }
}

struct ArmHandles {
    shoulder_pitch: Arc<DynamixelMotor<Mx64>>,
    shoulder_pitch_rev: Arc<DynamixelMotor<Mx64>>,
    shoulder_yaw: Arc<DynamixelMotor<Mx64>>,
    elbow: Arc<DynamixelMotor<Mx28>>,
    wrist: Arc<DynamixelMotor<Mx28>>,
    resetted: Arc<AtomicBool>,
    busy: Arc<Mutex<()>>,
}

impl ArmHandles {
    fn move_blocking(&self, z: f64, hit_target: bool) -> Result<()> {
        let mut max_x = 320;
        while max_x > 120 {
            let x = if hit_target { max_x as f64 } else { 120.0 };
            let zz = z + if hit_target { 40.0 } else { 0.0 };
            let pitch = if hit_target { 60.0 } else { 100.0 } * PI / 180.0;
            if let Some((q1, q1_rev, q2, q3, q4)) =
                Arm::inverse_kinematics(x, 0.0, zz, pitch, 0.0)
            {
                self.send_angles(q1, q1_rev, q2, q3, q4)?;
                return Ok(());
            }
            max_x -= 1;
        }
        Ok(())
    }

    fn reset_by_z_blocking(&self, z: f64) -> Result<()> {
        let pitch = PI / 2.0;
        let Some((q1, q1_rev, q2, q3, q4)) = Arm::inverse_kinematics(120.0, 0.0, z, pitch, 0.0)
        else {
            return Ok(());
        };
        self.send_angles(q1, q1_rev, q2, q3, q4)?;
        self.resetted.store(true, Ordering::SeqCst);
        Ok(())
    }

    fn send_angles(&self, q1: f64, q1_rev: f64, q2: f64, q3: f64, q4: f64) -> Result<()> {
        let mut writer = self.shoulder_pitch.bulk_writer();
        self.shoulder_pitch.set_angle_bulk(&mut writer, q1);
        self.shoulder_pitch_rev
            .set_angle_bulk(&mut writer, q1_rev);
        self.shoulder_yaw.set_angle_bulk(&mut writer, q2);
        self.elbow.set_angle_bulk(&mut writer, q3);
        self.wrist.set_angle_bulk(&mut writer, q4);
        writer.tx_packet().context("bulk write angles")?;
        Ok(())
    }
}

trait MotorOps {
    fn read_hardware_error_status(&self) -> Result<u8>;
    fn reboot(&self) -> Result<()>;
    fn set_profile_velocity(&self, velocity: u32) -> Result<()>;
    fn set_profile_acceleration(&self, acceleration: u32) -> Result<()>;
    fn set_torque_enable(&self, torque: Torque) -> Result<()>;
}

impl<M: crate::dynamixel::control_table::ControlTable> MotorOps for DynamixelMotor<M> {
    fn read_hardware_error_status(&self) -> Result<u8> {
        Ok(DynamixelMotor::read_hardware_error_status(self)?)
    }
    fn reboot(&self) -> Result<()> {
        Ok(DynamixelMotor::reboot(self)?)
    }
    fn set_profile_velocity(&self, velocity: u32) -> Result<()> {
        Ok(DynamixelMotor::set_profile_velocity(self, velocity)?)
    }
    fn set_profile_acceleration(&self, acceleration: u32) -> Result<()> {
        Ok(DynamixelMotor::set_profile_acceleration(self, acceleration)?)
    }
    fn set_torque_enable(&self, torque: Torque) -> Result<()> {
        Ok(DynamixelMotor::set_torque_enable(self, torque)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn ik_produces_finite_angles_for_nominal_pose() {
        let pitch = 100.0_f64.to_radians();
        let (q1, q1_rev, q2, q3, q4) =
            Arm::inverse_kinematics(120.0, 0.0, 250.0, pitch, 0.0).expect("reachable");
        assert!(q1.is_finite());
        assert!(q1_rev.is_finite());
        assert_relative_eq!(q2, 0.0, epsilon = 1e-9);
        assert!(q3.is_finite());
        assert!(q4.is_finite());
    }
}
