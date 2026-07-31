use anyhow::Result;

/// Linear slide axis wrapper.
///
/// On Windows with the `ajinextek` feature, this talks to the AJINEXTEK AXL/AXM SDK.
/// Otherwise a software stub is used so the project builds on Linux/macOS.
pub struct LinearMotor {
    #[allow(dead_code)]
    axis_no: i32,
    min: f64,
    max: f64,
    target_position: f64,
    #[cfg(not(feature = "ajinextek"))]
    simulated_position: f64,
    #[cfg(not(feature = "ajinextek"))]
    powered: bool,
}

impl LinearMotor {
    pub fn new(axis_no: i32) -> Result<Self> {
        #[cfg(feature = "ajinextek")]
        {
            return Self::new_ajinextek(axis_no);
        }
        #[cfg(not(feature = "ajinextek"))]
        {
            eprintln!(
                "warning: LinearMotor running in stub mode (enable `ajinextek` on Windows for real hardware)"
            );
            Ok(Self {
                axis_no,
                min: 0.0,
                max: 1.3,
                target_position: 0.0,
                simulated_position: 0.0,
                powered: false,
            })
        }
    }

    pub fn has_limit(&self) -> bool {
        self.min != 0.0 && self.max != 0.0
    }

    pub fn map(&self, value: f64, min: f64, max: f64) -> f64 {
        (value - min) / (max - min) * (self.max - self.min) + self.min
    }

    pub fn get_position(&self) -> f64 {
        #[cfg(feature = "ajinextek")]
        {
            return self.ajinextek_get_position();
        }
        #[cfg(not(feature = "ajinextek"))]
        {
            self.simulated_position
        }
    }

    pub fn get_mapped_position(&self, min: f64, max: f64) -> f64 {
        (self.get_position() - self.min) / (self.max - self.min) * (max - min) + min
    }

    pub fn set_position(&mut self, position: f64, wait: bool) {
        let clamped = self.get_clamped_position(position);
        self.target_position = position;
        #[cfg(feature = "ajinextek")]
        {
            self.ajinextek_set_position(clamped, wait);
        }
        #[cfg(not(feature = "ajinextek"))]
        {
            let _ = wait;
            self.simulated_position = clamped;
        }
    }

    pub fn update(&mut self) {
        if !self.is_moving() && (self.get_position() - self.target_position).abs() > 0.1 {
            self.set_position(self.target_position, false);
        }
    }

    pub fn is_moving(&self) -> bool {
        #[cfg(feature = "ajinextek")]
        {
            return self.ajinextek_is_moving();
        }
        #[cfg(not(feature = "ajinextek"))]
        {
            false
        }
    }

    pub fn on(&mut self) {
        #[cfg(feature = "ajinextek")]
        {
            self.ajinextek_on();
        }
        #[cfg(not(feature = "ajinextek"))]
        {
            self.powered = true;
        }
    }

    pub fn off(&mut self) {
        #[cfg(feature = "ajinextek")]
        {
            self.ajinextek_off();
        }
        #[cfg(not(feature = "ajinextek"))]
        {
            self.powered = false;
        }
    }

    fn get_clamped_position(&self, position: f64) -> f64 {
        if !self.has_limit() {
            return position;
        }
        position.clamp(self.min, self.max)
    }

    #[cfg(feature = "ajinextek")]
    fn new_ajinextek(axis_no: i32) -> Result<Self> {
        // FFI bindings would call AxlOpenNoReset / AxmMotSet* here.
        // The proprietary headers are not available in this environment.
        let _ = axis_no;
        anyhow::bail!("ajinextek feature selected but AXL bindings are not linked in this build")
    }

    #[cfg(feature = "ajinextek")]
    fn ajinextek_get_position(&self) -> f64 {
        0.0
    }
    #[cfg(feature = "ajinextek")]
    fn ajinextek_set_position(&mut self, _position: f64, _wait: bool) {}
    #[cfg(feature = "ajinextek")]
    fn ajinextek_is_moving(&self) -> bool {
        false
    }
    #[cfg(feature = "ajinextek")]
    fn ajinextek_on(&mut self) {}
    #[cfg(feature = "ajinextek")]
    fn ajinextek_off(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn map_reverses_table_y_range() {
        let lm = LinearMotor {
            axis_no: 0,
            min: 0.0,
            max: 1.3,
            target_position: 0.0,
            #[cfg(not(feature = "ajinextek"))]
            simulated_position: 0.0,
            #[cfg(not(feature = "ajinextek"))]
            powered: false,
        };
        let y_max = 1.525 - 0.18;
        let y_min = 0.18;
        let mapped = lm.map(y_max, y_max, y_min);
        assert!((mapped - 0.0).abs() < 1e-9);
        let mapped = lm.map(y_min, y_max, y_min);
        assert!((mapped - 1.3).abs() < 1e-9);
    }
}
