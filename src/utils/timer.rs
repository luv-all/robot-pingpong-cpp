use std::time::Instant;

/// Millisecond-resolution FPS timer matching the C++ `Timer` behavior.
pub struct Timer {
    last: Instant,
}

impl Timer {
    pub fn new() -> Self {
        Self {
            last: Instant::now(),
        }
    }

    pub fn get_fps(&mut self) -> f64 {
        let elapsed_ms = self.last.elapsed().as_millis() as f64;
        self.last = Instant::now();
        if elapsed_ms <= 0.0 {
            return f64::INFINITY;
        }
        1000.0 / elapsed_ms
    }
}

impl Default for Timer {
    fn default() -> Self {
        Self::new()
    }
}
