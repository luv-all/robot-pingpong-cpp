use crate::constants::{X_TABLE_SIZE, Y_TABLE_SIZE};
use crate::utils::PolynomialRegression;
use nalgebra::{DMatrix, DVector};
use std::time::SystemTime;

const TARGET_X: f64 = X_TABLE_SIZE - 0.03;
const FINAL_PREDICTION_Y: f64 = X_TABLE_SIZE - 0.6;
const FINAL_PREDICTION_Z: f64 = X_TABLE_SIZE - 0.3;
const HIT_X: f64 = X_TABLE_SIZE / 2.0;
const HIT_TIME_DELTA: f64 = 0.15;
const DRAG: f64 = 0.8;

#[derive(Clone, Copy, Debug, Default)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn as_array(&self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
}

#[derive(Clone, Debug)]
struct Sample {
    time: SystemTime,
    pos: Vec3,
}

/// Simple 7-state Kalman filter matching the C++ transition model.
struct BallKalman {
    state: DVector<f64>,
    p: DMatrix<f64>,
    f: DMatrix<f64>,
    h: DMatrix<f64>,
    q: DMatrix<f64>,
    r: DMatrix<f64>,
}

impl BallKalman {
    fn new(position: Vec3) -> Self {
        let mut f = DMatrix::identity(7, 7);
        f[(0, 3)] = 1.0;
        f[(1, 4)] = 1.0;
        f[(2, 5)] = 1.0;
        f[(3, 3)] = DRAG;
        f[(4, 4)] = DRAG;
        f[(5, 5)] = DRAG;
        f[(5, 6)] = 1.0;

        let mut h = DMatrix::zeros(3, 7);
        h[(0, 0)] = 1.0;
        h[(1, 1)] = 1.0;
        h[(2, 2)] = 1.0;

        let mut state = DVector::zeros(7);
        state[0] = position.x;
        state[1] = position.y;
        state[2] = position.z;

        Self {
            state,
            p: DMatrix::identity(7, 7),
            f,
            h,
            q: DMatrix::identity(7, 7) * 0.3,
            r: DMatrix::identity(3, 3) * 0.1,
        }
    }

    fn predict(&mut self) -> Vec3 {
        self.state = &self.f * &self.state;
        self.p = &self.f * &self.p * self.f.transpose() + &self.q;
        Vec3::new(self.state[0], self.state[1], self.state[2])
    }

    fn correct(&mut self, position: Vec3) {
        let z = DVector::from_vec(vec![position.x, position.y, position.z]);
        let s = &self.h * &self.p * self.h.transpose() + &self.r;
        let k = &self.p
            * self.h.transpose()
            * s.try_inverse().unwrap_or_else(|| DMatrix::identity(3, 3));
        let y = z - &self.h * &self.state;
        self.state = &self.state + &k * y;
        let i = DMatrix::identity(7, 7);
        self.p = (i - &k * &self.h) * &self.p;
    }
}

pub struct Predictor {
    history: Vec<Sample>,
    predicted: Vec<Sample>,
    miss_count: i32,
    bound_indices: Vec<usize>,
    bound_quadratic: Vec<Vec<f64>>,
    kalman: Option<BallKalman>,
    y_set: bool,
    y_final_set: bool,
    target_y: f64,
    z_set: bool,
    target_z: f64,
    hit: bool,
}

impl Default for Predictor {
    fn default() -> Self {
        Self::new()
    }
}

impl Predictor {
    pub fn new() -> Self {
        let mut p = Self {
            history: Vec::new(),
            predicted: Vec::new(),
            miss_count: 0,
            bound_indices: Vec::new(),
            bound_quadratic: Vec::new(),
            kalman: None,
            y_set: false,
            y_final_set: false,
            target_y: 0.0,
            z_set: false,
            target_z: 0.0,
            hit: false,
        };
        p.reset();
        p
    }

    pub fn history(&self) -> impl Iterator<Item = Vec3> + '_ {
        self.history.iter().map(|s| s.pos)
    }

    pub fn predicted(&self) -> impl Iterator<Item = Vec3> + '_ {
        self.predicted.iter().map(|s| s.pos)
    }

    pub fn bound_quadratic(&self) -> &[Vec<f64>] {
        &self.bound_quadratic
    }

    pub fn add_ball_position(&mut self, position: Vec3) {
        if position.x < 0.0
            || position.x > X_TABLE_SIZE
            || position.y < 0.0
            || position.y > Y_TABLE_SIZE
        {
            self.add_missing_ball_position();
            return;
        }

        if self.kalman.is_none() {
            self.kalman = Some(BallKalman::new(position));
        }

        self.miss_count = 0;
        let now = SystemTime::now();
        self.history.push(Sample {
            time: now,
            pos: position,
        });

        let predicted = {
            let kalman = self.kalman.as_mut().unwrap();
            let pred = kalman.predict();
            kalman.correct(position);
            pred
        };
        self.predicted.push(Sample {
            time: now,
            pos: predicted,
        });

        if self.history.len() < 3 {
            return;
        }

        let first = self.history[self.history.len() - 3].pos;
        let mid = self.history[self.history.len() - 2].pos;
        if first.x > mid.x && mid.x > position.x {
            if position.x < X_TABLE_SIZE / 2.0 {
                self.reset();
            }
            return;
        }

        if Self::check_is_bounded(first, mid, position) {
            self.bound_indices.push(self.history.len() - 2);
        }

        if self
            .predicted
            .iter()
            .any(|sample| sample.pos.x < X_TABLE_SIZE / 2.0)
        {
            self.predict(position);
        }
    }

    pub fn add_missing_ball_position(&mut self) {
        if self.history.is_empty() {
            return;
        }
        self.miss_count += 1;
        if self.miss_count > 30 {
            self.reset();
        }
    }

    pub fn predict_y(&self) -> Option<f64> {
        if self.y_set {
            Some(self.target_y)
        } else {
            None
        }
    }

    pub fn predict_z(&self) -> Option<f64> {
        if self.z_set {
            Some(self.target_z)
        } else {
            None
        }
    }

    pub fn hit_target(&self) -> bool {
        self.hit
    }

    pub fn get_velocity(&self) -> Vec3 {
        if self.history.len() < 2 {
            return Vec3::default();
        }
        let a = &self.history[self.history.len() - 2];
        let b = &self.history[self.history.len() - 1];
        let dt = duration_secs(a.time, b.time);
        if dt == 0.0 {
            return Vec3::default();
        }
        Vec3::new(
            (b.pos.x - a.pos.x) / dt,
            (b.pos.y - a.pos.y) / dt,
            (b.pos.z - a.pos.z) / dt,
        )
    }

    pub fn get_acceleration(&self) -> Vec3 {
        if self.history.len() < 3 {
            return Vec3::default();
        }
        let a = &self.history[self.history.len() - 3];
        let b = &self.history[self.history.len() - 2];
        let c = &self.history[self.history.len() - 1];
        let dt1 = duration_secs(a.time, b.time);
        let dt2 = duration_secs(b.time, c.time);
        if dt1 == 0.0 || dt2 == 0.0 {
            return Vec3::default();
        }
        let v1 = Vec3::new(
            (b.pos.x - a.pos.x) / dt1,
            (b.pos.y - a.pos.y) / dt1,
            (b.pos.z - a.pos.z) / dt1,
        );
        let v2 = Vec3::new(
            (c.pos.x - b.pos.x) / dt2,
            (c.pos.y - b.pos.y) / dt2,
            (c.pos.z - b.pos.z) / dt2,
        );
        Vec3::new(
            (v2.x - v1.x) / dt2,
            (v2.y - v1.y) / dt2,
            (v2.z - v1.z) / dt2,
        )
    }

    fn predict(&mut self, position: Vec3) {
        if !self.z_set || position.x < FINAL_PREDICTION_Z {
            self.bound_quadratic.clear();
            let start = self.bound_indices.last().copied().unwrap_or(0);
            // Match C++ range that excludes the newest sample (`&history.back()` exclusive).
            let end = self.history.len().saturating_sub(1);
            if end > start {
                let src_x: Vec<f64> = self.history[start..end].iter().map(|s| s.pos.x).collect();
                let src_y: Vec<f64> = self.history[start..end].iter().map(|s| s.pos.z).collect();
                if src_x.len() > 2 {
                    if let Ok(coeffs) = PolynomialRegression::fit(&src_x, &src_y, 2) {
                        self.bound_quadratic.push(coeffs);
                        if self.bound_quadratic.last().unwrap()[2] < 0.0 {
                            for _ in 0..10 {
                                let c = &self.bound_quadratic.last().unwrap();
                                let target_z =
                                    c[0] + c[1] * TARGET_X + c[2] * TARGET_X * TARGET_X;
                                if target_z > 0.0 {
                                    if target_z < 0.5 {
                                        self.target_z = target_z;
                                        self.z_set = true;
                                    }
                                    break;
                                }

                                let a = c[2];
                                let b = c[1];
                                let cc = c[0];
                                let disc = b * b - 4.0 * a * cc;
                                if disc < 0.0 {
                                    break;
                                }
                                let bound_x = (-b - disc.sqrt()) / (2.0 * a);
                                if bound_x.is_nan() {
                                    break;
                                }
                                let p = -b / (2.0 * a);
                                let q = cc - a * p * p;
                                let new_a = a * 0.9;
                                let new_p = p + (bound_x - p) * 2.0;
                                let new_q = q;
                                if (0.0..TARGET_X * 3.0).contains(&bound_x) {
                                    self.bound_quadratic.push(vec![
                                        new_a * new_p * new_p + new_q,
                                        -2.0 * new_a * new_p,
                                        new_a,
                                    ]);
                                } else {
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }

        if !self.y_set && position.x > X_TABLE_SIZE / 2.0 {
            let mut sum_x = 0.0;
            let mut sum_y = 0.0;
            let mut sum_xy = 0.0;
            let mut sum_x2 = 0.0;
            for sample in &self.predicted {
                sum_x += sample.pos.x;
                sum_y += sample.pos.y;
                sum_xy += sample.pos.x * sample.pos.y;
                sum_x2 += sample.pos.x * sample.pos.x;
            }
            let n = self.predicted.len() as f64;
            let denom = n * sum_x2 - sum_x * sum_x;
            if denom != 0.0 {
                let a = (n * sum_xy - sum_x * sum_y) / denom;
                let b = (sum_y - a * sum_x) / n;
                self.target_y = a * TARGET_X + b;
                self.y_set = true;
            }
        }

        if !self.y_final_set && position.x > FINAL_PREDICTION_Y {
            self.target_y = position.y;
            self.y_final_set = true;
        }

        if position.x > HIT_X {
            let front = self.history.first().unwrap();
            let back = self.history.last().unwrap();
            let dt = duration_secs(front.time, back.time);
            if dt > 0.0 {
                let dx = position.x - front.pos.x;
                let vx = dx / dt;
                if vx != 0.0 {
                    let left_t = (TARGET_X - position.x) / vx;
                    if left_t < HIT_TIME_DELTA {
                        self.hit = true;
                    }
                }
            }
        }
    }

    fn reset(&mut self) {
        self.history.clear();
        self.bound_indices.clear();
        self.predicted.clear();
        self.bound_quadratic.clear();
        self.kalman = None;
        self.miss_count = 0;
        self.y_set = false;
        self.y_final_set = false;
        self.z_set = false;
        self.hit = false;
        self.target_y = 0.0;
        self.target_z = 0.0;
    }

    fn check_is_bounded(a: Vec3, b: Vec3, c: Vec3) -> bool {
        a.z > b.z && b.z < c.z
    }
}

fn duration_secs(a: SystemTime, b: SystemTime) -> f64 {
    b.duration_since(a)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_out_of_table_and_resets_after_misses() {
        let mut p = Predictor::new();
        p.add_ball_position(Vec3::new(1.0, 0.5, 0.2));
        assert_eq!(p.history.len(), 1);
        for _ in 0..31 {
            p.add_missing_ball_position();
        }
        assert!(p.history.is_empty());
    }

    #[test]
    fn bounce_detection() {
        assert!(Predictor::check_is_bounded(
            Vec3::new(0.0, 0.0, 0.3),
            Vec3::new(0.0, 0.0, 0.1),
            Vec3::new(0.0, 0.0, 0.2),
        ));
    }
}
