use crate::constants::{X_TABLE_SIZE, Y_TABLE_SIZE};
use crate::predictor::Predictor;
use anyhow::Result;
use opencv::core::{Mat, MatTraitConst, Point, Rect, Scalar, Size, Vector};
use opencv::imgproc::{self, LINE_8};
use opencv::prelude::*;
use opencv::videoio::{VideoWriter, VideoWriterTrait};
use opencv::{core, highgui};
use std::time::{SystemTime, UNIX_EPOCH};

const WINDOW_NAME: &str = "screen";

pub struct Visualizer {
    screen: Mat,
    vision_screen: Mat,
    window_screen: Mat,
    top: Mat,
    right: Mat,
    ball_visible: bool,
    ball_position: [f64; 3],
    machine_position: f64,
    has_stopped: bool,
    writer: VideoWriter,
}

impl Visualizer {
    pub fn new() -> Result<Self> {
        let ts = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let file_name = format!("output{ts}.mkv");
        let writer = VideoWriter::new(
            &file_name,
            VideoWriter::fourcc('X', '2', '6', '4')?,
            30.0,
            Size::new(1280 * 2, 720 * 2),
            true,
        )?;
        highgui::named_window(WINDOW_NAME, highgui::WINDOW_AUTOSIZE)?;
        Ok(Self {
            screen: Mat::new_rows_cols_with_default(
                720 * 2,
                1280 * 2,
                core::CV_8UC3,
                Scalar::default(),
            )?,
            vision_screen: Mat::new_rows_cols_with_default(
                720,
                1280 * 2,
                core::CV_8UC3,
                Scalar::default(),
            )?,
            window_screen: Mat::new_rows_cols_with_default(
                900,
                1600,
                core::CV_8UC3,
                Scalar::default(),
            )?,
            top: Mat::new_rows_cols_with_default(720, 1280, core::CV_8UC3, Scalar::default())?,
            right: Mat::new_rows_cols_with_default(720, 1280, core::CV_8UC3, Scalar::default())?,
            ball_visible: false,
            ball_position: [0.0; 3],
            machine_position: Y_TABLE_SIZE / 2.0,
            has_stopped: false,
            writer,
        })
    }

    pub fn stopped(&self) -> bool {
        self.has_stopped
    }

    pub fn add_camera(&mut self, _index: usize, _k: &Mat, _r: &Mat, _t: &Mat) -> Result<()> {
        Ok(())
    }

    pub fn set_ball_position(&mut self, pos: crate::predictor::Vec3) {
        self.ball_visible = true;
        self.ball_position = pos.as_array();
    }

    pub fn set_machine_position(&mut self, y: f64) {
        self.machine_position = y;
    }

    pub fn set_screen(&mut self, screen: &Mat) -> Result<()> {
        screen.copy_to(&mut self.vision_screen)?;
        Ok(())
    }

    pub fn render(&mut self, predictor: &Predictor, fps: f64) -> Result<()> {
        self.screen.set_to(&Scalar::default(), &Mat::default())?;
        let mut roi = Mat::roi_mut(
            &mut self.screen,
            Rect::new(0, 0, 1280 * 2, 720),
        )?;
        self.vision_screen.copy_to(&mut roi)?;

        self.top.set_to(&Scalar::default(), &Mat::default())?;
        self.right.set_to(&Scalar::default(), &Mat::default())?;
        self.render_top_right(predictor)?;

        let mut top_roi = Mat::roi_mut(&mut self.screen, Rect::new(0, 720, 1280, 720))?;
        self.top.copy_to(&mut top_roi)?;
        let mut right_roi = Mat::roi_mut(&mut self.screen, Rect::new(1280, 720, 1280, 720))?;
        self.right.copy_to(&mut right_roi)?;

        imgproc::put_text(
            &mut self.screen,
            &format!("FPS: {fps}"),
            Point::new(10, 30),
            imgproc::FONT_HERSHEY_SIMPLEX,
            1.0,
            Scalar::new(0.0, 0.0, 255.0, 0.0),
            1,
            LINE_8,
            false,
        )?;

        if self.ball_visible {
            imgproc::put_text(
                &mut self.screen,
                &format!(
                    "X: {}, Y: {}, Z: {}",
                    self.ball_position[0], self.ball_position[1], self.ball_position[2]
                ),
                Point::new(10, 80),
                imgproc::FONT_HERSHEY_SIMPLEX,
                1.0,
                Scalar::all(255.0),
                1,
                LINE_8,
                false,
            )?;
        }
        if let Some(y) = predictor.predict_y() {
            imgproc::put_text(
                &mut self.screen,
                &format!("Y: {y}"),
                Point::new(10, 130),
                imgproc::FONT_HERSHEY_SIMPLEX,
                1.0,
                Scalar::all(255.0),
                1,
                LINE_8,
                false,
            )?;
        }
        if let Some(z) = predictor.predict_z() {
            imgproc::put_text(
                &mut self.screen,
                &format!("Z: {z}"),
                Point::new(10, 180),
                imgproc::FONT_HERSHEY_SIMPLEX,
                1.0,
                Scalar::all(255.0),
                1,
                LINE_8,
                false,
            )?;
        }
        if predictor.hit_target() {
            imgproc::put_text(
                &mut self.screen,
                "Hit target",
                Point::new(10, 230),
                imgproc::FONT_HERSHEY_SIMPLEX,
                1.0,
                Scalar::new(0.0, 255.0, 0.0, 0.0),
                1,
                LINE_8,
                false,
            )?;
        }

        self.writer.write(&self.screen)?;
        let window_size = Size::new(self.window_screen.cols(), self.window_screen.rows());
        imgproc::resize(
            &self.screen,
            &mut self.window_screen,
            window_size,
            0.0,
            0.0,
            imgproc::INTER_LINEAR,
        )?;
        highgui::imshow(WINDOW_NAME, &self.window_screen)?;
        if highgui::wait_key(1)? == 27 {
            self.has_stopped = true;
        }
        self.ball_visible = false;
        Ok(())
    }

    fn render_top_right(&mut self, predictor: &Predictor) -> Result<()> {
        self.circle([0.0, 0.0, 0.0], 3, Scalar::new(0.0, 0.0, 255.0, 0.0), 1)?;
        self.rect(
            [0.0, 0.0, 0.0],
            [X_TABLE_SIZE, Y_TABLE_SIZE, 0.0],
            Scalar::all(255.0),
        )?;
        self.rect(
            [X_TABLE_SIZE / 2.0, -0.1525, 0.0],
            [X_TABLE_SIZE / 2.0, Y_TABLE_SIZE + 0.1525, 0.15],
            Scalar::all(255.0),
        )?;
        if self.ball_visible {
            self.circle(self.ball_position, 10, Scalar::new(0.0, 0.0, 255.0, 0.0), -1)?;
        }
        imgproc::circle(
            &mut self.top,
            convert_to_top([X_TABLE_SIZE, self.machine_position, 0.0]),
            10,
            Scalar::new(0.0, 0.0, 255.0, 0.0),
            -1,
            LINE_8,
            0,
        )?;

        for pos in predictor.history() {
            self.circle(pos.as_array(), 5, Scalar::new(0.0, 255.0, 0.0, 0.0), -1)?;
        }
        for pos in predictor.predicted() {
            self.circle(pos.as_array(), 3, Scalar::new(0.0, 0.0, 255.0, 0.0), -1)?;
        }

        let y = predictor.predict_y().unwrap_or(0.0);
        let z = predictor.predict_z().unwrap_or(0.0);
        if y != 0.0 {
            self.line(
                [0.0, y, 0.0],
                [X_TABLE_SIZE - 0.1, y, 0.0],
                Scalar::new(0.0, 255.0, 0.0, 0.0),
                2,
            )?;
            self.line(
                [X_TABLE_SIZE - 0.1, y, 0.0],
                [X_TABLE_SIZE - 0.1, y, 4.0],
                Scalar::new(0.0, 255.0, 0.0, 0.0),
                2,
            )?;
        }
        if z != 0.0 {
            self.line(
                [X_TABLE_SIZE - 0.2, y, z],
                [X_TABLE_SIZE, y, z],
                Scalar::new(0.0, 255.0, 0.0, 0.0),
                2,
            )?;
        }

        for coeffs in predictor.bound_quadratic() {
            let mut points = Vector::<Point>::new();
            let mut x = 0.0;
            while x < X_TABLE_SIZE {
                let height = coeffs[0] + coeffs[1] * x + coeffs[2] * x * x;
                points.push(convert_to_right([x, 0.0, height]));
                x += 0.01;
            }
            imgproc::polylines(
                &mut self.right,
                &Vector::<Vector<Point>>::from_iter([points]),
                false,
                Scalar::new(255.0, 255.0, 0.0, 0.0),
                2,
                LINE_8,
                0,
            )?;
        }
        Ok(())
    }

    fn circle(&mut self, center: [f64; 3], radius: i32, color: Scalar, thickness: i32) -> Result<()> {
        imgproc::circle(
            &mut self.top,
            convert_to_top(center),
            radius,
            color,
            thickness,
            LINE_8,
            0,
        )?;
        imgproc::circle(
            &mut self.right,
            convert_to_right(center),
            radius,
            color,
            thickness,
            LINE_8,
            0,
        )?;
        Ok(())
    }

    fn line(&mut self, start: [f64; 3], end: [f64; 3], color: Scalar, thickness: i32) -> Result<()> {
        imgproc::line(
            &mut self.top,
            convert_to_top(start),
            convert_to_top(end),
            color,
            thickness,
            LINE_8,
            0,
        )?;
        imgproc::line(
            &mut self.right,
            convert_to_right(start),
            convert_to_right(end),
            color,
            thickness,
            LINE_8,
            0,
        )?;
        Ok(())
    }

    fn rect(&mut self, pt1: [f64; 3], pt2: [f64; 3], color: Scalar) -> Result<()> {
        imgproc::rectangle_points(
            &mut self.top,
            convert_to_top(pt1),
            convert_to_top(pt2),
            color,
            1,
            LINE_8,
            0,
        )?;
        imgproc::rectangle_points(
            &mut self.right,
            convert_to_right(pt1),
            convert_to_right(pt2),
            color,
            1,
            LINE_8,
            0,
        )?;
        Ok(())
    }
}

impl Drop for Visualizer {
    fn drop(&mut self) {
        let _ = self.writer.release();
        let _ = highgui::destroy_window(WINDOW_NAME);
    }
}

fn convert_to_top(vec: [f64; 3]) -> Point {
    Point::new(
        640 + ((vec[0] - X_TABLE_SIZE / 2.0) * 300.0) as i32,
        360 - ((vec[1] - Y_TABLE_SIZE / 2.0) * 300.0) as i32,
    )
}

fn convert_to_right(vec: [f64; 3]) -> Point {
    Point::new(
        640 + ((vec[0] - X_TABLE_SIZE / 2.0) * 300.0) as i32,
        360 - ((vec[2] - 0.15) * 300.0) as i32,
    )
}
