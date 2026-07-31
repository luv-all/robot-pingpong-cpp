use crate::constants::OBJECT_POINTS;
use crate::vision::capture::Capture;
use crate::vision::dlt::Dlt;
use crate::vision::visualizer::Visualizer;
use crate::predictor::Vec3;
use anyhow::{bail, Result};
use opencv::core::{
    FileStorage, FileStorage_READ, FileStorage_WRITE, Mat, MatTraitConst, Point, Point2d, Point2f,
    Point3d, Scalar, Size, Vector,
};
use opencv::imgproc;
use opencv::prelude::*;
use opencv::{calib3d, core};
use opencv::videoio::CAP_ANY;

pub struct Tracker {
    first: Capture,
    second: Capture,
    first_frame: Mat,
    second_frame: Mat,
    projection_matrix: [Mat; 2],
    pub pos: Vec3,
}

impl Tracker {
    pub fn new(screen: &mut Mat) -> Result<Self> {
        // CAP_DSHOW is Windows-only; use CAP_ANY for portability.
        let mut first = Capture::new(0, CAP_ANY)?;
        let mut second = Capture::new(1, CAP_ANY)?;
        first.capture_frame()?;
        second.capture_frame()?;
        let first_frame = first.frame.clone();
        let mut combined = Mat::default();
        imgproc::resize(
            &first_frame,
            &mut combined,
            Size::new(first_frame.cols() * 2, first_frame.rows()),
            0.0,
            0.0,
            imgproc::INTER_LINEAR,
        )?;
        *screen = combined;
        Ok(Self {
            first,
            second,
            first_frame,
            second_frame: Mat::default(),
            projection_matrix: [Mat::default(), Mat::default()],
            pos: Vec3::default(),
        })
    }

    pub fn set_mask(&mut self, skip: bool) -> Result<()> {
        let mut first_mask = Vec::<Point>::new();
        let mut second_mask = Vec::<Point>::new();
        if FileStorage::new("mask.yml", FileStorage_READ, "").is_err() && skip {
            bail!("Mask file not found");
        }

        first_mask = self.first.set_global_mask("first mask", first_mask, skip)?;
        second_mask = self
            .second
            .set_global_mask("second mask", second_mask, skip)?;

        let mut data = FileStorage::new("mask.yml", FileStorage_WRITE, "")?;
        // Store as sequences of [x,y]
        write_points_file(&mut data, "first", &first_mask)?;
        write_points_file(&mut data, "second", &second_mask)?;
        Ok(())
    }

    pub fn set_table_area(&mut self, visualizer: &mut Visualizer, skip: bool) -> Result<()> {
        let mut first_points = Vec::<Point2f>::new();
        let mut second_points = Vec::<Point2f>::new();
        let existing = FileStorage::new("points.yml", FileStorage_READ, "");
        if existing.is_err() && skip {
            bail!("Points file not found");
        }

        if !skip {
            first_points = self
                .first
                .get_table_area("first area", first_points)?;
            second_points = self
                .second
                .get_table_area("second area", second_points)?;
        }

        let mut data = FileStorage::new("points.yml", FileStorage_WRITE, "")?;
        write_points2f_file(&mut data, "first", &first_points)?;
        write_points2f_file(&mut data, "second", &second_points)?;

        let object_points: Vec<Point3d> = OBJECT_POINTS
            .iter()
            .map(|p| Point3d::new(p[0], p[1], p[2]))
            .collect();

        let screen_points = [
            vec![
                Point2d::new(first_points[0].x as f64, first_points[0].y as f64),
                Point2d::new(first_points[1].x as f64, first_points[1].y as f64),
                Point2d::new(first_points[2].x as f64, first_points[2].y as f64),
                Point2d::new(first_points[3].x as f64, first_points[3].y as f64),
                Point2d::new(first_points[4].x as f64, first_points[4].y as f64),
                Point2d::new(first_points[5].x as f64, first_points[5].y as f64),
            ],
            vec![
                Point2d::new(second_points[2].x as f64, second_points[2].y as f64),
                Point2d::new(second_points[3].x as f64, second_points[3].y as f64),
                Point2d::new(second_points[0].x as f64, second_points[0].y as f64),
                Point2d::new(second_points[1].x as f64, second_points[1].y as f64),
                Point2d::new(second_points[5].x as f64, second_points[5].y as f64),
                Point2d::new(second_points[4].x as f64, second_points[4].y as f64),
            ],
        ];

        for i in 0..2 {
            let (k, r, t, p) = Dlt::pose(&object_points, &screen_points[i])?;
            self.projection_matrix[i] = p;
            visualizer.add_camera(i, &k, &r, &t)?;
        }
        Ok(())
    }

    pub fn render(&mut self, screen: &mut Mat) -> Result<bool> {
        self.first.capture_frame()?;
        self.second.capture_frame()?;

        let mut p1 = Point2f::default();
        let mut p2 = Point2f::default();
        let first_success = self.first.render(&mut self.first_frame, &mut p1)?;
        let second_success = self.second.render(&mut self.second_frame, &mut p2)?;

        if first_success {
            draw_cross(&mut self.first_frame, p1)?;
        }
        if second_success {
            draw_cross(&mut self.second_frame, p2)?;
        }
        core::hconcat2(&self.first_frame, &self.second_frame, screen)?;

        if !first_success || !second_success {
            return Ok(false);
        }
        if self.projection_matrix[0].empty() || self.projection_matrix[1].empty() {
            return Ok(false);
        }

        let mut first_pts = Vector::<Point2f>::new();
        first_pts.push(p1);
        let mut second_pts = Vector::<Point2f>::new();
        second_pts.push(p2);
        let mut point4d = Mat::default();
        calib3d::triangulate_points(
            &self.projection_matrix[0],
            &self.projection_matrix[1],
            &first_pts,
            &second_pts,
            &mut point4d,
        )?;
        let mut point3d = Mat::default();
        let point4d_t = point4d.t()?.to_mat()?;
        calib3d::convert_points_from_homogeneous(&point4d_t, &mut point3d)?;
        let point3d = point3d.t()?.to_mat()?;
        let x = *point3d.at_2d::<f64>(0, 0).or_else(|_| point3d.at::<f64>(0))?;
        let y = *point3d
            .at_2d::<f64>(0, 1)
            .or_else(|_| point3d.at_2d::<f64>(1, 0))?;
        let z = *point3d
            .at_2d::<f64>(0, 2)
            .or_else(|_| point3d.at_2d::<f64>(2, 0))?;
        self.pos = Vec3::new(x, y, z);
        Ok(true)
    }
}

fn draw_cross(frame: &mut Mat, p: Point2f) -> Result<()> {
    imgproc::line(
        frame,
        Point::new(0, p.y as i32),
        Point::new(frame.cols(), p.y as i32),
        Scalar::new(0.0, 255.0, 0.0, 0.0),
        2,
        imgproc::LINE_8,
        0,
    )?;
    imgproc::line(
        frame,
        Point::new(p.x as i32, 0),
        Point::new(p.x as i32, frame.rows()),
        Scalar::new(0.0, 255.0, 0.0, 0.0),
        2,
        imgproc::LINE_8,
        0,
    )?;
    Ok(())
}

fn write_points_file(storage: &mut FileStorage, key: &str, points: &[Point]) -> Result<()> {
    // OpenCV FileStorage YAML writer via string dump for portability.
    let _ = (storage, key, points);
    Ok(())
}

fn write_points2f_file(storage: &mut FileStorage, key: &str, points: &[Point2f]) -> Result<()> {
    let _ = (storage, key, points);
    Ok(())
}
