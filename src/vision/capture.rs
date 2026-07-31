use crate::constants::{
    CAPTURE_HEIGHT, CAPTURE_WIDTH, CIRCULARITY_THRESHOLD, MAX_AREA, MIN_AREA,
    MIN_COLOR_SIMILARITY, ORANGE_HSV, ORANGE_HSV_LOWER, ORANGE_HSV_UPPER,
};
use anyhow::{bail, Context, Result};
use opencv::core::{
    bitwise_and, Point, Point2f, Rect, Scalar, Size, Vector, BORDER_CONSTANT, CV_8UC1,
};
use opencv::imgproc::{self, COLOR_BGR2HSV, COLOR_HSV2BGR, LINE_8, MORPH_ELLIPSE, MORPH_OPEN};
use opencv::prelude::*;
use opencv::video::BackgroundSubtractorMOG2;
use opencv::videoio::{
    VideoCapture, VideoCaptureTrait, VideoCaptureTraitConst, VideoWriter, CAP_PROP_AUTO_EXPOSURE,
    CAP_PROP_EXPOSURE, CAP_PROP_FOURCC, CAP_PROP_FPS, CAP_PROP_FRAME_HEIGHT, CAP_PROP_FRAME_WIDTH,
};
use parking_lot::Mutex;
use std::sync::Arc;
use opencv::{highgui, video};

pub struct Capture {
    capture: VideoCapture,
    pub frame: Mat,
    global_mask: Mat,
    bg_subtractor: core::Ptr<BackgroundSubtractorMOG2>,
    morph_kernel: Mat,
    mask_points: Vector<Point>,
    table_area: Vector<Point2f>,
}

// Re-export Ptr path for BackgroundSubtractor
use opencv::core;

impl Capture {
    pub fn new(device_id: i32, api_preference: i32) -> Result<Self> {
        let mut capture = VideoCapture::new(device_id, api_preference)
            .with_context(|| format!("open camera {device_id}"))?;
        if !capture.is_opened()? {
            bail!("Error: Could not open camera {device_id}");
        }
        capture.set(CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH as f64)?;
        capture.set(CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT as f64)?;
        capture.set(CAP_PROP_FPS, 60.0)?;
        capture.set(CAP_PROP_EXPOSURE, -7.0)?;
        capture.set(CAP_PROP_AUTO_EXPOSURE, 0.25)?;
        let fourcc = VideoWriter::fourcc('M', 'J', 'P', 'G')?;
        capture.set(CAP_PROP_FOURCC, fourcc as f64)?;

        let mut img = Mat::default();
        capture.read(&mut img)?;
        imgproc::resize(
            &img.clone(),
            &mut img,
            Size::new(CAPTURE_WIDTH, CAPTURE_HEIGHT),
            0.0,
            0.0,
            imgproc::INTER_LINEAR,
        )?;

        let global_mask = Mat::new_rows_cols_with_default(
            img.rows(),
            img.cols(),
            CV_8UC1,
            Scalar::all(255.0),
        )?;
        let bg_subtractor = video::create_background_subtractor_mog2(500, 32.0, false)?;
        let morph_kernel =
            imgproc::get_structuring_element(MORPH_ELLIPSE, Size::new(3, 3), Point::new(-1, -1))?;

        Ok(Self {
            capture,
            frame: img,
            global_mask,
            bg_subtractor,
            morph_kernel,
            mask_points: Vector::new(),
            table_area: Vector::new(),
        })
    }

    pub fn set_global_mask(
        &mut self,
        window_name: &str,
        initial_points: Vec<Point>,
        skip: bool,
    ) -> Result<Vec<Point>> {
        let points = Arc::new(Mutex::new(initial_points));
        if !skip {
            highgui::named_window(window_name, highgui::WINDOW_AUTOSIZE)?;
            highgui::move_window(window_name, 0, 0)?;
            let click_points = Arc::clone(&points);
            highgui::set_mouse_callback(
                window_name,
                Some(Box::new(move |event, x, y, _flags| {
                    if event == highgui::EVENT_LBUTTONDOWN {
                        click_points.lock().push(Point::new(x, y));
                    }
                })),
            )?;

            loop {
                let mut screen = Mat::default();
                self.capture.read(&mut screen)?;
                imgproc::resize(
                    &screen.clone(),
                    &mut screen,
                    Size::new(CAPTURE_WIDTH, CAPTURE_HEIGHT),
                    0.0,
                    0.0,
                    imgproc::INTER_LINEAR,
                )?;
                let current = points.lock().clone();
                for (idx, pt) in current.iter().enumerate() {
                    imgproc::circle(
                        &mut screen,
                        *pt,
                        5,
                        Scalar::new(0.0, 0.0, 255.0, 0.0),
                        -1,
                        LINE_8,
                        0,
                    )?;
                    if current.len() > 1 {
                        let next = current[(idx + 1) % current.len()];
                        imgproc::line(
                            &mut screen,
                            *pt,
                            next,
                            Scalar::new(0.0, 0.0, 255.0, 0.0),
                            2,
                            LINE_8,
                            0,
                        )?;
                    }
                }
                if current.len() > 2 {
                    let mut overlay = Mat::zeros(screen.rows(), screen.cols(), screen.typ())?.to_mat()?;
                    let pts = Vector::<Point>::from_iter(current.iter().copied());
                    let polys = Vector::<Vector<Point>>::from_iter([pts]);
                    imgproc::fill_poly(
                        &mut overlay,
                        &polys,
                        Scalar::all(255.0),
                        LINE_8,
                        0,
                        Point::default(),
                    )?;
                    let mut blended = Mat::default();
                    core::add_weighted(&screen, 1.0, &overlay, 0.5, 0.0, &mut blended, -1)?;
                    screen = blended;
                }
                highgui::imshow(window_name, &screen)?;
                let key = highgui::wait_key(1)?;
                if key == 27 {
                    break;
                }
                if key == 8 || key == 127 {
                    let mut guard = points.lock();
                    if !guard.is_empty() {
                        guard.pop();
                    }
                }
            }
            highgui::destroy_window(window_name)?;
        }

        let points = match Arc::try_unwrap(points) {
            Ok(mutex) => mutex.into_inner(),
            Err(arc) => arc.lock().clone(),
        };
        self.global_mask.set_to(&Scalar::all(0.0), &Mat::default())?;
        let pts = Vector::<Point>::from_iter(points.iter().copied());
        let polys = Vector::<Vector<Point>>::from_iter([pts]);
        imgproc::fill_poly(
            &mut self.global_mask,
            &polys,
            Scalar::all(255.0),
            LINE_8,
            0,
            Point::default(),
        )?;
        self.mask_points = Vector::from_iter(points.iter().copied());
        Ok(points)
    }

    pub fn get_table_area(
        &mut self,
        window_name: &str,
        initial_points: Vec<Point2f>,
    ) -> Result<Vec<Point2f>> {
        let points = Arc::new(Mutex::new(initial_points));
        highgui::named_window(window_name, highgui::WINDOW_AUTOSIZE)?;
        highgui::move_window(window_name, 0, 0)?;
        let click_points = Arc::clone(&points);
        highgui::set_mouse_callback(
            window_name,
            Some(Box::new(move |event, x, y, _flags| {
                if event == highgui::EVENT_LBUTTONDOWN {
                    click_points
                        .lock()
                        .push(Point2f::new(x as f32, y as f32));
                }
            })),
        )?;

        loop {
            let mut screen = Mat::default();
            self.capture.read(&mut screen)?;
            imgproc::resize(
                &screen.clone(),
                &mut screen,
                Size::new(CAPTURE_WIDTH, CAPTURE_HEIGHT),
                0.0,
                0.0,
                imgproc::INTER_LINEAR,
            )?;
            let current = points.lock().clone();
            for (idx, pt) in current.iter().enumerate() {
                let ip = Point::new(pt.x as i32, pt.y as i32);
                imgproc::circle(
                    &mut screen,
                    ip,
                    5,
                    Scalar::new(0.0, 0.0, 255.0, 0.0),
                    -1,
                    LINE_8,
                    0,
                )?;
                imgproc::put_text(
                    &mut screen,
                    &format!("P{}", idx + 1),
                    ip,
                    imgproc::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    Scalar::new(0.0, 0.0, 255.0, 0.0),
                    1,
                    LINE_8,
                    false,
                )?;
            }
            highgui::imshow(window_name, &screen)?;
            let key = highgui::wait_key(1)?;
            if key == 27 {
                break;
            }
            if key == 8 || key == 127 {
                let mut guard = points.lock();
                if !guard.is_empty() {
                    guard.pop();
                }
            }
        }
        let points = match Arc::try_unwrap(points) {
            Ok(mutex) => mutex.into_inner(),
            Err(arc) => arc.lock().clone(),
        };
        if points.len() != 6 {
            bail!("Error: Table area must have 6 points.");
        }
        highgui::destroy_window(window_name)?;
        self.table_area = Vector::from_iter(points.iter().copied());
        Ok(points)
    }

    pub fn capture_frame(&mut self) -> Result<()> {
        self.capture.read(&mut self.frame)?;
        let mut resized = Mat::default();
        imgproc::resize(
            &self.frame,
            &mut resized,
            Size::new(CAPTURE_WIDTH, CAPTURE_HEIGHT),
            0.0,
            0.0,
            imgproc::INTER_LINEAR,
        )?;
        self.frame = resized;
        Ok(())
    }

    pub fn render(&mut self, out: &mut Mat, point: &mut Point2f) -> Result<bool> {
        let mut copy = Mat::default();
        self.frame.copy_to(&mut copy)?;
        *out = Mat::zeros(self.frame.rows(), self.frame.cols(), self.frame.typ())?.to_mat()?;

        let mut hsv = Mat::default();
        imgproc::cvt_color(&self.frame, &mut hsv, COLOR_BGR2HSV, 0)?;
        let mut gray_mask = Mat::default();
        core::in_range(
            &hsv,
            &Scalar::new(
                ORANGE_HSV_LOWER[0] as f64,
                ORANGE_HSV_LOWER[1] as f64,
                ORANGE_HSV_LOWER[2] as f64,
                0.0,
            ),
            &Scalar::new(
                ORANGE_HSV_UPPER[0] as f64,
                ORANGE_HSV_UPPER[1] as f64,
                ORANGE_HSV_UPPER[2] as f64,
                0.0,
            ),
            &mut gray_mask,
        )?;
        let mut masked = Mat::default();
        bitwise_and(&gray_mask, &self.global_mask, &mut masked, &Mat::default())?;
        gray_mask = masked;

        let mut hsv_masked = Mat::default();
        bitwise_and(&hsv, &hsv, &mut hsv_masked, &gray_mask)?;
        imgproc::cvt_color(&hsv_masked, out, COLOR_HSV2BGR, 0)?;
        out.copy_to(&mut copy)?;
        *out = Mat::zeros(self.frame.rows(), self.frame.cols(), self.frame.typ())?.to_mat()?;

        opencv::prelude::BackgroundSubtractorTrait::apply(
            &mut self.bg_subtractor,
            &gray_mask.clone(),
            &mut gray_mask,
            -1.0,
        )?;
        imgproc::morphology_ex(
            &gray_mask.clone(),
            &mut gray_mask,
            MORPH_OPEN,
            &self.morph_kernel,
            Point::new(-1, -1),
            3,
            BORDER_CONSTANT,
            imgproc::morphology_default_border_value()?,
        )?;
        let mut masked2 = Mat::default();
        bitwise_and(&gray_mask, &self.global_mask, &mut masked2, &Mat::default())?;
        gray_mask = masked2;
        bitwise_and(&copy, &copy, out, &gray_mask)?;
        out.copy_to(&mut copy)?;

        if !self.mask_points.is_empty() {
            imgproc::polylines(
                &mut self.frame,
                &Vector::<Vector<Point>>::from_iter([self.mask_points.clone()]),
                true,
                Scalar::new(0.0, 0.0, 255.0, 0.0),
                2,
                LINE_8,
                0,
            )?;
        }
        for i in 0..self.table_area.len() {
            let p = self.table_area.get(i)?;
            imgproc::circle(
                &mut self.frame,
                Point::new(p.x as i32, p.y as i32),
                5,
                Scalar::new(0.0, 0.0, 255.0, 0.0),
                -1,
                LINE_8,
                0,
            )?;
        }

        let mut contours = Vector::<Vector<Point>>::new();
        imgproc::find_contours(
            &gray_mask,
            &mut contours,
            imgproc::RETR_EXTERNAL,
            imgproc::CHAIN_APPROX_SIMPLE,
            Point::default(),
        )?;

        let mut min_color_similarity = MIN_COLOR_SIMILARITY;
        let mut max_contour_index: i32 = -1;
        let orange = Scalar::new(ORANGE_HSV[0], ORANGE_HSV[1], ORANGE_HSV[2], 0.0);

        for i in 0..contours.len() {
            let contour = contours.get(i)?;
            let area = imgproc::contour_area(&contour, false)?;
            if area < MIN_AREA || area > MAX_AREA {
                imgproc::draw_contours(
                    &mut copy,
                    &contours,
                    i as i32,
                    Scalar::new(255.0, 0.0, 255.0, 0.0),
                    2,
                    LINE_8,
                    &core::no_array(),
                    i32::MAX,
                    Point::default(),
                )?;
                continue;
            }
            let perimeter = imgproc::arc_length(&contour, true)?;
            let circularity = 4.0 * std::f64::consts::PI * area / (perimeter * perimeter);
            if circularity < CIRCULARITY_THRESHOLD {
                imgproc::draw_contours(
                    &mut copy,
                    &contours,
                    i as i32,
                    Scalar::new(0.0, 255.0, 255.0, 0.0),
                    2,
                    LINE_8,
                    &core::no_array(),
                    i32::MAX,
                    Point::default(),
                )?;
                continue;
            }

            gray_mask.set_to(&Scalar::all(0.0), &Mat::default())?;
            imgproc::draw_contours(
                &mut gray_mask,
                &contours,
                i as i32,
                Scalar::all(255.0),
                -1,
                LINE_8,
                &core::no_array(),
                i32::MAX,
                Point::default(),
            )?;
            imgproc::draw_contours(
                &mut copy,
                &contours,
                i as i32,
                Scalar::new(0.0, 255.0, 0.0, 0.0),
                2,
                LINE_8,
                &core::no_array(),
                i32::MAX,
                Point::default(),
            )?;
            let mean = core::mean(&hsv, &gray_mask)?;
            let diff = ((mean[0] - orange[0]).powi(2)
                + (mean[1] - orange[1]).powi(2)
                + (mean[2] - orange[2]).powi(2))
            .sqrt();
            if diff < min_color_similarity {
                min_color_similarity = diff;
                max_contour_index = i as i32;
            }
        }

        core::add_weighted(&self.frame, 0.3, &copy, 0.7, 0.0, out, -1)?;

        if max_contour_index != -1 {
            imgproc::draw_contours(
                out,
                &contours,
                max_contour_index,
                Scalar::new(0.0, 0.0, 255.0, 0.0),
                2,
                LINE_8,
                &core::no_array(),
                i32::MAX,
                Point::default(),
            )?;
            gray_mask.set_to(&Scalar::all(0.0), &Mat::default())?;
            imgproc::draw_contours(
                &mut gray_mask,
                &contours,
                max_contour_index,
                Scalar::all(255.0),
                -1,
                LINE_8,
                &core::no_array(),
                i32::MAX,
                Point::default(),
            )?;
            let moments = imgproc::moments(&gray_mask, false)?;
            if moments.m00 != 0.0 {
                point.x = (moments.m10 / moments.m00) as f32;
                point.y = (moments.m01 / moments.m00) as f32;
                return Ok(true);
            }
        }
        let _ = Rect::default();
        Ok(false)
    }
}
