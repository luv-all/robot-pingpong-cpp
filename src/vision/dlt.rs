use anyhow::{bail, Result};
use nalgebra::{DMatrix, Matrix3, Matrix3x4, Vector3, Vector4};
use opencv::core::{Mat, MatTraitConst, Point2d, Point3d};
use opencv::prelude::*;
use opencv::{calib3d, core};

/// Direct Linear Transform camera pose estimation (ported from C++ `DLT::pose`).
pub struct Dlt;

impl Dlt {
    pub fn pose(
        object_points: &[Point3d],
        image_points: &[Point2d],
    ) -> Result<(Mat, Mat, Mat, Mat)> {
        if object_points.len() != image_points.len() || object_points.is_empty() {
            bail!("object/image point count mismatch");
        }
        let n = object_points.len();

        let mut object = DMatrix::zeros(n, 4);
        for (i, p) in object_points.iter().enumerate() {
            object[(i, 0)] = p.x;
            object[(i, 1)] = p.y;
            object[(i, 2)] = p.z;
            object[(i, 3)] = 1.0;
        }

        let mean_x = image_points.iter().map(|p| p.x).sum::<f64>() / n as f64;
        let mean_y = image_points.iter().map(|p| p.y).sum::<f64>() / n as f64;
        let std_x = (image_points
            .iter()
            .map(|p| (p.x - mean_x).powi(2))
            .sum::<f64>()
            / n as f64)
            .sqrt();
        let std_y = (image_points
            .iter()
            .map(|p| (p.y - mean_y).powi(2))
            .sum::<f64>()
            / n as f64)
            .sqrt();
        if std_x == 0.0 || std_y == 0.0 {
            bail!("image points have zero variance");
        }

        let t = Matrix3::new(
            1.0 / std_x,
            0.0,
            -mean_x / std_x,
            0.0,
            1.0 / std_y,
            -mean_y / std_y,
            0.0,
            0.0,
            1.0,
        );

        let mut normalized = DMatrix::zeros(3, n);
        for (i, p) in image_points.iter().enumerate() {
            let v = t * Vector3::new(p.x, p.y, 1.0);
            normalized.set_column(i, &v);
        }

        let mut a = DMatrix::zeros(3 * n, 12 + n);
        for i in 0..n {
            for j in 0..4 {
                a[(3 * i, j)] = object[(i, j)];
                a[(3 * i + 1, 4 + j)] = object[(i, j)];
                a[(3 * i + 2, 8 + j)] = object[(i, j)];
            }
            a[(3 * i, 12 + i)] = -normalized[(0, i)];
            a[(3 * i + 1, 12 + i)] = -normalized[(1, i)];
            a[(3 * i + 2, 12 + i)] = -normalized[(2, i)];
        }

        let svd = a.svd(true, true);
        let vt = svd.v_t.ok_or_else(|| anyhow::anyhow!("SVD missing V^T"))?;
        let last = vt.row(vt.nrows() - 1);
        let vals: Vec<f64> = last.iter().take(12).copied().collect();
        let mut p_norm = Matrix3x4::new(
            vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7], vals[8],
            vals[9], vals[10], vals[11],
        );

        let obj1 = Vector4::new(object[(1, 0)], object[(1, 1)], object[(1, 2)], object[(1, 3)]);
        let test_sign = p_norm * obj1;
        if test_sign.z < 0.0 {
            p_norm = -p_norm;
        }

        let t_inv = t.try_inverse().unwrap();
        let p = t_inv * p_norm;

        let mut p_mat = unsafe { Mat::new_rows_cols(3, 4, core::CV_64F)? };
        for r in 0..3 {
            for c in 0..4 {
                *p_mat.at_2d_mut::<f64>(r, c)? = p[(r as usize, c as usize)];
            }
        }

        let mut k = Mat::default();
        let mut r_mat = Mat::default();
        let mut t_out = Mat::default();
        let mut rx = Mat::default();
        let mut ry = Mat::default();
        let mut rz = Mat::default();
        calib3d::decompose_projection_matrix(
            &p_mat,
            &mut k,
            &mut r_mat,
            &mut t_out,
            &mut rx,
            &mut ry,
            &mut rz,
            &mut core::Vector::<f64>::new(),
        )?;

        let mut m = unsafe { Mat::new_rows_cols(3, 3, core::CV_64F)? };
        for r in 0..3 {
            for c in 0..3 {
                *m.at_2d_mut::<f64>(r, c)? = *p_mat.at_2d::<f64>(r, c)?;
            }
        }
        let mut p4 = unsafe { Mat::new_rows_cols(3, 1, core::CV_64F)? };
        for r in 0..3 {
            *p4.at_2d_mut::<f64>(r, 0)? = *p_mat.at_2d::<f64>(r, 3)?;
        }
        let mut m_inv = Mat::default();
        core::invert(&m, &mut m_inv, core::DECOMP_LU)?;
        let mut center = Mat::default();
        core::gemm(
            &m_inv,
            &p4,
            -1.0,
            &Mat::default(),
            0.0,
            &mut center,
            0,
        )?;

        let mut r_inv = Mat::default();
        core::invert(&r_mat, &mut r_inv, core::DECOMP_LU)?;

        Ok((k, r_inv, center, p_mat))
    }
}
