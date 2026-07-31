use anyhow::{bail, Result};

/// Least-squares polynomial regression (ported from the C++ header).
pub struct PolynomialRegression;

impl PolynomialRegression {
    pub fn fit(x: &[f64], y: &[f64], order: usize) -> Result<Vec<f64>> {
        if x.len() != y.len() {
            bail!("The size of x & y arrays are different");
        }
        if x.is_empty() {
            bail!("The size of x or y arrays is 0");
        }

        let n = order;
        let np1 = n + 1;
        let np2 = n + 2;
        let tnp1 = 2 * n + 1;
        let n_points = x.len();

        let mut powers = vec![0.0; tnp1];
        for (i, slot) in powers.iter_mut().enumerate() {
            for &xj in x {
                *slot += xj.powi(i as i32);
            }
        }

        let mut b = vec![vec![0.0; np2]; np1];
        for i in 0..=n {
            for j in 0..=n {
                b[i][j] = powers[i + j];
            }
        }

        let mut y_sum = vec![0.0; np1];
        for (i, slot) in y_sum.iter_mut().enumerate() {
            for j in 0..n_points {
                *slot += x[j].powi(i as i32) * y[j];
            }
        }
        for i in 0..=n {
            b[i][np1] = y_sum[i];
        }

        let size = n + 1;
        let nm1 = size - 1;

        for i in 0..size {
            for k in (i + 1)..size {
                if b[i][i] < b[k][i] {
                    b.swap(i, k);
                }
            }
        }

        for i in 0..nm1 {
            for k in (i + 1)..size {
                let t = b[k][i] / b[i][i];
                for j in 0..=size {
                    b[k][j] -= t * b[i][j];
                }
            }
        }

        let mut a = vec![0.0; size];
        for i in (0..size).rev() {
            a[i] = b[i][size];
            for j in 0..size {
                if j != i {
                    a[i] -= b[i][j] * a[j];
                }
            }
            a[i] /= b[i][i];
        }
        Ok(a)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn fits_line() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![1.0, 3.0, 5.0, 7.0];
        let coeffs = PolynomialRegression::fit(&x, &y, 1).unwrap();
        assert_relative_eq!(coeffs[0], 1.0, epsilon = 1e-9);
        assert_relative_eq!(coeffs[1], 2.0, epsilon = 1e-9);
    }

    #[test]
    fn fits_quadratic() {
        // y = 1 + 2x + 3x^2
        let x: Vec<f64> = (0..5).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|&v| 1.0 + 2.0 * v + 3.0 * v * v).collect();
        let coeffs = PolynomialRegression::fit(&x, &y, 2).unwrap();
        assert_relative_eq!(coeffs[0], 1.0, epsilon = 1e-6);
        assert_relative_eq!(coeffs[1], 2.0, epsilon = 1e-6);
        assert_relative_eq!(coeffs[2], 3.0, epsilon = 1e-6);
    }
}
