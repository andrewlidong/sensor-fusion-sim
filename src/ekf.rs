use nalgebra::{Matrix4, Vector2, Vector4};

/// Represents the state of the Extended Kalman Filter
/// [px, py, vx, vy]
pub struct Ekf {
    state: Vector4<f64>,
    covariance: Matrix4<f64>,
    dt: f64,
}

impl Ekf {
    /// Creates a new EKF with initial state and covariance
    pub fn new(initial_pos: Vector2<f64>, dt: f64) -> Self {
        let state = Vector4::new(initial_pos.x, initial_pos.y, 0.0, 0.0);
        let covariance = Matrix4::identity() * 1.0; // Initial uncertainty

        Ekf {
            state,
            covariance,
            dt,
        }
    }

    /// Predicts the next state using IMU measurements
    pub fn predict(&mut self, imu: &super::imu_sim::ImuReading) {
        // State transition matrix
        let f = Matrix4::new(
            1.0, 0.0, self.dt, 0.0, 0.0, 1.0, 0.0, self.dt, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        );

        // Process noise covariance
        let q = Matrix4::identity() * 0.1;

        // Update state
        self.state[2] += imu.accel.x * self.dt; // vx += ax * dt
        self.state[3] += imu.accel.y * self.dt; // vy += ay * dt
        self.state[0] += self.state[2] * self.dt; // px += vx * dt
        self.state[1] += self.state[3] * self.dt; // py += vy * dt

        // Update covariance
        self.covariance = f * self.covariance * f.transpose() + q;
    }

    /// Updates the state using GPS measurements
    pub fn update(&mut self, gps: Vector2<f64>) {
        // Measurement matrix (we only measure position)
        let h = Matrix4::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        );

        // Measurement noise covariance
        let r = Matrix4::identity() * 1.0;

        // Kalman gain
        let k = self.covariance
            * h.transpose()
            * (h * self.covariance * h.transpose() + r)
                .try_inverse()
                .unwrap();

        // Innovation
        let innovation = Vector4::new(gps.x - self.state[0], gps.y - self.state[1], 0.0, 0.0);

        // Update state and covariance
        self.state += k * innovation;
        self.covariance = (Matrix4::identity() - k * h) * self.covariance;
    }

    /// Returns the current state
    pub fn state(&self) -> Vector4<f64> {
        self.state
    }

    /// Returns the current covariance matrix
    pub fn covariance(&self) -> Matrix4<f64> {
        self.covariance
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::imu_sim::ImuReading;

    #[test]
    fn test_ekf() {
        let initial_pos = Vector2::new(0.0, 0.0);
        let mut ekf = Ekf::new(initial_pos, 0.01);

        // Test prediction
        let imu = ImuReading {
            accel: Vector2::new(1.0, 1.0),
        };
        ekf.predict(&imu);
        assert!(ekf.state()[0] > 0.0);

        // Test update
        let gps = Vector2::new(1.0, 1.0);
        ekf.update(gps);
        assert!(ekf.state()[0] > 0.0);
    }
}
