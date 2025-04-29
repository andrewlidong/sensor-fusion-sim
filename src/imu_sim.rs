use nalgebra::Vector2;
use rand::Rng;

/// Represents an IMU reading with acceleration measurements.
///
/// # Fields
/// * `accel` - 2D acceleration vector in m/s²
pub struct ImuReading {
    pub accel: Vector2<f64>,
}

/// Clamps each component of a Vector2 to the given range.
///
/// # Arguments
/// * `v` - The vector to clamp
/// * `min` - Minimum value for each component
/// * `max` - Maximum value for each component
///
/// # Returns
/// A new vector with each component clamped to the specified range
fn clamp_vector(v: Vector2<f64>, min: f64, max: f64) -> Vector2<f64> {
    Vector2::new(v.x.clamp(min, max), v.y.clamp(min, max))
}

/// Simulates an IMU reading between two positions.
///
/// This function computes the ideal acceleration between two positions and adds:
/// - Gaussian noise with standard deviation of 0.1 m/s²
/// - A slowly drifting bias that follows a random walk
///
/// # Arguments
/// * `prev_pos` - Previous position in meters
/// * `curr_pos` - Current position in meters
/// * `dt` - Time step in seconds
///
/// # Returns
/// An IMU reading with acceleration measurements including noise and bias
///
/// # Example
/// ```
/// use nalgebra::Vector2;
/// use sensor_fusion_sim::imu_sim::sample_imu;
///
/// let prev_pos = Vector2::new(0.0, 0.0);
/// let curr_pos = Vector2::new(1.0, 1.0);
/// let dt = 0.01;
///
/// let reading = sample_imu(prev_pos, curr_pos, dt);
/// assert!(reading.accel.norm() > 0.0);
/// ```
pub fn sample_imu(prev_pos: Vector2<f64>, curr_pos: Vector2<f64>, dt: f64) -> ImuReading {
    let mut rng = rand::thread_rng();

    // Compute true acceleration (assuming constant velocity over dt)
    let vel = (curr_pos - prev_pos) / dt;
    let accel = vel / dt; // This is a simplification

    // Add Gaussian noise (standard deviation of 0.1 m/s²)
    let noise_accel = Vector2::new(rng.gen_range(-0.1..0.1), rng.gen_range(-0.1..0.1));

    // Add slowly drifting bias (random walk)
    static mut BIAS: Vector2<f64> = Vector2::new(0.0, 0.0);
    unsafe {
        BIAS += Vector2::new(rng.gen_range(-0.01..0.01), rng.gen_range(-0.01..0.01));
        BIAS = clamp_vector(BIAS, -0.5, 0.5); // Limit bias magnitude
    }

    ImuReading {
        accel: accel + noise_accel + unsafe { BIAS },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sample_imu() {
        let prev_pos = Vector2::new(0.0, 0.0);
        let curr_pos = Vector2::new(1.0, 1.0);
        let dt = 0.01;

        let reading = sample_imu(prev_pos, curr_pos, dt);
        assert!(reading.accel.norm() > 0.0);
    }

    #[test]
    fn test_clamp_vector() {
        let v = Vector2::new(-2.0, 3.0);
        let clamped = clamp_vector(v, -1.0, 1.0);
        assert_eq!(clamped.x, -1.0);
        assert_eq!(clamped.y, 1.0);
    }
}
