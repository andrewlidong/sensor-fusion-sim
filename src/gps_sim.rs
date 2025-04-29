use nalgebra::Vector2;
use rand::Rng;

/// Simulates a GPS reading at a given position
///
/// # Arguments
/// * `true_pos` - The true position to sample around
///
/// # Returns
/// A noisy position measurement
pub fn sample_gps(true_pos: Vector2<f64>) -> Vector2<f64> {
    let mut rng = rand::thread_rng();

    // Add Gaussian noise (standard deviation of 1.0 meters)
    let noise = Vector2::new(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0));

    true_pos + noise
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sample_gps() {
        let true_pos = Vector2::new(10.0, 20.0);
        let gps_pos = sample_gps(true_pos);

        // GPS reading should be within reasonable range of true position
        assert!((gps_pos - true_pos).norm() < 3.0);
    }
}
