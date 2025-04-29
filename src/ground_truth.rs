use nalgebra::Vector2;

/// Generates a smooth 2D trajectory for simulation
///
/// # Arguments
/// * `t` - Time in seconds
///
/// # Returns
/// A 2D position vector representing the true position at time t
pub fn trajectory(t: f64) -> Vector2<f64> {
    // Create a smooth figure-8 pattern
    let scale = 10.0; // Scale factor for the trajectory
    let omega = 0.5; // Angular frequency (rad/s)

    Vector2::new(scale * (omega * t).sin(), scale * (2.0 * omega * t).sin())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trajectory() {
        let pos = trajectory(0.0);
        assert_eq!(pos.x, 0.0);
        assert_eq!(pos.y, 0.0);

        let pos = trajectory(std::f64::consts::PI / 2.0);
        assert!(pos.x > 0.0);
        assert!(pos.y > 0.0);
    }
}
