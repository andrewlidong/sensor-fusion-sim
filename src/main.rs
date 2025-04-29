mod ekf;
mod gps_sim;
mod ground_truth;
mod imu_sim;
mod plot;

use nalgebra::Vector2;
use std::time::{Duration, Instant};

/// Main entry point for the sensor fusion simulation.
///
/// This program:
/// 1. Generates a smooth 2D trajectory
/// 2. Simulates IMU and GPS measurements
/// 3. Runs an Extended Kalman Filter to fuse the measurements
/// 4. Generates visualization plots
///
/// # Simulation Parameters
/// - IMU update rate: 100 Hz
/// - GPS update rate: 1 Hz
/// - Duration: 10 seconds
///
/// # Output Files
/// - `fusion.png`: Main trajectory visualization
/// - `error_metrics.png`: Error metrics over time
/// - `covariance.png`: Covariance evolution
///
/// # Returns
/// Result indicating success or failure
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Simulation parameters
    let dt = 0.01; // 100 Hz
    let duration = 10.0; // 10 seconds
    let gps_rate = 1.0; // 1 Hz

    // Initialize EKF
    let initial_pos = ground_truth::trajectory(0.0);
    let mut ekf = ekf::Ekf::new(initial_pos, dt);

    // Storage for plotting
    let mut records = Vec::new();
    let mut last_gps_time = 0.0;

    // Simulation loop
    let start_time = Instant::now();
    let mut prev_pos = initial_pos;
    let mut prev_time = 0.0;

    while start_time.elapsed().as_secs_f64() < duration {
        let t = start_time.elapsed().as_secs_f64();

        // Get true position and velocity
        let curr_pos = ground_truth::trajectory(t);
        let curr_vel = (curr_pos - prev_pos) / (t - prev_time);

        // Sample IMU and predict
        let imu = imu_sim::sample_imu(prev_pos, curr_pos, dt);
        ekf.predict(&imu);

        // Sample GPS and update at 1 Hz
        if t - last_gps_time >= 1.0 / gps_rate {
            let gps = gps_sim::sample_gps(curr_pos);
            ekf.update(gps);

            // Record state for plotting
            let state = ekf.state();
            records.push(plot::Record {
                time: t,
                truth: curr_pos,
                gps,
                fused: Vector2::new(state[0], state[1]),
                fused_vel: Vector2::new(state[2], state[3]),
                truth_vel: curr_vel,
                covariance: ekf.covariance(),
            });

            last_gps_time = t;
        }

        prev_pos = curr_pos;
        prev_time = t;

        // Sleep to maintain real-time simulation
        std::thread::sleep(Duration::from_secs_f64(dt));
    }

    // Plot all results
    plot::render_all(&records)?;

    Ok(())
}
