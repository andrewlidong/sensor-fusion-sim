//! Sensor Fusion Simulation Library
//!
//! This library provides components for simulating and fusing IMU and GPS measurements
//! using an Extended Kalman Filter. It includes:
//!
//! - Ground truth trajectory generation
//! - IMU simulation with noise and bias
//! - GPS simulation with noise
//! - Extended Kalman Filter implementation
//! - Visualization utilities
//!
//! # Example
//! ```no_run
//! use nalgebra::Vector2;
//! use sensor_fusion_sim::{
//!     ekf::Ekf,
//!     imu_sim::sample_imu,
//!     gps_sim::sample_gps,
//!     ground_truth::trajectory,
//! };
//!
//! // Initialize EKF
//! let dt = 0.01;
//! let initial_pos = trajectory(0.0);
//! let mut ekf = Ekf::new(initial_pos, dt);
//!
//! // Simulate one step
//! let curr_pos = trajectory(dt);
//! let imu = sample_imu(initial_pos, curr_pos, dt);
//! ekf.predict(&imu);
//!
//! let gps = sample_gps(curr_pos);
//! ekf.update(gps);
//! ```

pub mod ekf;
pub mod gps_sim;
pub mod ground_truth;
pub mod imu_sim;
pub mod plot;
