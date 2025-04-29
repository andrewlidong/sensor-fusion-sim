# Sensor Fusion Simulation

A Rust-based simulation of IMU and GPS sensor fusion using an Extended Kalman Filter (EKF). This project demonstrates the principles of sensor fusion in a 2D environment, showing how combining noisy measurements from different sensors can improve position estimation.

## Features

- 2D trajectory generation with smooth motion
- IMU simulation with:
  - Acceleration measurements
  - Angular velocity measurements
  - Gaussian noise
  - Drifting bias
- GPS simulation with:
  - Position measurements
  - Gaussian noise
  - 1Hz update rate
- Extended Kalman Filter implementation
- Real-time visualization of:
  - Ground truth trajectory
  - GPS measurements
  - Fused estimate
  - Error metrics
  - Covariance evolution

## Requirements

- Rust (latest stable version)
- Cargo (comes with Rust)

## Installation

```bash
git clone https://github.com/andrewlidong/sensor_fusion_sim.git
cd sensor_fusion_sim
```

## Usage

Run the simulation:

```bash
cargo run
```

This will:
1. Generate a smooth 2D trajectory
2. Simulate IMU and GPS measurements
3. Run the EKF to fuse the measurements
4. Generate visualization plots in the project directory

## Output Files

- `fusion.png`: Main visualization showing the ground truth, GPS measurements, and fused estimate
- `error_metrics.png`: Plots of position and velocity errors over time
- `covariance.png`: Evolution of the Kalman filter's covariance matrix
- `noise_comparison.png`: Comparison of different noise levels
- `update_rate_comparison.png`: Comparison of different GPS update rates

## Project Structure

```
src/
├── main.rs         # Main simulation loop
├── ground_truth.rs # Trajectory generation
├── imu_sim.rs      # IMU simulation
├── gps_sim.rs      # GPS simulation
├── ekf.rs          # Extended Kalman Filter
└── plot.rs         # Visualization utilities
```

## Dependencies

- `nalgebra`: Linear algebra operations
- `rand`: Random number generation
- `plotters`: Plotting and visualization
- `csv`: Optional data export