use nalgebra::{Matrix4, Vector2};
use plotters::prelude::*;

/// Represents a recorded state at a specific time.
///
/// # Fields
/// * `time` - Time in seconds
/// * `truth` - Ground truth position in meters
/// * `gps` - GPS measurement in meters
/// * `fused` - Fused position estimate in meters
/// * `fused_vel` - Fused velocity estimate in m/s
/// * `truth_vel` - Ground truth velocity in m/s
/// * `covariance` - State covariance matrix
pub struct Record {
    pub time: f64,
    pub truth: Vector2<f64>,
    pub gps: Vector2<f64>,
    pub fused: Vector2<f64>,
    pub fused_vel: Vector2<f64>,
    pub truth_vel: Vector2<f64>,
    pub covariance: Matrix4<f64>,
}

/// Renders the trajectories to a PNG file.
///
/// This function creates a plot showing:
/// - Ground truth trajectory (red line)
/// - GPS measurements (blue dots)
/// - Fused estimate (green line)
///
/// # Arguments
/// * `filename` - Path to save the PNG file
/// * `records` - Vector of recorded states
///
/// # Returns
/// Result indicating success or failure
pub fn render_trajectories(
    filename: &str,
    records: &[Record],
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new(filename, (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .caption("Sensor Fusion Trajectories", ("sans-serif", 30))
        .margin(10)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(-15.0..15.0, -15.0..15.0)?;

    chart
        .configure_mesh()
        .x_desc("X Position (m)")
        .y_desc("Y Position (m)")
        .draw()?;

    // Plot truth trajectory
    chart
        .draw_series(LineSeries::new(
            records.iter().map(|r| (r.truth.x, r.truth.y)),
            &RED,
        ))?
        .label("Ground Truth")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    // Plot GPS measurements
    chart
        .draw_series(PointSeries::of_element(
            records.iter().map(|r| (r.gps.x, r.gps.y)),
            3,
            &BLUE,
            &|c, s, st| EmptyElement::at(c) + Circle::new((0, 0), s, st.filled()),
        ))?
        .label("GPS Measurements")
        .legend(|(x, y)| Circle::new((x + 10, y), 3, &BLUE));

    // Plot fused trajectory
    chart
        .draw_series(LineSeries::new(
            records.iter().map(|r| (r.fused.x, r.fused.y)),
            &GREEN,
        ))?
        .label("Fused Estimate")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &GREEN));

    // Add legend
    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    Ok(())
}

/// Renders error metrics over time to a PNG file.
///
/// This function creates a plot showing:
/// - Position error (red line)
/// - Velocity error (blue line)
///
/// # Arguments
/// * `filename` - Path to save the PNG file
/// * `records` - Vector of recorded states
///
/// # Returns
/// Result indicating success or failure
pub fn render_error_metrics(
    filename: &str,
    records: &[Record],
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new(filename, (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .caption("Error Metrics Over Time", ("sans-serif", 30))
        .margin(10)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(0.0..records.last().unwrap().time, 0.0..5.0)?;

    chart
        .configure_mesh()
        .x_desc("Time (s)")
        .y_desc("Error (m or m/s)")
        .draw()?;

    // Plot position error
    chart
        .draw_series(LineSeries::new(
            records.iter().map(|r| (r.time, (r.fused - r.truth).norm())),
            &RED,
        ))?
        .label("Position Error")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    // Plot velocity error
    chart
        .draw_series(LineSeries::new(
            records
                .iter()
                .map(|r| (r.time, (r.fused_vel - r.truth_vel).norm())),
            &BLUE,
        ))?
        .label("Velocity Error")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    // Add legend
    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    Ok(())
}

/// Renders covariance evolution to a PNG file.
///
/// This function creates a plot showing:
/// - X position covariance (red line)
/// - Y position covariance (blue line)
///
/// # Arguments
/// * `filename` - Path to save the PNG file
/// * `records` - Vector of recorded states
///
/// # Returns
/// Result indicating success or failure
pub fn render_covariance(
    filename: &str,
    records: &[Record],
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new(filename, (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .caption("Covariance Evolution", ("sans-serif", 30))
        .margin(10)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(0.0..records.last().unwrap().time, 0.0..2.0)?;

    chart
        .configure_mesh()
        .x_desc("Time (s)")
        .y_desc("Standard Deviation (m)")
        .draw()?;

    // Plot position covariance (sqrt of diagonal elements)
    chart
        .draw_series(LineSeries::new(
            records
                .iter()
                .map(|r| (r.time, r.covariance[(0, 0)].sqrt())),
            &RED,
        ))?
        .label("X Position")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    chart
        .draw_series(LineSeries::new(
            records
                .iter()
                .map(|r| (r.time, r.covariance[(1, 1)].sqrt())),
            &BLUE,
        ))?
        .label("Y Position")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    // Add legend
    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    Ok(())
}

/// Renders all plots to PNG files.
///
/// This function creates three plots:
/// 1. `fusion.png`: Main trajectory visualization
/// 2. `error_metrics.png`: Error metrics over time
/// 3. `covariance.png`: Covariance evolution
///
/// # Arguments
/// * `records` - Vector of recorded states
///
/// # Returns
/// Result indicating success or failure
pub fn render_all(records: &[Record]) -> Result<(), Box<dyn std::error::Error>> {
    render_trajectories("fusion.png", records)?;
    render_error_metrics("error_metrics.png", records)?;
    render_covariance("covariance.png", records)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Matrix4;

    #[test]
    fn test_render() {
        let records = vec![
            Record {
                time: 0.0,
                truth: Vector2::new(0.0, 0.0),
                gps: Vector2::new(0.1, 0.1),
                fused: Vector2::new(0.05, 0.05),
                fused_vel: Vector2::new(1.0, 1.0),
                truth_vel: Vector2::new(1.0, 1.0),
                covariance: Matrix4::identity(),
            },
            Record {
                time: 1.0,
                truth: Vector2::new(1.0, 1.0),
                gps: Vector2::new(1.1, 1.1),
                fused: Vector2::new(1.05, 1.05),
                fused_vel: Vector2::new(1.0, 1.0),
                truth_vel: Vector2::new(1.0, 1.0),
                covariance: Matrix4::identity(),
            },
        ];

        let result = render_all(&records);
        assert!(result.is_ok());
    }
}
