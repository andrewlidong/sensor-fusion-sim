use criterion::{black_box, criterion_group, criterion_main, Criterion};
use nalgebra::Vector2;
use sensor_fusion_sim::{ekf::Ekf, imu_sim::ImuReading};

fn ekf_predict_benchmark(c: &mut Criterion) {
    let initial_pos = Vector2::new(0.0, 0.0);
    let mut ekf = Ekf::new(initial_pos, 0.01);
    let imu = ImuReading {
        accel: Vector2::new(1.0, 1.0),
    };

    c.bench_function("ekf_predict", |b| {
        b.iter(|| {
            ekf.predict(black_box(&imu));
        })
    });
}

fn ekf_update_benchmark(c: &mut Criterion) {
    let initial_pos = Vector2::new(0.0, 0.0);
    let mut ekf = Ekf::new(initial_pos, 0.01);
    let gps = Vector2::new(1.0, 1.0);

    c.bench_function("ekf_update", |b| {
        b.iter(|| {
            ekf.update(black_box(gps));
        })
    });
}

criterion_group!(benches, ekf_predict_benchmark, ekf_update_benchmark);
criterion_main!(benches);
