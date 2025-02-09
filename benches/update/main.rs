use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use boids::{aos::Boids as AosBoids, setup::FlockBuilder, soa::Boids as SoaBoids, BoidCollection, Environment, SphereEnv};
use serde::{Deserialize, Serialize};
use toml;

fn bench_update(c: &mut Criterion) {
    let mut group = c.benchmark_group("update");
    group.sample_size(1000).measurement_time(std::time::Duration::from_secs(60 * 30));

    let (env, boids) = generate_boids();
    
    for (len, soa, aos) in boids {

        group.bench_with_input(BenchmarkId::new("seq-soa-update", len), &(&soa, &env), |b, (soa, env)| {
            let mut boids = (*soa).clone();
            b.iter(|| boids.update(soa, *env));
        });

        group.bench_with_input(BenchmarkId::new("seq-aos-update", len), &(&aos, &env), |b, (aos, env)| {
            let mut boids = (*aos).clone();
            b.iter(|| boids.update(aos, *env));
        });

        group.bench_with_input(BenchmarkId::new("par-soa-update", len), &(&soa, &env), |b, (soa, env)| {
            let mut boids = (*soa).clone();
            b.iter(|| boids.par_update(soa, *env));
        });

        group.bench_with_input(BenchmarkId::new("par-aos-update", len), &(&aos, &env), |b, (aos, env)| {
            let mut boids = (*aos).clone();
            b.iter(|| boids.par_update(aos, *env));
        });
    }

    group.finish();
}

fn generate_boids() -> (impl Environment, impl IntoIterator<Item=(usize, SoaBoids, AosBoids)>) {
    let mut config = Config::load("benches/update/config.toml").unwrap();

    (SphereEnv::new(config.env_size, config.turnback),
    config.dim_multipliers.into_iter().map(move |dim| {
        let mut vec = Vec::with_capacity(config.flocks.iter().map(|flock_builder| flock_builder.boids).sum());

        for flock_builder in config.flocks.iter_mut() {
            let base = flock_builder.boids;
            flock_builder.boids *= dim;
            vec.extend(flock_builder.build(config.env_size));
            flock_builder.boids = base;
        }

        (vec.len(), SoaBoids::new(&vec), AosBoids::new(vec))
    }))
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
    pub dim_multipliers: Vec<usize>,
    pub env_size: f32,
    pub turnback: f32,
    pub flocks: Vec<FlockBuilder>,
}

impl Config {
    pub fn load(path: &str) -> Result<Config, Box<dyn std::error::Error>> {
        let contents = std::fs::read_to_string(path)?;
        let config: Config = toml::from_str(&contents)?;
        Ok(config)
    }
}

criterion_group!(benches, bench_update);
criterion_main!(benches);