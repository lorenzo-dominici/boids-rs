//! # Boids Update Benchmark
//!
//! This benchmark module measures the performance of boid collection updates
//! using the Criterion benchmarking framework.
//!
//! ## Benchmarks
//!
//! The following update implementations are compared:
//!
//! - **Sequential SoA**: Single-threaded Struct of Arrays update
//! - **Sequential AoS**: Single-threaded Array of Structures update
//! - **Parallel SoA**: Multi-threaded Struct of Arrays update (using rayon)
//! - **Parallel AoS**: Multi-threaded Array of Structures update (using rayon)
//!
//! ## Configuration
//!
//! Benchmark parameters are loaded from `benches/update/config.toml`:
//! - `threads`: Thread counts to test (e.g., [1, 2, 4, 8])
//! - `dim_multipliers`: Boid count multipliers for scaling tests
//! - `environment`: Mesh environment configuration
//! - `flocks`: Flock definitions for generating test boids
//!
//! ## Running
//!
//! ```sh
//! cargo bench
//! ```

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use boids::{aos::Boids as AosBoids, setup::FlockBuilder, soa::Boids as SoaBoids, BoidCollection, MeshEnvironment, RepulsionConfig, load_mesh, center_mesh};
use serde::{Deserialize, Serialize};
use toml;
use std::sync::Arc;

/// Benchmark function for measuring boid update performance.
///
/// Tests all four combinations of memory layout (AoS vs SoA) and
/// execution mode (sequential vs parallel) across different boid
/// counts and thread configurations.
fn bench_update(c: &mut Criterion) {
    let mut group = c.benchmark_group("update");
    group.sample_size(100).measurement_time(std::time::Duration::from_secs(60 * 15));

    let (threads, env, boids) = generate_boids();

    for (len, soa, aos) in boids {
        for thread in &threads {
            let params = format!("{}-on-{}", len, thread);

            // Sequential benchmarks (no thread pool needed)
            group.bench_with_input(BenchmarkId::new("seq-soa-update", &params), &(&soa, &env), |b, (soa, env)| {
                let mut boids = (*soa).clone();
                b.iter(|| boids.update(soa, env.as_ref()));
            });

            group.bench_with_input(BenchmarkId::new("seq-aos-update", &params), &(&aos, &env), |b, (aos, env)| {
                let mut boids = (*aos).clone();
                b.iter(|| boids.update(aos, env.as_ref()));
            });

            // Parallel benchmarks using a local thread pool
            let pool = rayon::ThreadPoolBuilder::new().num_threads(*thread).build().unwrap();

            group.bench_with_input(BenchmarkId::new("par-soa-update", &params), &(&soa, &env), |b, (soa, env)| {
                let mut boids = (*soa).clone();
                pool.install(|| {
                    b.iter(|| boids.par_update(soa, env.as_ref()));
                });
            });

            group.bench_with_input(BenchmarkId::new("par-aos-update", &params), &(&aos, &env), |b, (aos, env)| {
                let mut boids = (*aos).clone();
                pool.install(|| {
                    b.iter(|| boids.par_update(aos, env.as_ref()));
                });
            });
        }
    }

    group.finish();
}

/// Generate boid collections for benchmarking.
///
/// Loads configuration from the benchmark config file and creates
/// both SoA and AoS boid collections for each dimension multiplier.
///
/// # Returns
///
/// A tuple containing:
/// - `Vec<usize>`: Thread counts to test
/// - `Arc<MeshEnvironment>`: The mesh environment for collision detection
/// - Iterator of `(count, SoaBoids, AosBoids)`: Boid collections at different scales
fn generate_boids() -> (Vec<usize>, Arc<MeshEnvironment>, impl IntoIterator<Item=(usize, SoaBoids, AosBoids)>) {
    let mut config = Config::load("benches/update/config.toml").unwrap();

    // Load mesh environment
    let mut triangles = load_mesh(&config.environment.path)
        .expect(&format!("Failed to load mesh from: {}", config.environment.path));
    center_mesh(&mut triangles);
    
    let repulsion_config = RepulsionConfig {
        repulsion_strength: config.environment.repulsion_strength,
        falloff_exponent: config.environment.falloff_exponent,
        weight_exponent: config.environment.weight_exponent,
        size_margin: config.environment.size_margin,
        max_triangles: config.environment.max_triangles,
        steering_strength: config.environment.steering_strength,
    };
    
    let mesh_env = Arc::new(MeshEnvironment::from_triangles(
        triangles,
        repulsion_config,
        config.environment.inverted,
    ));

    (
        config.threads,
        mesh_env,
    config.dim_multipliers.into_iter().map(move |dim| {
        let mut vec = Vec::with_capacity(config.flocks.iter().map(|flock_builder| flock_builder.boids).sum());
        let scale = config.environment.scale;

        for flock_builder in config.flocks.iter_mut() {
            let base = flock_builder.boids;
            flock_builder.boids *= dim;
            vec.extend(flock_builder.build(scale));
            flock_builder.boids = base;
        }

        (vec.len(), SoaBoids::new(&vec), AosBoids::new(vec))
    }))
}

/// Configuration for the mesh-based environment in benchmarks.
///
/// Mirrors the environment configuration used in the rerun example,
/// defining how boids interact with the 3D mesh boundaries.
#[derive(Debug, Deserialize, Serialize)]
pub struct EnvironmentConfig {
    /// Path to the mesh file (OBJ or STL)
    pub path: String,
    /// If true, boids stay inside the mesh; if false, they stay outside
    pub inverted: bool,
    /// Scale factor applied to the mesh
    pub scale: f32,
    /// Base strength of the APF repulsion force
    pub repulsion_strength: f32,
    /// Exponent for distance-based falloff (higher = steeper near surface)
    pub falloff_exponent: f32,
    /// Exponent for weighting multiple triangles by distance
    pub weight_exponent: f32,
    /// Safety margin multiplier for boid size during correction
    pub size_margin: f32,
    /// Maximum triangles to consider for repulsion (default: 16)
    #[serde(default = "default_max_triangles")]
    pub max_triangles: usize,
    /// Steering strength for turnover effect (default: 0.5)
    #[serde(default = "default_steering_strength")]
    pub steering_strength: f32,
}

fn default_max_triangles() -> usize {
    16
}

fn default_steering_strength() -> f32 {
    0.5
}

/// Benchmark configuration loaded from TOML.
///
/// Defines all parameters for running the update benchmarks,
/// including thread counts, scaling factors, and flock definitions.
#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
    /// Thread counts to benchmark (e.g., [1, 2, 4, 8])
    pub threads: Vec<usize>,
    /// Multipliers applied to base boid counts for scaling tests
    pub dim_multipliers: Vec<usize>,
    /// Environment size (legacy, kept for compatibility)
    pub env_size: f32,
    /// Turnback factor (legacy, kept for compatibility)
    pub turnback: f32,
    /// Mesh environment configuration
    pub environment: EnvironmentConfig,
    /// Flock builders defining the boid populations
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