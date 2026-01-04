//! This module contains the `Context` and `Config` structs that are used to setup the simulation.

use boids::{aos, setup::FlockBuilder, soa, Environment, MeshEnvironment, RepulsionConfig};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// Configuration for mesh-based environment
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct EnvironmentConfig {
    /// Path to OBJ or STL file
    pub path: String,
    /// Whether boids stay inside (true) or outside (false) the mesh
    pub inverted: bool,
    /// Scale factor to apply to the mesh
    pub scale: f32,
    /// Repulsion strength factor
    pub repulsion_strength: f32,
    /// Exponent for distance-based falloff
    pub falloff_exponent: f32,
    /// Exponent for distance-based weighting of multiple triangles
    pub weight_exponent: f32,
    /// Safety margin multiplier for boid size
    pub size_margin: f32,
    /// Maximum number of triangles to consider for repulsion (default: 16)
    pub max_triangles: usize,
    /// Steering strength for turnover effect (default: 0.5)
    pub steering_strength: f32,
}

/// A struct representing the context of the simulation.
///
/// # Fields
///
/// - `soa_seq_prev`: The previous state of boids in SoA format (sequential).
/// - `soa_seq_next`: The next state of boids in SoA format (sequential).
/// - `aos_seq_prev`: The previous state of boids in AoS format (sequential).
/// - `aos_seq_next`: The next state of boids in AoS format (sequential).
/// - `soa_par_prev`: The previous state of boids in SoA format (parallel).
/// - `soa_par_next`: The next state of boids in SoA format (parallel).
/// - `aos_par_prev`: The previous state of boids in AoS format (parallel).
/// - `aos_par_next`: The next state of boids in AoS format (parallel).
/// - `config`: The configuration of the simulation.
/// - `environment`: The environment for the simulation (boxed for dynamic dispatch).
pub struct Context {
    // Sequential versions
    pub soa_seq_prev: soa::Boids,
    pub soa_seq_next: soa::Boids,
    pub aos_seq_prev: aos::Boids,
    pub aos_seq_next: aos::Boids,
    // Parallel versions (separate state)
    pub soa_par_prev: soa::Boids,
    pub soa_par_next: soa::Boids,
    pub aos_par_prev: aos::Boids,
    pub aos_par_next: aos::Boids,
    pub config: Config,
    pub environment: Arc<dyn Environment>,
    /// Mesh environment reference for visualization
    pub mesh_env: Arc<MeshEnvironment>,
}

impl std::fmt::Debug for Context {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Context")
            .field("config", &self.config)
            .field("mesh_env", &self.mesh_env)
            .finish_non_exhaustive()
    }
}

impl Context {
    /// Create a new context for the simulation.
    ///
    /// # Arguments
    ///
    /// - `config`: The configuration of the simulation.
    ///
    /// # Returns
    ///
    /// A new `Context` instance.
    pub fn new(config: Config) -> Self {
        // Create mesh environment
        let repulsion_config = RepulsionConfig {
            repulsion_strength: config.environment.repulsion_strength,
            falloff_exponent: config.environment.falloff_exponent,
            weight_exponent: config.environment.weight_exponent,
            size_margin: config.environment.size_margin,
            max_triangles: config.environment.max_triangles,
            steering_strength: config.environment.steering_strength,
        };

        // Load and scale the mesh
        let mut triangles = boids::load_mesh(&config.environment.path)
            .expect(&format!("Failed to load mesh from: {}", config.environment.path));
        
        // Center and scale the mesh
        boids::center_mesh(&mut triangles);
        if (config.environment.scale - 1.0).abs() > 1e-6 {
            boids::transform_triangles(&mut triangles, config.environment.scale, glam::Vec3A::ZERO);
        }

        let mesh_env = Arc::new(MeshEnvironment::from_triangles(
            triangles,
            repulsion_config,
            config.environment.inverted,
        ));
        
        let environment: Arc<dyn Environment> = mesh_env.clone();

        // Build boids using each flock's spawn points
        // Scale is applied to spawn positions and bias positions
        let scale = config.environment.scale;
        let boids_number: usize = config.flocks.iter().map(|flock| flock.boids).sum();
        let mut vec = Vec::with_capacity(boids_number);
        for flock_builder in config.flocks.iter() {
            let boids = flock_builder.build(scale);
            vec.extend(boids);
        }

        // Create SoA versions (sequential and parallel start with same state)
        let soa_seq_next = soa::Boids::new(&vec);
        let soa_par_next = soa::Boids::new(&vec);

        // Create AoS versions (sequential and parallel start with same state)
        let aos_seq_next = aos::Boids::new(vec.clone());
        let aos_par_next = aos::Boids::new(vec);

        Self {
            // Sequential versions
            soa_seq_prev: soa::Boids::with_capacity(boids_number),
            soa_seq_next,
            aos_seq_prev: aos::Boids::with_capacity(boids_number),
            aos_seq_next,
            // Parallel versions
            soa_par_prev: soa::Boids::with_capacity(boids_number),
            soa_par_next,
            aos_par_prev: aos::Boids::with_capacity(boids_number),
            aos_par_next,
            config,
            environment,
            mesh_env,
        }
    }
}

/// A struct representing the configuration of the simulation.
///
/// # Fields
///
/// - `iters`: The number of iterations to run the simulation.
/// - `env_size`: The size of the environment.
/// - `turnback`: The turnback factor.
/// - `environment`: The environment configuration.
/// - `flocks`: A vector of `FlockBuilder` instances.
/// - `execution`: The execution mode(s) to run.
#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
    pub iters: u32,
    pub environment: EnvironmentConfig,
    pub flocks: Vec<FlockBuilder>,
    #[serde(default)]
    pub execution: ExecutionMode,
}

/// Execution mode for the simulation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum ExecutionMode {
    /// Run all four execution modes
    #[default]
    All,
    /// Sequential Array of Structs
    AosSeq,
    /// Sequential Struct of Arrays
    SoaSeq,
    /// Parallel Array of Structs
    AosPar,
    /// Parallel Struct of Arrays
    SoaPar,
}

impl Config {
    /// Load the configuration from a TOML file.
    ///
    /// # Arguments
    ///
    /// - `path`: A string slice that holds the path to the TOML file.
    ///
    /// # Returns
    ///
    /// A `Result` containing the `Config` instance or an error.
    ///
    /// # Errors
    ///
    /// This function will return an error if the file cannot be read or the TOML data cannot be deserialized.
    pub fn load(path: &str) -> Result<Config, Box<dyn std::error::Error>> {
        let contents = std::fs::read_to_string(path)?;
        let config: Config = toml::from_str(&contents)?;
        Ok(config)
    }
}
