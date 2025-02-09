//! This module contains the `Context` and `Config` structs that are used to setup the simulation.

use boids::{aos, setup::FlockBuilder, soa};
use serde::{Deserialize, Serialize};

/// A struct representing the context of the simulation.
///
/// # Fields
///
/// - `soa_prev`: The previous state of boids in SoA format.
/// - `soa_next`: The next state of boids in SoA format.
/// - `aos_prev`: The previous state of boids in AoS format.
/// - `aos_next`: The next state of boids in AoS format.
/// - `config`: The configuration of the simulation.
#[derive(Debug)]
pub struct Context {
    pub soa_prev: soa::Boids,
    pub soa_next: soa::Boids,
    pub aos_prev: aos::Boids,
    pub aos_next: aos::Boids,
    pub config: Config,
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
        let boids_number: usize = config.flocks.iter().map(|flock| flock.boids).sum();
        let mut vec = Vec::with_capacity(boids_number);
        for flock_builder in config.flocks.iter() {
            let boids = flock_builder.build(config.env_size);
            vec.extend(boids);
        }
        Self {
            soa_prev: soa::Boids::with_capacity(boids_number),
            soa_next: soa::Boids::new(&vec),
            aos_prev: aos::Boids::with_capacity(boids_number),
            aos_next: aos::Boids::new(vec),
            config,
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
/// - `flocks`: A vector of `FlockBuilder` instances.
#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
    pub iters: u32,
    pub env_size: f32,
    pub turnback: f32,
    pub flocks: Vec<FlockBuilder>,
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
