//! # Setup module
//! 
//! This module contains the implementation of the setup functions for the boids model.

use std::sync::Arc;
use serde::{Deserialize, Serialize};
use toml;
use glam::Vec3A;
use crate::model::{self, aos, soa, Boid, Flock, Range};

/// A struct representing the configuration of the simulation.
/// 
/// # Fields
/// 
/// - `iters`: The number of iterations to run the simulation.
/// - `layout`: The layout of the environment.
/// - `env_size`: The size of the environment.
/// - `turnback`: The turnback factor.
/// - `flocks`: A vector of `FlockBuilder` instances.
#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
    pub iters: u32,
    pub layout: u32,
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

/// A struct representing the builder for a flock of boids.
/// 
/// # Fields
/// 
/// - `flock`: A shared reference to a `Flock` instance.
/// - `boids`: The number of boids in the flock.
/// - `state`: The range of state values.
/// - `biases`: A vector of `Bias` instances.
#[derive(Debug, Deserialize, Serialize)]
pub struct FlockBuilder {
    pub flock: Arc<Flock>,
    pub boids: usize,
    pub state: Range,
    pub biases: Vec<Bias>
}

impl FlockBuilder {
    /// Build the boids for the flock.
    /// 
    /// # Arguments
    /// 
    /// - `env_size`: The size of the environment.
    /// 
    /// # Returns
    /// 
    /// A vector of `Boid` instances.
    pub fn build(&self, env_size: f32) -> Vec<Boid> {
        let mut boids = Vec::with_capacity(self.boids);
        for _ in 0..self.boids {
            // Generate random position and velocity for each boid
            let length = rand::random::<f32>() * env_size * 0.75;
            let versor = Vec3A::new(rand::random::<f32>() - 0.5, rand::random::<f32>() - 0.5, rand::random::<f32>() - 0.5).normalize();
            let pos = versor * length;
            let versor = Vec3A::new(rand::random::<f32>() - 0.5, rand::random::<f32>() - 0.5, rand::random::<f32>() - 0.5).normalize();
            let vel = versor * self.flock.kind.speed.random();
            let mut boid_bias = model::Bias{weight: 0.0, pos: Vec3A::new(0.0, 0.0, 0.0)};
            for bias in self.biases.iter() {
                // Apply bias with a certain probability
                if rand::random::<f32>() < bias.prob {
                    boid_bias = model::Bias::new(bias.weight_range.random(), Vec3A::new(bias.position.0, bias.position.1, bias.position.2));
                    break;
                }
            }
            // Create a new boid and add it to the vector
            let boid = Boid::new(self.flock.clone(), self.state.random(), boid_bias, pos, vel);
            boids.push(boid);
        }
        boids
    }
}

/// A struct representing a bias that can be applied to a boid.
/// 
/// # Fields
/// 
/// - `position`: The position of the bias.
/// - `weight_range`: The range of weight values.
/// - `prob`: The probability of applying the bias.
#[derive(Debug, Deserialize, Serialize)]
pub struct Bias {
    pub position: (f32, f32, f32),
    pub weight_range: Range,
    pub prob: f32,
}

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
        let mut vec: Vec<Boid> = Vec::with_capacity(boids_number);
        for flock_builder in config.flocks.iter() {
            let mut boids = flock_builder.build(config.env_size);
            vec.append(&mut boids);
        }
        Self { 
            soa_prev: soa::Boids::new(&Vec::new()),
            soa_next: soa::Boids::new(&vec),
            aos_prev: aos::Boids::new(Vec::new()),
            aos_next: aos::Boids::new(vec),
            config,
        }
    }
}