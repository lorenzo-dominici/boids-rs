//! # Setup module
//!
//! This module contains the implementation of the setup functions for the boids model.

use super::{Bias as BoidBias, Boid, Flock, Range};
use glam::Vec3A;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// A struct representing the builder for a flock of boids.
///
/// # Fields
///
/// - `flock`: The flock of boids.
/// - `boids`: The number of boids in the flock.
/// - `state`: The range of state values.
/// - `biases`: A vector of `Bias` instances.
#[derive(Debug, Deserialize, Serialize)]
pub struct FlockBuilder {
    pub flock: Flock,
    pub boids: usize,
    pub state: Range,
    pub biases: Vec<Bias>,
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
        let flock = Arc::new(self.flock.clone());
        for _ in 0..self.boids {
            // Generate random position and velocity for each boid
            let length = rand::random::<f32>() * env_size * 0.75;
            let versor = Vec3A::new(
                rand::random::<f32>() - 0.5,
                rand::random::<f32>() - 0.5,
                rand::random::<f32>() - 0.5,
            )
            .normalize();
            let pos = versor * length;
            let versor = Vec3A::new(
                rand::random::<f32>() - 0.5,
                rand::random::<f32>() - 0.5,
                rand::random::<f32>() - 0.5,
            )
            .normalize();
            let vel = versor * self.flock.kind.speed.random();
            let mut boid_bias = BoidBias {
                weight: 0.0,
                pos: Vec3A::new(0.0, 0.0, 0.0),
            };
            for bias in self.biases.iter() {
                // Apply bias with a certain probability
                if rand::random::<f32>() < bias.prob {
                    boid_bias = BoidBias::new(
                        bias.weight_range.random(),
                        Vec3A::new(bias.position.0, bias.position.1, bias.position.2),
                    );
                    break;
                }
            }
            // Create a new boid and add it to the vector
            let boid = Boid::new(flock.clone(), self.state.random(), boid_bias, pos, vel);
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
