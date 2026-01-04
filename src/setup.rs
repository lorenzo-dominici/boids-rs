//! # Setup module
//!
//! This module contains the implementation of the setup functions for the boids model.

use super::{Bias as BoidBias, Boid, Flock, Range};
use glam::Vec3A;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// A struct representing a spawn point for boids.
///
/// # Fields
///
/// - `position`: The 3D position of the spawn point center.
/// - `radius`: The radius within which boids will be spawned.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct SpawnPoint {
    pub position: (f32, f32, f32),
    pub radius: f32,
}

/// A struct representing the builder for a flock of boids.
///
/// # Fields
///
/// - `flock`: The flock of boids.
/// - `boids`: The number of boids in the flock.
/// - `state`: The range of state values.
/// - `biases`: A vector of `Bias` instances.
/// - `spawns`: A vector of `SpawnPoint` instances defining where boids spawn.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct FlockBuilder {
    pub flock: Flock,
    pub boids: usize,
    pub state: Range,
    pub biases: Vec<Bias>,
    pub spawns: Vec<SpawnPoint>,
}

impl FlockBuilder {
    /// Build the boids for the flock using the configured spawn points.
    ///
    /// Boids are distributed evenly across all spawn points.
    /// Each boid is placed at a random position within the sphere defined
    /// by the spawn point's position and radius.
    ///
    /// # Arguments
    ///
    /// - `scale`: Scale factor to apply to spawn positions, spawn radii, and bias positions.
    ///
    /// # Returns
    ///
    /// A vector of `Boid` instances.
    pub fn build(&self, scale: f32) -> Vec<Boid> {
        if self.spawns.is_empty() {
            return Vec::new();
        }

        let flock = Arc::new(self.flock.clone());
        let mut boids = Vec::with_capacity(self.boids);
        
        // Distribute boids across spawn points
        let boids_per_spawn = self.boids / self.spawns.len();
        let remainder = self.boids % self.spawns.len();
        
        for (i, spawn) in self.spawns.iter().enumerate() {
            // Give extra boids to the first spawn points if there's a remainder
            let count = boids_per_spawn + if i < remainder { 1 } else { 0 };
            
            // Scale spawn position and radius by environment scale
            let scaled_spawn_pos = Vec3A::new(
                spawn.position.0 * scale,
                spawn.position.1 * scale,
                spawn.position.2 * scale,
            );
            let scaled_radius = spawn.radius * scale;
            
            for _ in 0..count {
                let pos = random_point_in_sphere(scaled_spawn_pos, scaled_radius);
                
                let versor = Vec3A::new(
                    rand::random::<f32>() - 0.5,
                    rand::random::<f32>() - 0.5,
                    rand::random::<f32>() - 0.5,
                )
                .normalize();
                let vel = versor * self.flock.kind.speed.random();
                
                let mut boid_bias = BoidBias {
                    weight: 0.0,
                    pos: Vec3A::ZERO,
                };
                for bias in self.biases.iter() {
                    if rand::random::<f32>() < bias.prob {
                        // Scale bias position by environment scale
                        boid_bias = BoidBias::new(
                            bias.weight_range.random(),
                            Vec3A::new(
                                bias.position.0 * scale,
                                bias.position.1 * scale,
                                bias.position.2 * scale,
                            ),
                        );
                        break;
                    }
                }
                
                boids.push(Boid::new(flock.clone(), self.state.random(), boid_bias, pos, vel));
            }
        }
        
        boids
    }
}

/// Generate a random point uniformly distributed inside a sphere.
///
/// # Arguments
///
/// - `center`: The center of the sphere.
/// - `radius`: The radius of the sphere.
///
/// # Returns
///
/// A random point within the sphere.
fn random_point_in_sphere(center: Vec3A, radius: f32) -> Vec3A {
    // Use rejection sampling for uniform distribution in sphere
    loop {
        let x = rand::random::<f32>() * 2.0 - 1.0;
        let y = rand::random::<f32>() * 2.0 - 1.0;
        let z = rand::random::<f32>() * 2.0 - 1.0;
        
        let len_sq = x * x + y * y + z * z;
        if len_sq <= 1.0 {
            return center + Vec3A::new(x, y, z) * radius;
        }
    }
}

/// A struct representing a bias that can be applied to a boid.
///
/// # Fields
///
/// - `position`: The position of the bias.
/// - `weight_range`: The range of weight values.
/// - `prob`: The probability of applying the bias.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Bias {
    pub position: (f32, f32, f32),
    pub weight_range: Range,
    pub prob: f32,
}
