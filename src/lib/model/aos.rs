//! # Array of Structures (AoS) Boid Collection
//! 
//! This module contains the implementation of the `Boids` collection in the Array of Structures (AoS) format.

use std::sync::Arc;
use glam::Vec3A;
use rayon::prelude::*;
use super::*;

/// A collection of boids (bird-oid objects) represented in an array of structures (AoS) format.
/// 
/// # Fields
/// 
/// - `boids`: A vector of `Boid` objects.
#[derive(Debug, Clone)]
pub struct Boids {
    boids: Vec<Boid>,
}

impl Boids {
    /// Creates a new `Boids` collection from a vector of `Boid` objects.
    /// 
    /// # Arguments
    /// 
    /// - `boids`: A vector of `Boid` objects.
    /// 
    /// # Returns
    /// 
    /// A new `Boids` collection.
    pub fn new(boids: Vec<Boid>) -> Self {
        Self { boids }
    }
}

impl BoidCollection for Boids {
    fn iter(&self) -> impl Iterator<Item = (&Arc<Flock>, &f32, &Bias, &Vec3A, &Vec3A)> {
        self.boids.iter().map(|boid| (
            &boid.flock,
            &boid.state,
            &boid.bias,
            &boid.pos,
            &boid.vel,
        ))
    }

    fn iter_mut(&mut self) -> impl Iterator<Item = (&mut Arc<Flock>, &mut f32, &mut Bias, &mut Vec3A, &mut Vec3A)> + '_ {
        self.boids.iter_mut().map(|boid| (
            &mut boid.flock,
            &mut boid.state,
            &mut boid.bias,
            &mut boid.pos,
            &mut boid.vel,
        ))
    }

    fn par_iter(&self) -> impl ParallelIterator<Item = (&Arc<Flock>, &f32, &Bias, &Vec3A, &Vec3A)> {
        self.boids.par_iter().map(|boid| (
            &boid.flock,
            &boid.state,
            &boid.bias,
            &boid.pos,
            &boid.vel,
        ))
    }

    fn par_iter_mut(&mut self) -> impl ParallelIterator<Item = (&mut Arc<Flock>, &mut f32, &mut Bias, &mut Vec3A, &mut Vec3A)> + '_ {
        self.boids.par_iter_mut().map(|boid| (
            &mut boid.flock,
            &mut boid.state,
            &mut boid.bias,
            &mut boid.pos,
            &mut boid.vel,
        ))
    }
}