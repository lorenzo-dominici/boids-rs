//! # Struct of Arrays
//! 
//! This module contains the implementation of the Struct of Arrays (SoA) design pattern for the boids model.

use std::sync::Arc;
use glam::Vec3A;
use rayon::prelude::*;

use super::*;

/// A struct representing a collection of boids stored in a Struct of Arrays (SoA) format.
/// 
/// # Fields
/// 
/// - `flock`: A vector of shared references to the flocks.
/// - `state`: A vector of state values.
/// - `bias`: A vector of biases.
/// - `pos`: A vector of positions.
/// - `vel`: A vector of velocities.
/// - `len`: The number of boids in the collection.
#[derive(Debug, Clone)]
pub struct Boids {
    pub flock: Vec<Arc<Flock>>,
    pub state: Vec<f32>,
    pub bias: Vec<Bias>,
    pub pos: Vec<Vec3A>,
    pub vel: Vec<Vec3A>,
    pub len: usize,
}

impl Boids {
    /// Creates a new Boids instance from a vector of Boid.
    /// 
    /// # Arguments
    /// 
    /// - `boids`: A reference to a vector of `Boid`.
    /// 
    /// # Returns
    /// 
    /// A new `Boids` instance.
    pub fn new(boids: &Vec<Boid>) -> Self {
        let len = boids.len();
        
        // Preallocate vectors with the capacity of the number of boids
        let mut flock = Vec::with_capacity(len);
        let mut state = Vec::with_capacity(len);
        let mut bias = Vec::with_capacity(len);
        let mut pos = Vec::with_capacity(len);
        let mut vel = Vec::with_capacity(len);

        // Populate vectors using parallel iteration
        for boid in boids {
            flock.push(boid.flock.clone());
            state.push(boid.state);
            bias.push(boid.bias);
            pos.push(boid.pos);
            vel.push(boid.vel);
        }

        // Return the new Boids instance
        Self { flock, state, bias, pos, vel, len }
    }
}

impl BoidCollection for Boids {
    fn iter(&self) -> impl Iterator<Item = (&Arc<Flock>, &f32, &Bias, &Vec3A, &Vec3A)> {
        self.flock.iter()
            .zip(self.state.iter())
            .zip(self.bias.iter())
            .zip(self.pos.iter())
            .zip(self.vel.iter())
            .map(|((((flock, state), bias), pos), vel)| (
                flock,
                state,
                bias,
                pos,
                vel,
            )
        )
    }

    fn iter_mut(&mut self) -> impl Iterator<Item = (&mut Arc<Flock>, &mut f32, &mut Bias, &mut Vec3A, &mut Vec3A)> + '_ {
        self.flock.iter_mut()
            .zip(self.state.iter_mut())
            .zip(self.bias.iter_mut())
            .zip(self.pos.iter_mut())
            .zip(self.vel.iter_mut())
            .map(|((((flock, state), bias), pos), vel)| (
                flock,
                state,
                bias,
                pos,
                vel,
            )
        )
    }

    fn par_iter(&self) -> impl ParallelIterator<Item = (&Arc<Flock>, &f32, &Bias, &Vec3A, &Vec3A)> {
        (&self.flock, &self.state, &self.bias, &self.pos, &self.vel).into_par_iter()
    }

    fn par_iter_mut(&mut self) -> impl ParallelIterator<Item = (&mut Arc<Flock>, &mut f32, &mut Bias, &mut Vec3A, &mut Vec3A)> + '_ {
        (&mut self.flock, &mut self.state, &mut self.bias, &mut self.pos, &mut self.vel).into_par_iter()
    }
}