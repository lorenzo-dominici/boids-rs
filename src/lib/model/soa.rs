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
#[derive(Debug, Clone)]
pub struct Boids {
    pub flock: Vec<Arc<Flock>>,
    pub state: Vec<f32>,
    pub bias: Vec<Bias>,
    pub pos: Vec<Vec3A>,
    pub vel: Vec<Vec3A>,
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
        boids.par_iter().map(|boid| boid.flock.clone()).collect_into_vec(&mut flock);
        boids.par_iter().map(|boid| boid.state).collect_into_vec(&mut state);
        boids.par_iter().map(|boid| boid.bias).collect_into_vec(&mut bias);
        boids.par_iter().map(|boid| boid.pos).collect_into_vec(&mut pos);
        boids.par_iter().map(|boid| boid.vel).collect_into_vec(&mut vel);

        // Return the new Boids instance
        Self { flock, state, bias, pos, vel }
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
        self.flock.par_iter()
            .zip(self.state.par_iter())
            .zip(self.bias.par_iter())
            .zip(self.pos.par_iter())
            .zip(self.vel.par_iter())
            .map(|((((flock, state), bias), pos), vel)| (
                flock,
                state,
                bias,
                pos,
                vel,
            )
        )
    }

    fn par_iter_mut(&mut self) -> impl ParallelIterator<Item = (&mut Arc<Flock>, &mut f32, &mut Bias, &mut Vec3A, &mut Vec3A)> + '_ {
        self.flock.par_iter_mut()
            .zip(self.state.par_iter_mut())
            .zip(self.bias.par_iter_mut())
            .zip(self.pos.par_iter_mut())
            .zip(self.vel.par_iter_mut())
            .map(|((((flock, state), bias), pos), vel)| (
                flock,
                state,
                bias,
                pos,
                vel,
            ))
    }
}