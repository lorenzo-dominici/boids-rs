use std::env;

use log::error;
use serde::{Deserialize, Serialize};
use toml;
use glam::Vec3A;

use crate::model::{self, Boid, Flock, Range};

#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
    env_size: f32,
    flocks: Vec<FlockBuilder>,
}

impl Config {
    pub fn load(path: &str) -> Result<Config, Box<dyn std::error::Error>> {
        let contents = std::fs::read_to_string(path)?;
        let config: Config = toml::from_str(&contents)?;
        Ok(config)
    }
}

#[derive(Debug, Deserialize, Serialize)]
struct FlockBuilder {
    pub flock: Flock,
    pub boids: usize,
    pub state: Range,
    pub biases: Vec<Bias>
}

impl FlockBuilder {
    pub fn build(&self, env_size: f32) -> Vec<Boid> {
        let mut boids = Vec::with_capacity(self.boids);
        let flock = self.flock.clone();
        for _ in 0..self.boids {
            let length = rand::random::<f32>() * env_size * 0.75;
            let versor = Vec3A::new(rand::random::<f32>() - 0.5, rand::random::<f32>() - 0.5, rand::random::<f32>() - 0.5).normalize();
            let pos = versor * length;
            let versor = Vec3A::new(rand::random::<f32>() - 0.5, rand::random::<f32>() - 0.5, rand::random::<f32>() - 0.5).normalize();
            let vel = versor * self.flock.kind.speed.random();
            let mut boid_bias = model::Bias{weight: 0.0, pos: Vec3A::new(0.0, 0.0, 0.0)};
            for bias in self.biases.iter() {
                if rand::random::<f32>() < bias.prob {
                    boid_bias = model::Bias{weight: bias.weight_range.random(), pos: Vec3A::new(bias.position.0, bias.position.1, bias.position.2)};
                    break;
                }
            }
            let boid = Boid::new(&self.flock, self.state.random(), boid_bias, pos, vel);
            boids.push(boid);
        }
        boids
    }
}

#[derive(Debug, Deserialize, Serialize)]
struct Bias {
    pub position: (f32, f32, f32),
    pub weight_range: Range,
    pub prob: f32,
}

#[derive(Debug)]
pub struct Context<'a> {
    pub prev: Vec<Boid<'a>>,
    pub next: Vec<Boid<'a>>,
    pub env_size: f32,
}

impl<'a> Context<'a> {
    pub fn new(config: &'a Config) -> Context {
        let env_size = config.env_size;
        let boids_number: usize = config.flocks.iter().map(|flock| flock.boids).sum();
        let mut vec: Vec<Boid> = Vec::with_capacity(boids_number);
        for flock_builder in config.flocks.iter() {
            let mut boids = flock_builder.build(env_size);
            vec.append(&mut boids);
        }
        Context { prev: vec.clone(), next: vec, env_size }
    }
}