

use core::panic;

use std::sync::Arc;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use glam::Vec3A;

#[derive(Debug, Clone)]
pub struct Boid {
    pub flock: Arc<Flock>,
    pub state: f32,
    pub bias: Bias,
    pub pos: Vec3A,
    pub vel: Vec3A,
}

impl Boid {
    pub fn new(flock: Arc<Flock>, state: f32, bias: Bias, pos: Vec3A, vel: Vec3A) -> Boid {
        Boid {
            flock,
            state,
            bias,
            pos,
            vel,
        }
    }

    pub fn is_in_sight(&self, boid: &Boid) -> bool {
        self.flock.kind.vision.scale(self.state).contains(self.pos.distance(boid.pos))
    }

    pub fn is_too_close(&self, boid: &Boid) -> bool {
        self.flock.kind.protected.scale(self.state).contains(self.pos.distance(boid.pos))
    }

    pub fn update<'a, T, I>(&mut self, boids: &'a I, environment: &T)
    where 
        T: Environment,
        I: IntoParallelRefIterator<'a, Item = &'a Boid>,
    {
        let (sep_sum, sep_count, align_sum, align_count, coh_sum, coh_count) = boids.par_iter().filter(|boid| boid.pos != self.pos)
            .map(|boid| {
                let role = self.flock.kind.compare(&boid.flock.kind);
                let sep = if self.is_too_close(boid) {
                    match role {
                        Role::Peer | Role::Predator => (self.pos - boid.pos, 1),
                        Role::Prey | Role::Rival => (boid.pos - self.pos, 1),
                    }
                } else {
                    (Vec3A::ZERO, 0)
                };

                let align = if self.is_in_sight(boid) && !self.is_too_close(boid) {
                    match role {
                        Role::Peer => if self.flock.name == boid.flock.name { (boid.vel, 1) } else { (Vec3A::ZERO, 0) },
                        Role::Prey => (boid.vel, 1),
                        Role::Predator | Role::Rival => (Vec3A::ZERO, 0),
                    }
                } else {
                    (Vec3A::ZERO, 0)
                };

                let coh = if self.is_in_sight(boid) && !self.is_too_close(boid) {
                    match role {
                        Role::Peer => if self.flock.name == boid.flock.name { (boid.pos, 1) } else { (Vec3A::ZERO, 0) },
                        Role::Prey => (boid.pos, 1),
                        Role::Predator | Role::Rival => (-boid.pos, 1),
                    }
                } else {
                    (Vec3A::ZERO, 0)
                };

                (sep.0, sep.1, align.0, align.1, coh.0, coh.1)
            })
            .fold(
                || (Vec3A::ZERO, 0, Vec3A::ZERO, 0, Vec3A::ZERO, 0),
                |(sep_sum, sep_count, align_sum, align_count, coh_sum, coh_count), (sep, sep_c, align, align_c, coh, coh_c)| {
                    (
                        sep_sum + sep,
                        sep_count + sep_c,
                        align_sum + align,
                        align_count + align_c,
                        coh_sum + coh,
                        coh_count + coh_c,
                    )
                },
            )
            .reduce(
                || (Vec3A::ZERO, 0, Vec3A::ZERO, 0, Vec3A::ZERO, 0),
                |(sep_sum1, sep_count1, align_sum1, align_count1, coh_sum1, coh_count1), (sep_sum2, sep_count2, align_sum2, align_count2, coh_sum2, coh_count2)| {
                    (
                        sep_sum1 + sep_sum2,
                        sep_count1 + sep_count2,
                        align_sum1 + align_sum2,
                        align_count1 + align_count2,
                        coh_sum1 + coh_sum2,
                        coh_count1 + coh_count2,
                    )
                },
            );

        let separation = if sep_count > 0 {
            sep_sum / sep_count as f32
        } else {
            Vec3A::ZERO
        };

        let alignment = if align_count > 0 {
            align_sum / align_count as f32
        } else {
            Vec3A::ZERO
        };

        let cohesion = if coh_count > 0 {
            coh_sum / coh_count as f32
        } else {
            Vec3A::ZERO
        };

        let behavior = separation * self.flock.kind.sep_weight + (alignment - self.vel) * self.flock.kind.align_weight + (cohesion - self.pos) * self.flock.kind.coh_weight;

        let vel = self.vel.clone();

        self.vel += behavior + self.bias.get(&self.pos) + environment.repulsion(self);
        
        self.vel = (self.flock.kind.acceleration.scale(self.state).clamp((self.vel - vel).length()) + vel.length()) * self.vel.normalize();

        let angle = vel.angle_between(self.vel) / std::f32::consts::PI;

        self.vel = Boid::rotate_towards(self.vel, vel, self.flock.kind.angular_speed.scale(self.state).clamp(angle));

        self.vel = self.flock.kind.speed.scale(self.state).clamp(self.vel.length()) * self.vel.normalize();

        self.pos += self.vel + self.flock.kind.momentum * (vel - self.vel);

        environment.correction(self);

    }

    fn rotate_towards(a: Vec3A, b: Vec3A, angle: f32) -> Vec3A {
        let current_angle = a.angle_between(b) / std::f32::consts::PI;

        if current_angle <= angle {
            return a;
        }

        let t = angle / current_angle;
        a.normalize().lerp(b.normalize(), 1.0 - t).normalize() * a.length()
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Bias {
    pub weight: f32,
    pub pos: Vec3A,
}

impl Bias {
    pub fn new(weight: f32, pos: Vec3A) -> Bias {
        Bias { weight, pos }
    }

    pub fn get(&self, pos: &Vec3A) -> Vec3A {
        self.weight * (self.pos - pos).normalize()
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Flock {
    pub name: String,
    pub kind: Kind,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Kind {
    pub kind: String,
    pub size: f32,
    pub color: (u8, u8, u8),
    pub preys: Vec<String>,
    pub predators: Vec<String>,
    pub speed: Range,
    pub angular_speed: Range,
    pub acceleration: Range,
    pub momentum: f32,
    pub vision: Range,
    pub protected: Range,
    pub sep_weight: f32,
    pub align_weight: f32,
    pub coh_weight: f32,
}

impl Kind {
    pub fn compare(&self, other: &Self) -> Role {
        match (self.preys.contains(&other.kind), self.predators.contains(&other.kind), other.preys.contains(&self.kind), other.predators.contains(&self.kind)) {
            (false, false, false, false) => Role::Peer,
            (true, false, false, true) => Role::Prey,
            (false, true, true, false) => Role::Predator,
            (true, true, true, true) => Role::Rival,
            _ => {panic!("Invalid Comparison")},
        }
    }
}

#[derive(Debug, Deserialize, Serialize)]
pub enum Role {
    Peer,
    Prey,
    Predator,
    Rival
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Range {
    pub min: f32,
    pub max: f32,
}

impl Range {
    pub fn new(min: f32, max: f32) -> Range {
        Range { min, max }
    }

    pub fn contains(&self, value: f32) -> bool {
        value >= self.min && value <= self.max
    }

    pub fn clamp(&self, value: f32) -> f32 {
        if value < self.min {
            self.min
        } else if value > self.max {
            self.max
        } else {
            value
        }
    }

    pub fn scale(&self, value: f32) -> Self{
        Range::new(self.min * value, self.max * value)
    }

    pub fn random(&self) -> f32 {
        rand::random::<f32>() * (self.max - self.min) + self.min
    }
}

pub trait Environment {
    fn repulsion(&self, boid: &Boid) -> Vec3A;
    fn correction(&self, boid: &mut Boid);
}

#[derive(Clone, Debug)]
pub struct SphereEnv {
    pub size: f32,
    pub turnback: f32,
}

impl SphereEnv {
    pub fn new(size: f32, turnback: f32) -> SphereEnv {
        SphereEnv { size, turnback }
    }
}

impl Environment for SphereEnv {
    fn repulsion(&self, boid: &Boid) -> Vec3A {
        let perspective = (boid.pos.length() + boid.flock.kind.vision.scale(boid.state).max - self.size) / (boid.flock.kind.vision.scale(boid.state).max / boid.flock.kind.protected.scale(boid.state).max);
        if perspective > 0.0 {
            -boid.pos.normalize() * perspective * self.turnback
        } else {
            Vec3A::ZERO
        }
    }

    fn correction(&self, boid: &mut Boid) {
        if boid.pos.length() + boid.flock.kind.size/2.0 > self.size {
            boid.pos = boid.pos.normalize() * (self.size - boid.flock.kind.size/2.0)
        }
    }
}