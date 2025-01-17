use rayon::prelude::*;

use serde::{Deserialize, Serialize};
use glam::Vec3A;

#[derive(Clone, Debug)]
pub struct Boid<'a> {
    pub flock: &'a Flock,
    pub state: f32,
    pub bias: Bias,
    pub pos: Vec3A,
    pub vel: Vec3A,
}

impl Boid<'_> {
    pub fn new(flock: &Flock, state: f32, bias: Bias, pos: Vec3A, vel: Vec3A) -> Boid {
        Boid {
            flock,
            state,
            bias,
            pos,
            vel,
        }
    }

    pub fn is_in_sight(&self, boid: &Boid) -> bool {
        self.flock.kind.vision.scale(self.state).contains((self.pos - boid.pos).length())
    }

    pub fn is_too_close(&self, boid: &Boid) -> bool {
        self.flock.kind.protected.scale(self.state).contains((self.pos - boid.pos).length())
    }

    pub fn update<F: Fn(Vec3A) -> f32>(&mut self, boids: &Vec<Boid>, env: F) {
        let (sep_sum, sep_count, align_sum, align_count, coh_sum, coh_count) = boids.par_iter()
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

        self.vel += behavior + env(self.pos).abs() * (-self.pos.normalize()) + self.bias.get(self.pos);

        self.vel = self.flock.kind.speed.scale(self.state).clamp(self.vel.length()) * self.vel.normalize();

        self.vel = (1.0 + self.flock.kind.acceleration.scale(self.state).clamp(self.vel.length() / vel.length() - 1.0)) * vel.length() * self.vel.normalize();

        self.vel = self.vel.length() * Boid::rotate_towards(self.vel, vel, self.flock.kind.angular_speed.scale(self.state).clamp((self.vel.normalize().dot(vel.normalize()) - 1.0) / (-2.0)) * std::f32::consts::PI);

        self.pos += self.vel;

        if env(self.pos) < 0.0 {
            self.pos -= -self.pos.normalize() * env(self.pos);
        }

    }

    fn rotate_towards(a: Vec3A, b: Vec3A, angle: f32) -> Vec3A {
        let dot = a.dot(b).clamp(-1.0, 1.0); // Ensure the dot product is within the valid range
        let current_angle = dot.acos(); // Calculate the current angle between a and b

        if current_angle <= angle {
            return b; // If the current angle is already less than or equal to the target angle, return b
        }

        let t = angle / current_angle; // Calculate the interpolation factor
        a.lerp(b, t).normalize() // Perform the spherical linear interpolation and normalize the result
    }
}

#[derive(Clone, Debug)]
pub struct Bias {
    pub weight: f32,
    pub pos: Vec3A,
}

impl Bias {
    pub fn new(weight: f32, pos: Vec3A) -> Bias {
        Bias { weight, pos }
    }

    pub fn get(&self, pos: Vec3A) -> Vec3A {
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

pub fn sphere_env(pos: Vec3A, env_size: f32) -> f32 {
    if pos.length() > env_size * 0.9 {
        if pos.length() > env_size {
            env_size - pos.length()
        } else {
            env_size * 0.1
        }
    } else {
        0.0
    }
}