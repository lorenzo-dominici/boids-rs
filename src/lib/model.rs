pub mod aos;
pub mod soa;

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
}

type BoidRef<'a> = (&'a Arc<Flock>, &'a f32, &'a Bias, &'a Vec3A, &'a Vec3A);
type BoidRefMut<'a> = (&'a mut Arc<Flock>, &'a mut f32, &'a mut Bias, &'a mut Vec3A, &'a mut Vec3A);

pub trait BoidCollection: Send + Sync {
    fn iter(&self) -> impl Iterator<Item = BoidRef>;
    fn iter_mut(&mut self) -> impl Iterator<Item = BoidRefMut> + '_;
    fn par_iter(&self) -> impl ParallelIterator<Item = BoidRef>;
    fn par_iter_mut(&mut self) -> impl ParallelIterator<Item = BoidRefMut> + '_;

    fn is_in_sight(self_flock: &Arc<Flock>, self_status: f32, self_pos: Vec3A, other_pos: Vec3A) -> bool {
        self_flock.kind.vision.scale(self_status).contains(self_pos.distance(other_pos))
    }

    fn is_too_close(self_flock: &Arc<Flock>, self_status: f32, self_pos: Vec3A, other_pos: Vec3A) -> bool {
        self_flock.kind.protected.scale(self_status).contains(self_pos.distance(other_pos))
    }

    fn is_at_safe_distance(self_flock: &Arc<Flock>, self_status: f32, self_pos: Vec3A, other_pos: Vec3A) -> bool {
        Self::is_in_sight(self_flock, self_status, self_pos, other_pos) && !Self::is_too_close(self_flock, self_status, self_pos, other_pos)
    }

    fn update<E: Environment>(&mut self, boids: &Self, environment: &E) {
        self.iter_mut().for_each(|(s_flock, s_state, s_bias, s_pos, s_vel)| {

            let (sep_sum, sep_count, align_sum, align_count, coh_sum, coh_count) = boids.iter().filter(|(_, _, _, o_pos, _)| s_pos != *o_pos)
            .map(|(o_flock, _, _, o_pos, o_vel)| {
                let role = s_flock.kind.compare(&o_flock.kind);
                let sep = if Self::is_too_close(s_flock, *s_state, *s_pos, *o_pos) {
                    match role {
                        Role::Peer | Role::Predator => (*s_pos - o_pos, 1),
                        Role::Prey | Role::Rival => (o_pos - *s_pos, 1),
                    }
                } else {
                    (Vec3A::ZERO, 0)
                };

                let align = if Self::is_at_safe_distance(s_flock, *s_state, *s_pos, *o_pos) {
                    match role {
                        Role::Peer => if s_flock.name == o_flock.name { (*o_vel, 1) } else { (Vec3A::ZERO, 0) },
                        Role::Prey => (*o_vel, 1),
                        Role::Predator | Role::Rival => (Vec3A::ZERO, 0),
                    }
                } else {
                    (Vec3A::ZERO, 0)
                };

                let coh = if Self::is_at_safe_distance(s_flock, *s_state, *s_pos, *o_pos) {
                    match role {
                        Role::Peer => if s_flock.name == o_flock.name { (*o_pos, 1) } else { (Vec3A::ZERO, 0) },
                        Role::Prey => (*o_pos, 1),
                        Role::Predator | Role::Rival => (-*o_pos, 1),
                    }
                } else {
                    (Vec3A::ZERO, 0)
                };

                (sep.0, sep.1, align.0, align.1, coh.0, coh.1)
            })
            .fold((Vec3A::ZERO, 0, Vec3A::ZERO, 0, Vec3A::ZERO, 0), |(sep_sum, sep_count, align_sum, align_count, coh_sum, coh_count), (sep, sep_c, align, align_c, coh, coh_c)| {
                (sep_sum + sep, sep_count + sep_c, align_sum + align, align_count + align_c, coh_sum + coh, coh_count + coh_c)
            });
            
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

            let behavior = separation * s_flock.kind.sep_weight + (alignment - *s_vel) * s_flock.kind.align_weight + (cohesion - *s_pos) * s_flock.kind.coh_weight;

            let vel = s_vel.clone();

            *s_vel += behavior + s_bias.get(*s_pos) + environment.repulsion((s_flock, s_state, s_bias, s_pos, s_vel));

            *s_vel = (s_flock.kind.acceleration.scale(*s_state).clamp((*s_vel - vel).length()) + vel.length()) * s_vel.normalize();

            let angle = vel.angle_between(*s_vel) / std::f32::consts::PI;

            *s_vel = rotate_towards(*s_vel, vel, s_flock.kind.angular_speed.scale(*s_state).clamp(angle));

            *s_vel = s_flock.kind.speed.scale(*s_state).clamp(s_vel.length()) * s_vel.normalize();

            *s_pos += *s_vel + s_flock.kind.momentum * (vel - *s_vel);

            environment.correction((s_flock, s_state, s_bias, s_pos, s_vel));  
        });
    }

    fn par_update<E: Environment>(&mut self, boids: &Self, environment: &E) {
        self.par_iter_mut().for_each(|(s_flock, s_state, s_bias, s_pos, s_vel)| {

            let (sep_sum, sep_count, align_sum, align_count, coh_sum, coh_count) = boids.par_iter().filter(|(_, _, _, o_pos, _)| s_pos != *o_pos)
            .map(|(o_flock, _, _, o_pos, o_vel)| {
                let role = s_flock.kind.compare(&o_flock.kind);
                let sep = if Self::is_too_close(s_flock, *s_state, *s_pos, *o_pos) {
                    match role {
                        Role::Peer | Role::Predator => (*s_pos - o_pos, 1),
                        Role::Prey | Role::Rival => (o_pos - *s_pos, 1),
                    }
                } else {
                    (Vec3A::ZERO, 0)
                };

                let align = if Self::is_at_safe_distance(s_flock, *s_state, *s_pos, *o_pos) {
                    match role {
                        Role::Peer => if s_flock.name == o_flock.name { (*o_vel, 1) } else { (Vec3A::ZERO, 0) },
                        Role::Prey => (*o_vel, 1),
                        Role::Predator | Role::Rival => (Vec3A::ZERO, 0),
                    }
                } else {
                    (Vec3A::ZERO, 0)
                };

                let coh = if Self::is_at_safe_distance(s_flock, *s_state, *s_pos, *o_pos) {
                    match role {
                        Role::Peer => if s_flock.name == o_flock.name { (*o_pos, 1) } else { (Vec3A::ZERO, 0) },
                        Role::Prey => (*o_pos, 1),
                        Role::Predator | Role::Rival => (-*o_pos, 1),
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

            let behavior = separation * s_flock.kind.sep_weight + (alignment - *s_vel) * s_flock.kind.align_weight + (cohesion - *s_pos) * s_flock.kind.coh_weight;

            let vel = s_vel.clone();

            *s_vel += behavior + s_bias.get(*s_pos) + environment.repulsion((s_flock, s_state, s_bias, s_pos, s_vel));

            *s_vel = (s_flock.kind.acceleration.scale(*s_state).clamp((*s_vel - vel).length()) + vel.length()) * s_vel.normalize();

            let angle = vel.angle_between(*s_vel) / std::f32::consts::PI;

            *s_vel = rotate_towards(*s_vel, vel, s_flock.kind.angular_speed.scale(*s_state).clamp(angle));

            *s_vel = s_flock.kind.speed.scale(*s_state).clamp(s_vel.length()) * s_vel.normalize();

            *s_pos += *s_vel + s_flock.kind.momentum * (vel - *s_vel);

            environment.correction((s_flock, s_state, s_bias, s_pos, s_vel));  
        });
    }
}

fn rotate_towards(a: Vec3A, b: Vec3A, angle: f32) -> Vec3A {
    let current_angle = a.angle_between(b) / std::f32::consts::PI;

    if current_angle <= angle {
        return a;
    }

    let t = angle / current_angle;
    a.normalize().lerp(b.normalize(), 1.0 - t).normalize() * a.length()
}

pub trait Environment: Send + Sync {
    fn repulsion(&self, boid: BoidRef) -> Vec3A;
    fn correction(&self, boid: BoidRefMut);
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
    fn repulsion(&self, boid: BoidRef) -> Vec3A {
        let (flock, state, _, pos, _) = boid;
        let perspective = (pos.length() + flock.kind.vision.scale(*state).max - self.size) / (flock.kind.vision.scale(*state).max / flock.kind.protected.scale(*state).max);
        if perspective > 0.0 {
            -pos.normalize() * perspective * self.turnback
        } else {
            Vec3A::ZERO
        }
    }

    fn correction(&self, boid: BoidRefMut) {
        let (flock, _, _, pos, _) = boid;
        if pos.length() + flock.kind.size/2.0 > self.size {
            *pos = pos.normalize() * (self.size - flock.kind.size/2.0)
        }
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