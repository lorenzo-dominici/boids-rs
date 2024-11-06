use std::sync::Arc;
use rand;

#[derive(Debug, Clone)]
pub struct Vector {
    pub x: f32,
    pub y: f32
}

impl Vector {
    pub fn new(x: f32, y: f32) -> Vector {
        Vector { x, y }
    }

    pub fn zero() -> Vector {
        Vector { x: 0.0, y: 0.0 }
    }

    pub fn add(&self, other: &Vector) -> Vector {
        Vector {
            x: self.x + other.x,
            y: self.y + other.y
        }
    }

    pub fn sub(&self, other: &Vector) -> Vector {
        Vector {
            x: self.x - other.x,
            y: self.y - other.y
        }
    }

    pub fn mul(&self, scalar: f32) -> Vector {
        Vector {
            x: self.x * scalar,
            y: self.y * scalar
        }
    }

    pub fn div(&self, scalar: f32) -> Vector {
        Vector {
            x: self.x / scalar,
            y: self.y / scalar
        }
    }

    pub fn dot(&self, other: &Vector) -> f32 {
        self.x * other.x + self.y * other.y
    }
    
    pub fn sq_norm(&self) -> f32 {
        self.dot(self)
    }

    pub fn norm(&self) -> f32 {
        self.sq_norm().sqrt()
    }

    pub fn normalize(&self) -> Vector {
        let norm = self.norm();
        Vector {
            x: self.x / norm,
            y: self.y / norm
        }
    }

    pub fn distance(&self, other: &Vector) -> f32 {
        self.sub(other).norm()
    }

    pub fn angle(&self, other: &Vector) -> f32 {
        let dot = self.dot(other);
        let det = self.x * other.y - self.y * other.x;
        det.atan2(dot)
    }

}

#[derive(Debug, Clone)]
pub enum Role {
    Leader,
    Follower
}

#[derive(Debug, Clone)]
pub enum Kind {
    Predator,
    Prey
}

#[derive(Debug, Clone)]
pub enum Bias {
    Absolute(Vector),
    Relative(Vector)
}

#[derive(Debug, Clone)]
pub struct Flock {
    pub min_vel: f32,
    pub avg_vel: f32,
    pub max_vel: f32,
    pub min_acc: f32,
    pub max_acc: f32,
    pub area_center: Vector,
    pub area_radius: f32,
    pub sep_w: f32,
    pub ali_w: f32,
    pub coh_w: f32,
    pub kind: Kind
}

impl Flock {
    pub fn new(min_vel: f32, avg_vel: f32, max_vel: f32, min_acc: f32, max_acc: f32, area_center: Vector, area_radius: f32, sep_w: f32, ali_w: f32, coh_w: f32, kind: Kind) -> Flock {
        Flock { min_vel, avg_vel, max_vel, min_acc, max_acc, area_center, area_radius, sep_w, ali_w, coh_w, kind }
    }

    pub fn populate(self: Arc<Self>, size: usize, leader_prob: f32, center_prob: f32, ali_prob: f32) -> Vec<Boid> {
        let mut boids = Vec::with_capacity(size);
        for _ in 0..size {
            let pos = Vector::new(
                self.area_center.x + (rand::random::<f32>() - 0.5) * self.area_radius * 2.0,
                self.area_center.y + (rand::random::<f32>() - 0.5) * self.area_radius * 2.0
            );
            let vel = Vector::new(
                self.avg_vel * (rand::random::<f32>() - 0.5) * 2.0 * 2.0_f32.sqrt(),
                self.avg_vel * (rand::random::<f32>() - 0.5) * 2.0 * 2.0_f32.sqrt()
            );
            let pos_bias;
            let pos_bias_w;
            let role;
            if rand::random::<f32>() < leader_prob {
                role = Role::Leader;
                pos_bias = Bias::Absolute(if rand::random::<f32>() < center_prob {
                    self.area_center.clone()
                } else {
                    Vector::new(
                        self.area_center.x + (rand::random::<f32>() - 0.5) * self.area_radius * 2.0,
                        self.area_center.y + (rand::random::<f32>() - 0.5) * self.area_radius * 2.0
                    )
                });
                pos_bias_w = rand::random::<f32>() * 0.5 + 0.5;
            } else {
                role = Role::Follower;
                pos_bias = Bias::Relative(if rand::random::<f32>() < ali_prob {
                    Vector::zero()
                } else {
                    Vector::new(
                        rand::random::<f32>(),
                        rand::random::<f32>()
                    ).normalize()
                });
                pos_bias_w = rand::random::<f32>() * 0.5;
            }
            let status = 1.0 - rand::random::<f32>() * 0.3;
            boids.push(Boid::new(pos, vel, pos_bias, pos_bias_w, status, self.clone(), role.clone()));
        }
        boids
    }
}

#[derive(Debug, Clone)]
pub struct Boid {
    pub pos: Vector,
    pub vel: Vector,
    pub pos_bias: Bias,
    pub pos_bias_w: f32,
    pub status: f32,
    pub flock: Arc<Flock>,
    pub role: Role
}

impl Boid {
    pub fn new(pos: Vector, vel: Vector, pos_bias: Bias, pos_bias_w: f32, status: f32, flock: Arc<Flock>, role: Role) -> Boid {
        Boid { pos, vel, pos_bias, pos_bias_w, status, flock, role}
    }
}