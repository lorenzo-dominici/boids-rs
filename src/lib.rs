//! # Boids Simulation Library
//!
//! This library provides the core functionality for simulating boids in a 3D environment. It includes the following modules:
//!
//! * `aos` - Array of structures implementation
//! * `soa` - Struct of arrays implementation
//! * `setup` - Setup functions for the boids model
//!
//! ## Usage
//!
//! To use the library, add it as a dependency in your `Cargo.toml` file:
//!
//! ```toml
//!
//! [dependencies]
//! boids = { path = "path/to/boids" }
//!
//! ```
//!
//! Then, you can use the library in your code by importing the modules:
//!
//! ```rust
//!
//! use boids::{aos, soa, setup};
//!
//! ```
//!
//! ## License
//!
//! This project is licensed under the MIT License.
//!
//! ## Contributing
//!
//! Contributions are welcome. Feel free to open issues, submit pull requests, or suggest new features.
//!
//! ## Acknowledgements
//!
//! This project was created as midterm project for the course "Parallel Computing" at the University of Studies of Florence.
//!
//! ## References
//!
//! * [Boids algorithm](https://vanhunteradams.com/Pico/Animal_Movement/Boids-algorithm.html)
//! * [rerun.io](https://rerun.io)

pub mod aos;
pub mod setup;
pub mod soa;

use glam::Vec3A;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// Structure representing a single boid in the flock
///
/// # Fields
///
/// * `flock` - A shared reference to the flock to which the boid belongs.
/// * `state` - A floating-point value representing the state of the boid.
/// * `bias` - A `Bias` struct representing the bias affecting the boid's movement.
/// * `pos` - A `Vec3A` vector representing the position of the boid.
/// * `vel` - A `Vec3A` vector representing the velocity of the boid.
#[derive(Debug, Clone)]
pub struct Boid {
    pub flock: Arc<Flock>,
    pub state: f32,
    pub bias: Bias,
    pub pos: Vec3A,
    pub vel: Vec3A,
}

impl Boid {
    /// Constructor to create a new boid
    ///
    /// # Arguments
    ///
    /// * `flock` - A shared reference to the flock to which the boid belongs.
    /// * `state` - A floating-point value representing the state of the boid.
    /// * `bias` - A `Bias` struct representing the bias affecting the boid's movement.
    /// * `pos` - A `Vec3A` vector representing the position of the boid.
    /// * `vel` - A `Vec3A` vector representing the velocity of the boid.
    ///
    /// # Returns
    ///
    /// A `Boid` struct with the given flock, state, bias, position, and velocity.
    pub fn new(flock: Arc<Flock>, state: f32, bias: Bias, pos: Vec3A, vel: Vec3A) -> Self {
        Self {
            flock,
            state,
            bias,
            pos,
            vel,
        }
    }
}

/// Type alias for an immutable reference to a boid's component
pub type BoidRef<'a> = (&'a Arc<Flock>, &'a f32, &'a Bias, &'a Vec3A, &'a Vec3A);

/// Type alias for a mutable reference to a boid's components
pub type BoidRefMut<'a> = (
    &'a mut Arc<Flock>,
    &'a mut f32,
    &'a mut Bias,
    &'a mut Vec3A,
    &'a mut Vec3A,
);

/// Trait for a collection of boids
pub trait BoidCollection: Send + Sync {
    /// Returns an iterator over immutable references to the boid components
    ///
    /// # Returns
    ///
    /// An iterator over immutable references to the boid components
    fn iter(&self) -> impl Iterator<Item = BoidRef>;

    /// Returns an iterator over mutable references to the boid components
    ///
    /// # Returns
    ///
    /// An iterator over mutable references to the boid components
    fn iter_mut(&mut self) -> impl Iterator<Item = BoidRefMut> + '_;

    /// Returns a parallel iterator over immutable references to the boid components
    ///
    /// # Returns
    ///
    /// A parallel iterator over immutable references to the boid components
    fn par_iter(&self) -> impl ParallelIterator<Item = BoidRef>;

    /// Returns a parallel iterator over mutable references to the boid components
    ///
    /// # Returns
    ///
    /// A parallel iterator over mutable references to the boid components
    fn par_iter_mut(&mut self) -> impl ParallelIterator<Item = BoidRefMut> + '_;

    /// Check if another boid is within the vision range
    ///
    /// # Arguments
    ///
    /// * `self_flock` - A reference to the flock to which the boid belongs.
    /// * `self_status` - A floating-point value representing the state of the boid.
    /// * `self_pos` - A `Vec3A` vector representing the position of the boid.
    /// * `other_pos` - A `Vec3A` vector representing the position of the other boid.
    ///
    /// # Returns
    ///
    /// A boolean value indicating whether the other boid is within the vision range.
    fn is_in_sight(
        self_flock: &Arc<Flock>,
        self_status: f32,
        self_pos: Vec3A,
        other_pos: Vec3A,
    ) -> bool {
        self_flock
            .kind
            .vision
            .scale(self_status)
            .contains(self_pos.distance(other_pos))
    }

    /// Check if another boid is within the protected range (too close)
    ///
    /// # Arguments
    ///
    /// * `self_flock` - A reference to the flock to which the boid belongs.
    /// * `self_status` - A floating-point value representing the state of the boid.
    /// * `self_pos` - A `Vec3A` vector representing the position of the boid.
    /// * `other_pos` - A `Vec3A` vector representing the position of the other boid.
    ///
    /// # Returns
    ///
    /// A boolean value indicating whether the other boid is within the protected range.
    fn is_too_close(
        self_flock: &Arc<Flock>,
        self_status: f32,
        self_pos: Vec3A,
        other_pos: Vec3A,
    ) -> bool {
        self_flock
            .kind
            .protected
            .scale(self_status)
            .contains(self_pos.distance(other_pos))
    }

    /// Update the state of the boids based on their interactions and the environment
    ///
    /// # Arguments
    ///
    /// * `boids` - A reference to the boids collection to update from.
    /// * `environment` - A reference to the environment in which the boids are placed.
    ///
    /// # Panics
    ///
    /// Panics if the `Role` comparison is invalid
    fn update<E: Environment>(&mut self, boids: &Self, environment: &E) {
        self.iter_mut()
            .for_each(|(s_flock, s_state, s_bias, s_pos, s_vel)| {
                // Calculate separation, alignment, and cohesion vectors
                let (sep_sum, sep_count, align_sum, align_count, coh_sum, coh_count) = boids
                    .iter()
                    .filter(|(_, _, _, o_pos, _)| {
                        s_pos != *o_pos && Self::is_in_sight(s_flock, *s_state, *s_pos, **o_pos)
                    })
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

                        let align = if !Self::is_too_close(s_flock, *s_state, *s_pos, *o_pos) {
                            match role {
                                Role::Peer => {
                                    if s_flock.name == o_flock.name {
                                        (*o_vel, 1)
                                    } else {
                                        (Vec3A::ZERO, 0)
                                    }
                                }
                                Role::Prey => (*o_vel, 1),
                                Role::Predator | Role::Rival => (Vec3A::ZERO, 0),
                            }
                        } else {
                            (Vec3A::ZERO, 0)
                        };

                        let coh = if !Self::is_too_close(s_flock, *s_state, *s_pos, *o_pos) {
                            match role {
                                Role::Peer => {
                                    if s_flock.name == o_flock.name {
                                        (*o_pos, 1)
                                    } else {
                                        (Vec3A::ZERO, 0)
                                    }
                                }
                                Role::Prey => (*o_pos, 1),
                                Role::Predator | Role::Rival => (-*o_pos, 1),
                            }
                        } else {
                            (Vec3A::ZERO, 0)
                        };

                        (sep.0, sep.1, align.0, align.1, coh.0, coh.1)
                    })
                    .fold(
                        (Vec3A::ZERO, 0, Vec3A::ZERO, 0, Vec3A::ZERO, 0),
                        |(sep_sum, sep_count, align_sum, align_count, coh_sum, coh_count),
                         (sep, sep_c, align, align_c, coh, coh_c)| {
                            (
                                sep_sum + sep,
                                sep_count + sep_c,
                                align_sum + align,
                                align_count + align_c,
                                coh_sum + coh,
                                coh_count + coh_c,
                            )
                        },
                    );

                // Aggregate separation, alignment, and cohesion vectors
                let (separation, alignment, cohesion) = aggregate_behaviors(
                    sep_sum,
                    sep_count,
                    align_sum,
                    align_count,
                    coh_sum,
                    coh_count,
                );

                // Apply behavior and constraints
                apply_behavior_and_constraints(
                    environment,
                    s_flock,
                    s_state,
                    s_bias,
                    s_pos,
                    s_vel,
                    separation,
                    alignment,
                    cohesion,
                );
            });
    }

    /// Parallel version of the update function
    ///
    /// # Arguments
    ///
    /// * `boids` - A reference to the boids collection to update from.
    /// * `environment` - A reference to the environment in which the boids are placed.
    ///
    /// # Panics
    ///
    /// Panics if the `Role` comparison is invalid
    fn par_update<E: Environment>(&mut self, boids: &Self, environment: &E) {
        self.par_iter_mut()
            .for_each(|(s_flock, s_state, s_bias, s_pos, s_vel)| {
                // Calculate separation, alignment, and cohesion vectors in parallel
                let (sep_sum, sep_count, align_sum, align_count, coh_sum, coh_count) =
                    boids
                        .par_iter()
                        .filter(|(_, _, _, o_pos, _)| {
                            s_pos != *o_pos && Self::is_in_sight(s_flock, *s_state, *s_pos, **o_pos)
                        })
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

                            let align = if !Self::is_too_close(s_flock, *s_state, *s_pos, *o_pos) {
                                match role {
                                    Role::Peer => {
                                        if s_flock.name == o_flock.name {
                                            (*o_vel, 1)
                                        } else {
                                            (Vec3A::ZERO, 0)
                                        }
                                    }
                                    Role::Prey => (*o_vel, 1),
                                    Role::Predator | Role::Rival => (Vec3A::ZERO, 0),
                                }
                            } else {
                                (Vec3A::ZERO, 0)
                            };

                            let coh = if !Self::is_too_close(s_flock, *s_state, *s_pos, *o_pos) {
                                match role {
                                    Role::Peer => {
                                        if s_flock.name == o_flock.name {
                                            (*o_pos, 1)
                                        } else {
                                            (Vec3A::ZERO, 0)
                                        }
                                    }
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
                            |(sep_sum, sep_count, align_sum, align_count, coh_sum, coh_count),
                             (sep, sep_c, align, align_c, coh, coh_c)| {
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
                            |(
                                sep_sum1,
                                sep_count1,
                                align_sum1,
                                align_count1,
                                coh_sum1,
                                coh_count1,
                            ),
                             (
                                sep_sum2,
                                sep_count2,
                                align_sum2,
                                align_count2,
                                coh_sum2,
                                coh_count2,
                            )| {
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

                let (separation, alignment, cohesion) = aggregate_behaviors(
                    sep_sum,
                    sep_count,
                    align_sum,
                    align_count,
                    coh_sum,
                    coh_count,
                );

                apply_behavior_and_constraints(
                    environment,
                    s_flock,
                    s_state,
                    s_bias,
                    s_pos,
                    s_vel,
                    separation,
                    alignment,
                    cohesion,
                );
            });
    }
}

/// Function to aggregate the separation, alignment, and cohesion behaviors
///
/// # Arguments
///
/// * `sep_sum` - A `Vec3A` vector representing the sum of separation vectors.
/// * `sep_count` - An integer value representing the count of separation vectors.
/// * `align_sum` - A `Vec3A` vector representing the sum of alignment vectors.
/// * `align_count` - An integer value representing the count of alignment vectors.
/// * `coh_sum` - A `Vec3A` vector representing the sum of cohesion vectors.
/// * `coh_count` - An integer value representing the count of cohesion vectors.
///
/// # Returns
///
/// A tuple of `Vec3A` vectors representing the separation, alignment, and cohesion vectors.
fn aggregate_behaviors(
    sep_sum: Vec3A,
    sep_count: i32,
    align_sum: Vec3A,
    align_count: i32,
    coh_sum: Vec3A,
    coh_count: i32,
) -> (Vec3A, Vec3A, Vec3A) {
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
    (separation, alignment, cohesion)
}

/// Function to apply behavior and constraints to a boid
///
/// # Arguments
///
/// * `environment` - A reference to the environment in which the boids are placed.
/// * `s_flock` - A mutable reference to the flock to which the boid belongs.
/// * `s_state` - A mutable reference to the state of the boid.
/// * `s_bias` - A mutable reference to the bias affecting the boid's movement.
/// * `s_pos` - A mutable reference to the position of the boid.
/// * `s_vel` - A mutable reference to the velocity of the boid.
/// * `separation` - A `Vec3A` vector representing the separation vector.
/// * `alignment` - A `Vec3A` vector representing the alignment vector.
/// * `cohesion` - A `Vec3A` vector representing the cohesion vector.
///
/// # Panics
///
/// Panics if the `Role` comparison is invalid
fn apply_behavior_and_constraints<E: Environment>(
    environment: &E,
    s_flock: &mut Arc<Flock>,
    s_state: &mut f32,
    s_bias: &mut Bias,
    s_pos: &mut Vec3A,
    s_vel: &mut Vec3A,
    separation: Vec3A,
    alignment: Vec3A,
    cohesion: Vec3A,
) {
    // Calculate the overall behavior vector
    let behavior = separation * s_flock.kind.sep_weight
        + (alignment - *s_vel) * s_flock.kind.align_weight
        + (cohesion - *s_pos) * s_flock.kind.coh_weight;

    let vel = *s_vel;

    // Update velocity based on behavior, bias, and environment repulsion
    *s_vel += behavior
        + s_bias.get(*s_pos)
        + environment.repulsion((s_flock, s_state, s_bias, s_pos, s_vel));

    // Clamp the acceleration
    *s_vel = (s_flock
        .kind
        .acceleration
        .scale(*s_state)
        .clamp((*s_vel - vel).length())
        + vel.length())
        * s_vel.normalize();

    // Rotate towards the new velocity direction
    let angle = vel.angle_between(*s_vel) / std::f32::consts::PI;
    *s_vel = rotate_towards(
        *s_vel,
        vel,
        s_flock.kind.angular_speed.scale(*s_state).clamp(angle),
    );

    // Clamp the speed
    *s_vel = s_flock.kind.speed.scale(*s_state).clamp(s_vel.length()) * s_vel.normalize();

    // Update position based on velocity and momentum
    *s_pos += *s_vel + s_flock.kind.momentum * (vel - *s_vel);

    // Apply environment correction
    environment.correction((s_flock, s_state, s_bias, s_pos, s_vel));
}

/// Function to rotate vector `a` towards vector `b` by a given angle
///
/// # Arguments
///
/// * `a` - A `Vec3A` vector representing the vector to rotate.
/// * `b` - A `Vec3A` vector representing the target vector.
/// * `angle` - A floating-point value representing the angle to rotate by.
///
/// # Returns
///
/// A `Vec3A` vector representing the rotated vector.
fn rotate_towards(a: Vec3A, b: Vec3A, angle: f32) -> Vec3A {
    let current_angle = a.angle_between(b) / std::f32::consts::PI;

    if current_angle <= angle {
        return a;
    }

    let t = angle / current_angle;
    a.normalize().lerp(b.normalize(), 1.0 - t).normalize() * a.length()
}

/// Trait for the environment in which the boids are placed
pub trait Environment: Send + Sync {
    /// Calculate the repulsion vector for a given boid
    ///
    /// # Arguments
    ///
    /// * `boid` - A reference to the boid for which to calculate the repulsion vector.
    ///
    /// # Returns
    ///
    /// A `Vec3A` vector representing the repulsion vector.
    fn repulsion(&self, boid: BoidRef) -> Vec3A;

    /// Correct the position of a boid if it goes outside the environment bounds
    ///
    /// # Arguments
    ///
    /// * `boid` - A mutable reference to the boid to correct.
    fn correction(&self, boid: BoidRefMut);
}

/// Structure representing a spherical environment
#[derive(Clone, Debug)]
pub struct SphereEnv {
    pub size: f32,     // Size of the sphere
    pub turnback: f32, // Factor to turn back the boid when it get close to the edge
}

impl SphereEnv {
    /// Constructor to create a new spherical environment
    ///
    /// # Arguments
    ///
    /// * `size` - A floating-point value representing the size of the sphere.
    /// * `turnback` - A floating-point value representing the factor to turn back the boid when it gets close to the edge.
    ///
    /// # Returns
    ///
    /// A `SphereEnv` struct with the given size and turnback factor.
    pub fn new(size: f32, turnback: f32) -> Self {
        Self { size, turnback }
    }
}

impl Environment for SphereEnv {
    fn repulsion(&self, boid: BoidRef) -> Vec3A {
        let (flock, state, _, pos, _) = boid;
        let perspective = (pos.length() + flock.kind.vision.scale(*state).max - self.size)
            / (flock.kind.vision.scale(*state).max / flock.kind.protected.scale(*state).max);
        if perspective > 0.0 {
            -pos.normalize() * perspective * self.turnback
        } else {
            Vec3A::ZERO
        }
    }

    fn correction(&self, boid: BoidRefMut) {
        let (flock, _, _, pos, _) = boid;
        if pos.length() + flock.kind.size / 2.0 > self.size {
            *pos = pos.normalize() * (self.size - flock.kind.size / 2.0)
        }
    }
}

/// Structure representing a bias affecting the boid's movement
///
/// # Fields
///
/// * `weight` - A floating-point value representing the weight of the bias.
/// * `pos` - A `Vec3A` vector representing the position of the bias.
#[derive(Copy, Clone, Debug)]
pub struct Bias {
    pub weight: f32,
    pub pos: Vec3A,
}

impl Bias {
    /// Constructor to create a new bias
    ///
    /// # Arguments
    ///
    /// * `weight` - A floating-point value representing the weight of the bias.
    /// * `pos` - A `Vec3A` vector representing the position of the bias.
    ///
    /// # Returns
    ///
    /// A `Bias` struct with the given weight and position.
    pub fn new(weight: f32, pos: Vec3A) -> Self {
        Self { weight, pos }
    }

    /// Get the bias vector for a given position
    ///
    /// # Arguments
    ///
    /// * `pos` - A `Vec3A` vector representing the position of the boid.
    ///
    /// # Returns
    ///
    /// A `Vec3A` vector representing the bias vector.
    pub fn get(&self, pos: Vec3A) -> Vec3A {
        self.weight * (self.pos - pos).normalize()
    }
}

/// Structure representing a flock of boids
///
/// # Fields
///
/// * `name` - A string representing the name of the flock.
/// * `kind` - A `Kind` struct representing the type of the flock.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Flock {
    pub name: String,
    pub kind: Kind,
}

/// Represents a type of entity in the simulation with various properties.
///
/// # Fields
///
/// * `kind` - A string representing the type of the entity.
/// * `size` - A floating-point value representing the size of the entity.
/// * `color` - A tuple of three `u8` values representing the RGB color of the entity.
/// * `preys` - A vector of strings representing the types of entities that this entity preys on.
/// * `predators` - A vector of strings representing the types of entities that prey on this entity.
/// * `speed` - A `Range` representing the speed range of the entity.
/// * `angular_speed` - A `Range` representing the angular speed range of the entity.
/// * `acceleration` - A `Range` representing the acceleration range of the entity.
/// * `momentum` - A floating-point value representing the momentum of the entity.
/// * `vision` - A `Range` representing the vision range of the entity.
/// * `protected` - A `Range` representing the protected range of the entity.
/// * `sep_weight` - A floating-point value representing the separation weight of the entity.
/// * `align_weight` - A floating-point value representing the alignment weight of the entity.
/// * `coh_weight` - A floating-point value representing the cohesion weight of the entity.
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
    /// Compares this entity with another entity to determine their relationship.
    ///
    /// # Arguments
    ///
    /// * `other` - A reference to another `Kind` struct to compare with.
    ///
    /// # Returns
    ///
    /// A `Role` enum representing the relationship between the two entities.
    ///
    /// # Panics
    ///
    /// Panics if the comparison is invalid
    pub fn compare(&self, other: &Self) -> Role {
        match (
            self.preys.contains(&other.kind),
            self.predators.contains(&other.kind),
            other.preys.contains(&self.kind),
            other.predators.contains(&self.kind),
        ) {
            (false, false, false, false) => Role::Peer,
            (true, false, false, true) => Role::Prey,
            (false, true, true, false) => Role::Predator,
            (true, true, true, true) => Role::Rival,
            _ => {
                panic!("Invalid Comparison")
            }
        }
    }
}

/// Represents a role that an entity can have in the simulation.
///
/// The roles are:
///
/// * `Peer` - An entity that is neither a predator nor a prey to another entity.
/// * `Prey` - An entity that is prey to another entity.
/// * `Predator` - An entity that is a predator to another entity.
/// * `Rival` - An entity that is both a predator and a prey to another entity.
#[derive(Debug, Deserialize, Serialize)]
pub enum Role {
    Peer,
    Prey,
    Predator,
    Rival,
}

/// Represents a range of floating-point values.
///
/// # Fields
///
/// * `min` - The minimum value of the range.
/// * `max` - The maximum value of the range.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Range {
    pub min: f32,
    pub max: f32,
}

impl Range {
    /// Creates a new `Range` struct with the given minimum and maximum values.
    ///
    /// # Arguments
    ///
    /// * `min` - A floating-point value representing the minimum value of the range.
    /// * `max` - A floating-point value representing the maximum value of the range.
    ///
    /// # Returns
    ///
    /// A `Range` struct with the given minimum and maximum values.
    pub fn new(min: f32, max: f32) -> Self {
        Self { min, max }
    }

    /// Checks if a value is contained within the range.
    ///
    /// # Arguments
    ///
    /// * `value` - A floating-point value to check.
    ///
    /// # Returns
    ///
    /// A boolean value indicating whether the value is contained within the range.
    pub fn contains(&self, value: f32) -> bool {
        value >= self.min && value <= self.max
    }

    /// Clamps a value to the range.
    ///
    /// # Arguments
    ///
    /// * `value` - A floating-point value to clamp.
    ///
    /// # Returns
    ///
    /// A floating-point value clamped to the range.
    pub fn clamp(&self, value: f32) -> f32 {
        if value < self.min {
            self.min
        } else if value > self.max {
            self.max
        } else {
            value
        }
    }

    /// Scales the range by a given value.
    ///
    /// # Arguments
    ///
    /// * `value` - A floating-point value to scale the range by.
    ///
    /// # Returns
    ///
    /// A new `Range` struct with the scaled minimum and maximum values.
    pub fn scale(&self, value: f32) -> Self {
        Range::new(self.min * value, self.max * value)
    }

    /// Generates a random value within the range.
    ///
    /// # Returns
    ///
    /// A random floating-point value within the range.
    pub fn random(&self) -> f32 {
        rand::random::<f32>() * (self.max - self.min) + self.min
    }
}
