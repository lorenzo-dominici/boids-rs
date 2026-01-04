//! # Environment Module
//!
//! This module provides abstractions for 3D mesh-based environments in the boids simulation.
//!
//! ## Architecture
//!
//! The environment system uses Artificial Potential Fields (APF) for smooth repulsion.
//! Multiple triangles within the boid's vision range contribute to the repulsion,
//! weighted by their distance to create smooth behavior near edges and corners.
//!
//! ## Usage
//!
//! ```rust,ignore
//! use boids::environment::{MeshEnvironment, RepulsionConfig};
//!
//! // Load a mesh environment
//! let env = MeshEnvironment::from_file(
//!     "path/to/mesh.obj",
//!     repulsion_config,
//!     true,  // inverted (boids stay inside)
//! )?;
//! ```

pub mod sdf;
pub mod mesh;
pub mod loader;

pub use sdf::SignedDistance;
pub use mesh::{Triangle, Aabb, MeshSdf};
pub use loader::{load_mesh, load_obj, load_stl, LoadError, transform_triangles, center_mesh};

use crate::{BoidRef, BoidRefMut, Environment};
use glam::Vec3A;

/// Configuration for APF-based environment repulsion
/// All values are specified in the external TOML configuration file.
#[derive(Clone, Debug)]
pub struct RepulsionConfig {
    /// Repulsion strength factor
    pub repulsion_strength: f32,
    
    /// Exponent for distance-based falloff
    /// Higher values create steeper falloff near surface
    pub falloff_exponent: f32,
    
    /// Exponent for distance-based weighting of multiple triangles
    /// Higher values give more weight to closer triangles
    pub weight_exponent: f32,
    
    /// Safety margin multiplier for boid size in correction
    pub size_margin: f32,
    
    /// Maximum number of triangles to consider for repulsion calculation
    /// Higher values = more accurate but slower
    /// Typical values: 8-32
    pub max_triangles: usize,

    /// Steering strength for turnover effect when inside mesh
    /// Higher values create stronger turning away from surface
    /// Typical values: 0.3-0.7
    pub steering_strength: f32,
}

/// Mesh-based environment using Artificial Potential Fields (APF)
///
/// Computes repulsion by considering all triangles within vision range,
/// weighted by distance, to create smooth behavior near edges and corners.
pub struct MeshEnvironment {
    mesh: MeshSdf,
    config: RepulsionConfig,
    inverted: bool,
}

impl MeshEnvironment {
    /// Create environment from mesh file
    pub fn from_file<P: AsRef<std::path::Path>>(
        path: P,
        config: RepulsionConfig,
        inverted: bool,
    ) -> Result<Self, LoadError> {
        let mut triangles = load_mesh(path)?;
        center_mesh(&mut triangles);
        let mesh = MeshSdf::new(triangles);
        Ok(Self { mesh, config, inverted })
    }

    /// Create from pre-loaded triangles
    pub fn from_triangles(
        triangles: Vec<Triangle>,
        config: RepulsionConfig,
        inverted: bool,
    ) -> Self {
        let mesh = MeshSdf::new(triangles);
        Self { mesh, config, inverted }
    }

    /// Get mesh for visualization
    pub fn mesh(&self) -> &MeshSdf {
        &self.mesh
    }

    /// Get repulsion config
    pub fn config(&self) -> &RepulsionConfig {
        &self.config
    }

    /// Check if environment is inverted (boids inside mesh)
    pub fn is_inverted(&self) -> bool {
        self.inverted
    }

    /// Calculate APF repulsion considering the most significant triangles within range.
    /// 
    /// Uses a smart BVH query that selects triangles based on significance score:
    /// significance = alignment / distance
    /// 
    /// All filtering is done during BVH traversal, so returned triangles are
    /// guaranteed to contribute to repulsion (no post-filtering needed).
    fn calculate_apf_repulsion(
        &self,
        pos: Vec3A,
        vel: Vec3A,
        speed: f32,
        vision: f32,
    ) -> Vec3A {
        let vel_dir = vel.normalize_or_zero();
        
        // Skip if not moving
        if vel_dir == Vec3A::ZERO {
            return Vec3A::ZERO;
        }
        
        // Find the most significant triangles (fully pre-filtered)
        let significant_triangles = self.mesh.most_significant_triangles(
            pos,
            vel_dir,
            vision,
            self.config.max_triangles,
            self.inverted,
        );
        
        if significant_triangles.is_empty() {
            return Vec3A::ZERO;
        }
        
        // Compute weighted repulsion from significant triangles
        // All triangles here are guaranteed to contribute (pre-filtered in BVH query)
        let mut total_force = Vec3A::ZERO;
        let mut total_weight = 0.0;
        
        for (_significance, dist, _closest_point, _tri_idx, tri_normal) in significant_triangles {
            // Correct normal based on inversion
            let repulsive_dir = if self.inverted { -tri_normal } else { tri_normal };
            
            // Calculate force magnitude using APF potential
            let normalized_dist = (dist / vision).clamp(f32::EPSILON, 1.0);
            let potential = 1.0 / normalized_dist;
            let force_magnitude = potential.powf(self.config.falloff_exponent);
            
            // Weight by inverse distance
            let weight = 1.0 / normalized_dist.powf(self.config.weight_exponent);

            // Project force onto normal direction relative to velocity
            let normal_force = repulsive_dir * repulsive_dir.dot(-vel_dir);

            let parallel_force = (vel - vel.project_onto(-repulsive_dir)).normalize_or_zero() * self.config.steering_strength;

            total_force += normal_force * force_magnitude * weight + parallel_force * force_magnitude * weight;
            total_weight += weight;
        }
        
        if total_weight < f32::EPSILON {
            return Vec3A::ZERO;
        }
        
        // Normalize by total weight and scale
        (total_force / total_weight) * self.config.repulsion_strength * speed
    }
}

impl Environment for MeshEnvironment {
    fn repulsion(&self, boid: BoidRef) -> Vec3A {
        let (flock, state, _, pos, vel) = boid;
        let kind = &flock.kind;
        
        // Get boid parameters scaled by state
        let speed_range = kind.speed.scale(*state);
        let speed = vel.length().max(speed_range.min);
        
        self.calculate_apf_repulsion(*pos, *vel, speed, kind.vision.max)
    }

    fn correction(&self, boid: BoidRefMut) {
        let (flock, _, _, pos, _) = boid;

        // Get closest point on mesh to boid position
        let (_dist, closest_point, tri_idx) = self.mesh.closest_point(*pos);
        let tri_normal = self.mesh.triangles()[tri_idx].normal();
        let correct_tri_normal = if self.inverted { -tri_normal } else { tri_normal };

        // Normalized correction direction
        let correction_dir = correct_tri_normal.normalize();
        
        // Direction from closest point to boid
        let to_boid = *pos - closest_point;
        
        // Minimum safe distance from surface
        let min_safe_dist = flock.kind.size * self.config.size_margin;

        // Determine if boid is on the correct side
        if to_boid.dot(correction_dir) > 0.0 && to_boid.length() >= min_safe_dist {
            return; // No correction needed if on correct side
        }
        
        // Move to closest point, then push in safe direction
        *pos = closest_point + correction_dir * min_safe_dist;
    }
}

impl std::fmt::Debug for MeshEnvironment {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MeshEnvironment")
            .field("mesh_triangles", &self.mesh.triangle_count())
            .field("mesh_bounds", &self.mesh.bounds())
            .field("config", &self.config)
            .field("inverted", &self.inverted)
            .finish()
    }
}
