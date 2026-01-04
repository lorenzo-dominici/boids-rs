//! # Mesh and BVH Module
//!
//! This module provides triangle mesh representation with BVH acceleration
//! for efficient distance queries.

use super::sdf::SignedDistance;
use glam::Vec3A;

/// A triangle defined by three vertices
#[derive(Clone, Debug)]
pub struct Triangle {
    pub v0: Vec3A,
    pub v1: Vec3A,
    pub v2: Vec3A,
    normal: Vec3A,
}

impl Triangle {
    /// Create a new triangle from three vertices
    ///
    /// The normal is computed automatically using the right-hand rule.
    pub fn new(v0: Vec3A, v1: Vec3A, v2: Vec3A) -> Self {
        let normal = (v1 - v0).cross(v2 - v0).normalize_or_zero();
        Self { v0, v1, v2, normal }
    }

    /// Create a new triangle from three vertices with an explicit normal
    ///
    /// Use this when the OBJ/mesh file provides explicit normals that should
    /// be trusted over the computed winding-based normal.
    pub fn with_normal(v0: Vec3A, v1: Vec3A, v2: Vec3A, normal: Vec3A) -> Self {
        Self { v0, v1, v2, normal }
    }

    /// Create a copy of this triangle with the normal flipped
    pub fn with_flipped_normal(&self) -> Self {
        Self {
            v0: self.v0,
            v1: self.v1,
            v2: self.v2,
            normal: -self.normal,
        }
    }

    /// Get the triangle's normal vector
    #[inline]
    pub fn normal(&self) -> Vec3A {
        self.normal
    }

    /// Find the closest point on the triangle to a given point
    ///
    /// Uses barycentric coordinates with edge clamping for robust computation.
    pub fn closest_point(&self, p: Vec3A) -> Vec3A {
        let ab = self.v1 - self.v0;
        let ac = self.v2 - self.v0;
        let ap = p - self.v0;

        let d1 = ab.dot(ap);
        let d2 = ac.dot(ap);

        // Check if P in vertex region outside A
        if d1 <= 0.0 && d2 <= 0.0 {
            return self.v0;
        }

        let bp = p - self.v1;
        let d3 = ab.dot(bp);
        let d4 = ac.dot(bp);

        // Check if P in vertex region outside B
        if d3 >= 0.0 && d4 <= d3 {
            return self.v1;
        }

        // Check if P in edge region of AB
        let vc = d1 * d4 - d3 * d2;
        if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            let v = d1 / (d1 - d3);
            return self.v0 + ab * v;
        }

        let cp = p - self.v2;
        let d5 = ab.dot(cp);
        let d6 = ac.dot(cp);

        // Check if P in vertex region outside C
        if d6 >= 0.0 && d5 <= d6 {
            return self.v2;
        }

        // Check if P in edge region of AC
        let vb = d5 * d2 - d1 * d6;
        if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            let w = d2 / (d2 - d6);
            return self.v0 + ac * w;
        }

        // Check if P in edge region of BC
        let va = d3 * d6 - d5 * d4;
        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return self.v1 + (self.v2 - self.v1) * w;
        }

        // P inside face region
        let denom = 1.0 / (va + vb + vc);
        let v = vb * denom;
        let w = vc * denom;
        self.v0 + ab * v + ac * w
    }

    /// Compute signed distance to the triangle
    ///
    /// Sign is determined by which side of the triangle plane the point is on.
    pub fn signed_distance(&self, p: Vec3A) -> f32 {
        let closest = self.closest_point(p);
        let diff = p - closest;
        let dist = diff.length();

        // Sign based on which side of the triangle plane
        if diff.dot(self.normal) >= 0.0 {
            dist
        } else {
            -dist
        }
    }

    /// Get the center (centroid) of the triangle
    #[inline]
    pub fn center(&self) -> Vec3A {
        (self.v0 + self.v1 + self.v2) / 3.0
    }
}

/// Axis-Aligned Bounding Box
#[derive(Clone, Debug)]
pub struct Aabb {
    pub min: Vec3A,
    pub max: Vec3A,
}

impl Aabb {
    /// Create a new AABB from min and max corners
    pub fn new(min: Vec3A, max: Vec3A) -> Self {
        Self { min, max }
    }

    /// Create an AABB from a triangle
    pub fn from_triangle(tri: &Triangle) -> Self {
        Self {
            min: tri.v0.min(tri.v1).min(tri.v2),
            max: tri.v0.max(tri.v1).max(tri.v2),
        }
    }

    /// Merge two AABBs into one that contains both
    pub fn merge(&self, other: &Aabb) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// Expand AABB by a uniform margin
    pub fn expand(&self, margin: f32) -> Self {
        Self {
            min: self.min - Vec3A::splat(margin),
            max: self.max + Vec3A::splat(margin),
        }
    }

    /// Get the center of the AABB
    #[inline]
    pub fn center(&self) -> Vec3A {
        (self.min + self.max) * 0.5
    }

    /// Get the size (extent) of the AABB
    #[inline]
    pub fn size(&self) -> Vec3A {
        self.max - self.min
    }

    /// Distance from a point to the AABB surface (0 if inside)
    #[inline]
    pub fn distance_to_point(&self, p: Vec3A) -> f32 {
        let clamped = p.clamp(self.min, self.max);
        (p - clamped).length()
    }

    /// Check if a point is inside the AABB
    #[inline]
    pub fn contains(&self, p: Vec3A) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }
}

/// BVH node - either a leaf with triangle index or internal with children
#[derive(Debug)]
pub enum BvhNode {
    Leaf {
        bounds: Aabb,
        triangle_idx: usize,
    },
    Internal {
        bounds: Aabb,
        left: Box<BvhNode>,
        right: Box<BvhNode>,
    },
}

impl BvhNode {
    /// Get the bounds of this node
    pub fn bounds(&self) -> &Aabb {
        match self {
            BvhNode::Leaf { bounds, .. } => bounds,
            BvhNode::Internal { bounds, .. } => bounds,
        }
    }
}

/// Mesh with BVH acceleration structure for efficient distance queries
pub struct MeshSdf {
    triangles: Vec<Triangle>,
    bvh: BvhNode,
    bounds: Aabb,
}

impl MeshSdf {
    /// Build MeshSdf from triangles
    ///
    /// Constructs a BVH (Bounding Volume Hierarchy) for efficient spatial queries.
    /// Trusts the provided triangle normals - ensure your mesh has consistent normals.
    ///
    /// # Panics
    /// Panics if the triangles vector is empty.
    pub fn new(triangles: Vec<Triangle>) -> Self {
        assert!(!triangles.is_empty(), "Mesh must have at least one triangle");
        
        // Trust the normals from the mesh file - do not auto-correct
        // For CAD-generated meshes (like Onshape), the explicit normals are correct

        let indices: Vec<usize> = (0..triangles.len()).collect();
        let bounds = triangles
            .iter()
            .map(Aabb::from_triangle)
            .reduce(|a, b| a.merge(&b))
            .unwrap();

        let bvh = Self::build_bvh(&triangles, indices);

        Self {
            triangles,
            bvh,
            bounds,
        }
    }

    /// Build BVH recursively using median split
    fn build_bvh(triangles: &[Triangle], mut indices: Vec<usize>) -> BvhNode {
        // Base case: single triangle
        if indices.len() == 1 {
            let idx = indices[0];
            return BvhNode::Leaf {
                bounds: Aabb::from_triangle(&triangles[idx]),
                triangle_idx: idx,
            };
        }

        // Compute bounds for all triangles
        let bounds = indices
            .iter()
            .map(|&i| Aabb::from_triangle(&triangles[i]))
            .reduce(|a, b| a.merge(&b))
            .unwrap();

        // Find longest axis for split
        let size = bounds.size();
        let axis = if size.x >= size.y && size.x >= size.z {
            0
        } else if size.y >= size.z {
            1
        } else {
            2
        };

        // Sort by centroid along chosen axis
        indices.sort_by(|&a, &b| {
            let ca = triangles[a].center();
            let cb = triangles[b].center();
            let va = [ca.x, ca.y, ca.z][axis];
            let vb = [cb.x, cb.y, cb.z][axis];
            va.partial_cmp(&vb).unwrap()
        });

        // Split at median
        let mid = indices.len() / 2;
        let left_indices = indices[..mid].to_vec();
        let right_indices = indices[mid..].to_vec();

        let left = Box::new(Self::build_bvh(triangles, left_indices));
        let right = Box::new(Self::build_bvh(triangles, right_indices));

        let merged_bounds = left.bounds().merge(right.bounds());

        BvhNode::Internal {
            bounds: merged_bounds,
            left,
            right,
        }
    }

    /// Find closest point on mesh to given point
    ///
    /// # Returns
    /// Tuple of (distance, closest_point, triangle_index)
    pub fn closest_point(&self, p: Vec3A) -> (f32, Vec3A, usize) {
        let mut best = (f32::MAX, Vec3A::ZERO, 0usize);
        self.closest_point_recursive(&self.bvh, p, &mut best);
        best
    }

    fn closest_point_recursive(&self, node: &BvhNode, p: Vec3A, best: &mut (f32, Vec3A, usize)) {
        match node {
            BvhNode::Leaf { triangle_idx, .. } => {
                let tri = &self.triangles[*triangle_idx];
                let closest = tri.closest_point(p);
                let dist = (p - closest).length();
                if dist < best.0 {
                    *best = (dist, closest, *triangle_idx);
                }
            }
            BvhNode::Internal {
                bounds,
                left,
                right,
                ..
            } => {
                // Early exit if bounds are farther than current best
                if bounds.distance_to_point(p) >= best.0 {
                    return;
                }

                // Visit closer child first for better pruning
                let left_dist = left.bounds().distance_to_point(p);
                let right_dist = right.bounds().distance_to_point(p);

                if left_dist < right_dist {
                    self.closest_point_recursive(left, p, best);
                    self.closest_point_recursive(right, p, best);
                } else {
                    self.closest_point_recursive(right, p, best);
                    self.closest_point_recursive(left, p, best);
                }
            }
        }
    }

    /// Get mesh bounds
    pub fn bounds(&self) -> &Aabb {
        &self.bounds
    }

    /// Get triangle count
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Get triangles for visualization
    pub fn triangles(&self) -> &[Triangle] {
        &self.triangles
    }

    /// Find all triangles within a given range from a point.
    /// Returns a vector of (distance, closest_point, triangle_index, triangle_normal).
    pub fn triangles_in_range(&self, p: Vec3A, range: f32) -> Vec<(f32, Vec3A, usize, Vec3A)> {
        let mut results = Vec::new();
        self.triangles_in_range_recursive(&self.bvh, p, range, &mut results);
        results
    }

    fn triangles_in_range_recursive(
        &self,
        node: &BvhNode,
        p: Vec3A,
        range: f32,
        results: &mut Vec<(f32, Vec3A, usize, Vec3A)>,
    ) {
        match node {
            BvhNode::Leaf { triangle_idx, bounds } => {
                // Early exit if bounds are too far
                if bounds.distance_to_point(p) > range {
                    return;
                }
                let tri = &self.triangles[*triangle_idx];
                let closest = tri.closest_point(p);
                let dist = (p - closest).length();
                if dist <= range {
                    results.push((dist, closest, *triangle_idx, tri.normal()));
                }
            }
            BvhNode::Internal { bounds, left, right, .. } => {
                // Early exit if bounds are too far
                if bounds.distance_to_point(p) > range {
                    return;
                }
                self.triangles_in_range_recursive(left, p, range, results);
                self.triangles_in_range_recursive(right, p, range, results);
            }
        }
    }

    /// Find the K most significant triangles for repulsion calculation.
    /// 
    /// Significance is computed as: `alignment / distance`
    /// where:
    /// - `alignment` = how much the boid is heading toward the triangle (0 to 1)
    /// - `distance` = distance from boid to closest point on triangle
    /// 
    /// This efficiently selects triangles that will contribute most to repulsion,
    /// prioritizing close triangles that the boid is moving toward.
    /// 
    /// All repulsion filtering conditions are applied during traversal:
    /// 1. Boid must be on correct side of triangle (based on inverted flag)
    /// 2. Boid must be moving toward the surface (not away)
    /// 3. Triangle must be in the boid's trajectory path
    /// 4. Trajectory angle check for surface intersection
    /// 
    /// Uses iterative BVH traversal with an explicit stack for better performance
    /// (avoids function call overhead and stack frame allocation of recursion).
    /// 
    /// Returns a vector of (significance, distance, closest_point, triangle_index, triangle_normal).
    pub fn most_significant_triangles(
        &self,
        pos: Vec3A,
        vel_dir: Vec3A,
        range: f32,
        max_count: usize,
        inverted: bool,
    ) -> Vec<(f32, f32, Vec3A, usize, Vec3A)> {
        // Result heap - keeps track of top K most significant triangles
        let mut results: Vec<(f32, f32, Vec3A, usize, Vec3A)> = Vec::with_capacity(max_count + 1);
        
        // Explicit stack for iterative traversal
        // Capacity 32 is sufficient for BVH depth ~14 (covers millions of triangles)
        let mut stack: Vec<&BvhNode> = Vec::with_capacity(32);
        stack.push(&self.bvh);
        
        // Track minimum significance in results for efficient pruning
        let mut min_significance: f32 = 0.0;
        
        while let Some(node) = stack.pop() {
            match node {
                BvhNode::Leaf { triangle_idx, bounds } => {
                    let bounds_dist = bounds.distance_to_point(pos);
                    
                    // Early exit if bounds are too far
                    if bounds_dist > range {
                        continue;
                    }
                    
                    // Compute upper bound on significance for this AABB
                    // Best case: distance = bounds_dist, alignment = 1.0
                    let max_possible_significance = if bounds_dist > f32::EPSILON {
                        1.0 / bounds_dist
                    } else {
                        f32::INFINITY
                    };
                    
                    // Prune if can't beat current minimum
                    if results.len() >= max_count && max_possible_significance <= min_significance {
                        continue;
                    }
                    
                    let tri = &self.triangles[*triangle_idx];
                    let closest = tri.closest_point(pos);
                    let to_boid = pos - closest;
                    let dist = to_boid.length();
                    
                    if dist > range || dist < f32::EPSILON {
                        continue;
                    }
                    
                    // Get corrected normal based on inversion
                    let tri_normal = tri.normal();
                    let repulsive_dir = if inverted { -tri_normal } else { tri_normal };
                    
                    // === CONDITION 1: Boid must be on correct side of triangle ===
                    if to_boid.dot(repulsive_dir) < 0.0 {
                        continue;
                    }
                    
                    // === CONDITION 2: Boid must be moving toward the surface ===
                    if vel_dir.dot(repulsive_dir) >= 0.0 {
                        continue;
                    }
                    
                    let to_boid_dir = to_boid / dist;
                    
                    // === CONDITION 3: Triangle must be in trajectory path ===
                    if repulsive_dir.dot(to_boid_dir) + f32::EPSILON < repulsive_dir.dot(-vel_dir) {
                        continue;
                    }
                    
                    // === CONDITION 4: Trajectory angle check ===
                    if repulsive_dir.dot(-vel_dir) > to_boid_dir.dot(-vel_dir) + f32::EPSILON {
                        continue;
                    }
                    
                    // Compute alignment: how much velocity points toward the triangle
                    let to_surface_dir = -to_boid_dir;
                    let alignment = vel_dir.dot(to_surface_dir).max(0.0);
                    
                    // Significance = alignment / distance
                    let significance = alignment / dist;
                    
                    // Only add if significant enough
                    if results.len() < max_count || significance > min_significance {
                        results.push((significance, dist, closest, *triangle_idx, tri_normal));
                        
                        // If we exceeded max_count, remove the least significant
                        if results.len() > max_count {
                            let min_idx = results
                                .iter()
                                .enumerate()
                                .min_by(|a, b| a.1.0.partial_cmp(&b.1.0).unwrap_or(std::cmp::Ordering::Equal))
                                .map(|(i, _)| i)
                                .unwrap();
                            results.swap_remove(min_idx);
                        }
                        
                        // Update min_significance for better pruning
                        if results.len() >= max_count {
                            min_significance = results
                                .iter()
                                .map(|r| r.0)
                                .fold(f32::INFINITY, f32::min);
                        }
                    }
                }
                BvhNode::Internal { bounds, left, right, .. } => {
                    let bounds_dist = bounds.distance_to_point(pos);
                    
                    // Early exit if bounds are too far
                    if bounds_dist > range {
                        continue;
                    }
                    
                    // Compute upper bound on significance for this AABB
                    let max_possible_significance = if bounds_dist > f32::EPSILON {
                        1.0 / bounds_dist
                    } else {
                        f32::INFINITY
                    };
                    
                    // Prune entire subtree if can't beat current minimum
                    if results.len() >= max_count && max_possible_significance <= min_significance {
                        continue;
                    }
                    
                    // Compute which child is more "in front" of the boid (better pruning first)
                    let left_center = left.bounds().center();
                    let right_center = right.bounds().center();
                    let left_ahead = vel_dir.dot(left_center - pos);
                    let right_ahead = vel_dir.dot(right_center - pos);
                    
                    // Push in reverse order so the better candidate is popped first
                    // (stack is LIFO, so push the one we want to visit last first)
                    if left_ahead >= right_ahead {
                        stack.push(right.as_ref());
                        stack.push(left.as_ref());
                    } else {
                        stack.push(left.as_ref());
                        stack.push(right.as_ref());
                    }
                }
            }
        }
        
        // Sort by significance descending for consistent iteration order
        results.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        results
    }
}

impl SignedDistance for MeshSdf {
    fn distance(&self, point: Vec3A) -> f32 {
        let (dist, closest, tri_idx) = self.closest_point(point);

        // Determine sign using triangle normal
        let tri = &self.triangles[tri_idx];
        let to_point = point - closest;

        if to_point.dot(tri.normal()) >= 0.0 {
            dist // Outside (same side as normal)
        } else {
            -dist // Inside (opposite side)
        }
    }

    fn gradient(&self, point: Vec3A) -> Vec3A {
        let (_dist, closest, tri_idx) = self.closest_point(point);
        let tri = &self.triangles[tri_idx];
        let diff = point - closest;
        let len = diff.length();

        if len > 1e-6 {
            let raw_dir = diff / len;
            // Gradient should always point in direction of INCREASING signed distance
            // Outside (positive dist): gradient points away from surface (same as raw_dir)
            // Inside (negative dist): gradient points toward surface (opposite of raw_dir)
            let is_inside = diff.dot(tri.normal()) < 0.0;
            if is_inside {
                -raw_dir  // Point toward surface when inside
            } else {
                raw_dir   // Point away from surface when outside
            }
        } else {
            // On surface, use triangle normal (points outward = positive direction)
            tri.normal()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_box_triangles(min: Vec3A, max: Vec3A) -> Vec<Triangle> {
        // Create a simple box from 12 triangles (2 per face)
        let corners = [
            Vec3A::new(min.x, min.y, min.z), // 0: ---
            Vec3A::new(max.x, min.y, min.z), // 1: +--
            Vec3A::new(max.x, max.y, min.z), // 2: ++-
            Vec3A::new(min.x, max.y, min.z), // 3: -+-
            Vec3A::new(min.x, min.y, max.z), // 4: --+
            Vec3A::new(max.x, min.y, max.z), // 5: +-+
            Vec3A::new(max.x, max.y, max.z), // 6: +++
            Vec3A::new(min.x, max.y, max.z), // 7: -++
        ];
        
        // Each face has 2 triangles, winding order for outward normals
        vec![
            // Front face (z = max)
            Triangle::new(corners[4], corners[5], corners[6]),
            Triangle::new(corners[4], corners[6], corners[7]),
            // Back face (z = min)
            Triangle::new(corners[1], corners[0], corners[3]),
            Triangle::new(corners[1], corners[3], corners[2]),
            // Right face (x = max)
            Triangle::new(corners[1], corners[2], corners[6]),
            Triangle::new(corners[1], corners[6], corners[5]),
            // Left face (x = min)
            Triangle::new(corners[0], corners[4], corners[7]),
            Triangle::new(corners[0], corners[7], corners[3]),
            // Top face (y = max)
            Triangle::new(corners[3], corners[7], corners[6]),
            Triangle::new(corners[3], corners[6], corners[2]),
            // Bottom face (y = min)
            Triangle::new(corners[0], corners[1], corners[5]),
            Triangle::new(corners[0], corners[5], corners[4]),
        ]
    }

    #[test]
    fn test_triangle_closest_point() {
        let tri = Triangle::new(
            Vec3A::new(0.0, 0.0, 0.0),
            Vec3A::new(1.0, 0.0, 0.0),
            Vec3A::new(0.0, 1.0, 0.0),
        );

        // Point above triangle center
        let p = Vec3A::new(0.25, 0.25, 1.0);
        let closest = tri.closest_point(p);
        assert!((closest - Vec3A::new(0.25, 0.25, 0.0)).length() < 0.001);
    }

    #[test]
    fn test_aabb_distance() {
        let aabb = Aabb::new(Vec3A::ZERO, Vec3A::ONE);

        // Inside
        assert_eq!(aabb.distance_to_point(Vec3A::splat(0.5)), 0.0);

        // Outside
        let dist = aabb.distance_to_point(Vec3A::new(2.0, 0.5, 0.5));
        assert!((dist - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_mesh_sdf_box() {
        let triangles = create_box_triangles(Vec3A::ZERO, Vec3A::ONE);
        let mesh = MeshSdf::new(triangles);

        // Test distance at center (should be inside, negative)
        let center_dist = mesh.distance(Vec3A::splat(0.5));
        assert!(center_dist < 0.0, "Center should be inside (negative distance)");

        // Test distance outside (should be positive)
        let outside_dist = mesh.distance(Vec3A::splat(2.0));
        assert!(outside_dist > 0.0, "Point outside should have positive distance");
    }
}
