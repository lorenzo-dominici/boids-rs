//! # Signed Distance Function Trait
//!
//! This module defines the `SignedDistance` trait, which provides an interface
//! for computing signed distances and gradients from arbitrary 3D shapes.
//!
//! ## Convention
//!
//! - **Negative values**: Point is inside the surface
//! - **Positive values**: Point is outside the surface
//! - **Zero**: Point is on the surface

use glam::Vec3A;

/// Signed Distance Function trait
///
/// Implementors provide distance and gradient queries for 3D shapes.
/// The trait is designed to be thread-safe (`Send + Sync`) for parallel processing.
pub trait SignedDistance: Send + Sync {
    /// Compute signed distance from a point to the surface
    ///
    /// # Arguments
    /// * `point` - The 3D point to query
    ///
    /// # Returns
    /// - Negative if inside the surface
    /// - Positive if outside the surface
    /// - Zero if on the surface
    fn distance(&self, point: Vec3A) -> f32;

    /// Compute gradient (direction away from nearest surface)
    ///
    /// The default implementation uses central differences for numerical
    /// gradient computation. Implementors can override this with analytical
    /// gradients for better performance.
    ///
    /// # Arguments
    /// * `point` - The 3D point to query
    ///
    /// # Returns
    /// A normalized vector pointing away from the nearest surface point
    fn gradient(&self, point: Vec3A) -> Vec3A {
        const EPS: f32 = 0.001;
        let dx = self.distance(point + Vec3A::X * EPS) - self.distance(point - Vec3A::X * EPS);
        let dy = self.distance(point + Vec3A::Y * EPS) - self.distance(point - Vec3A::Y * EPS);
        let dz = self.distance(point + Vec3A::Z * EPS) - self.distance(point - Vec3A::Z * EPS);

        let grad = Vec3A::new(dx, dy, dz);
        let len = grad.length();
        if len > 1e-6 {
            grad / len
        } else {
            Vec3A::ZERO
        }
    }
}

/// A sphere SDF primitive
#[derive(Clone, Debug)]
pub struct SphereSdf {
    pub center: Vec3A,
    pub radius: f32,
}

impl SphereSdf {
    pub fn new(center: Vec3A, radius: f32) -> Self {
        Self { center, radius }
    }
}

impl SignedDistance for SphereSdf {
    #[inline]
    fn distance(&self, point: Vec3A) -> f32 {
        (point - self.center).length() - self.radius
    }

    #[inline]
    fn gradient(&self, point: Vec3A) -> Vec3A {
        (point - self.center).normalize_or_zero()
    }
}

/// An axis-aligned box SDF primitive
#[derive(Clone, Debug)]
pub struct BoxSdf {
    pub center: Vec3A,
    pub half_extents: Vec3A,
}

impl BoxSdf {
    pub fn new(center: Vec3A, half_extents: Vec3A) -> Self {
        Self { center, half_extents }
    }
}

impl SignedDistance for BoxSdf {
    fn distance(&self, point: Vec3A) -> f32 {
        let p = (point - self.center).abs();
        let q = p - self.half_extents;
        q.max(Vec3A::ZERO).length() + q.x.max(q.y.max(q.z)).min(0.0)
    }

    fn gradient(&self, point: Vec3A) -> Vec3A {
        let p = point - self.center;
        let sign = p.signum();
        let q = p.abs() - self.half_extents;

        // Determine which face/edge/corner we're closest to
        if q.x > q.y && q.x > q.z {
            Vec3A::new(sign.x, 0.0, 0.0)
        } else if q.y > q.z {
            Vec3A::new(0.0, sign.y, 0.0)
        } else {
            Vec3A::new(0.0, 0.0, sign.z)
        }
    }
}

/// A cylinder SDF primitive (aligned with Y-axis)
#[derive(Clone, Debug)]
pub struct CylinderSdf {
    pub center: Vec3A,
    pub radius: f32,
    pub half_height: f32,
}

impl CylinderSdf {
    pub fn new(center: Vec3A, radius: f32, half_height: f32) -> Self {
        Self { center, radius, half_height }
    }
}

impl SignedDistance for CylinderSdf {
    fn distance(&self, point: Vec3A) -> f32 {
        let p = point - self.center;
        let d_xz = Vec3A::new(p.x, 0.0, p.z).length() - self.radius;
        let d_y = p.y.abs() - self.half_height;

        let outside = Vec3A::new(d_xz.max(0.0), d_y.max(0.0), 0.0).length();
        let inside = d_xz.max(d_y).min(0.0);
        outside + inside
    }
}

/// A plane SDF defined by normal and distance from origin
#[derive(Clone, Debug)]
pub struct PlaneSdf {
    pub normal: Vec3A,
    pub distance: f32,
}

impl PlaneSdf {
    pub fn new(normal: Vec3A, distance: f32) -> Self {
        Self {
            normal: normal.normalize(),
            distance,
        }
    }
}

impl SignedDistance for PlaneSdf {
    #[inline]
    fn distance(&self, point: Vec3A) -> f32 {
        point.dot(self.normal) - self.distance
    }

    #[inline]
    fn gradient(&self, _point: Vec3A) -> Vec3A {
        self.normal
    }
}

/// Inverts an SDF (inside becomes outside and vice versa)
#[derive(Clone, Debug)]
pub struct InvertedSdf<S: SignedDistance> {
    pub inner: S,
}

impl<S: SignedDistance> InvertedSdf<S> {
    pub fn new(inner: S) -> Self {
        Self { inner }
    }
}

impl<S: SignedDistance> SignedDistance for InvertedSdf<S> {
    #[inline]
    fn distance(&self, point: Vec3A) -> f32 {
        -self.inner.distance(point)
    }

    #[inline]
    fn gradient(&self, point: Vec3A) -> Vec3A {
        -self.inner.gradient(point)
    }
}

/// Union of two SDFs (exterior of both shapes)
#[derive(Clone, Debug)]
pub struct UnionSdf<A: SignedDistance, B: SignedDistance> {
    pub a: A,
    pub b: B,
}

impl<A: SignedDistance, B: SignedDistance> UnionSdf<A, B> {
    pub fn new(a: A, b: B) -> Self {
        Self { a, b }
    }
}

impl<A: SignedDistance, B: SignedDistance> SignedDistance for UnionSdf<A, B> {
    #[inline]
    fn distance(&self, point: Vec3A) -> f32 {
        self.a.distance(point).min(self.b.distance(point))
    }
}

/// Intersection of two SDFs (interior of both shapes)
#[derive(Clone, Debug)]
pub struct IntersectionSdf<A: SignedDistance, B: SignedDistance> {
    pub a: A,
    pub b: B,
}

impl<A: SignedDistance, B: SignedDistance> IntersectionSdf<A, B> {
    pub fn new(a: A, b: B) -> Self {
        Self { a, b }
    }
}

impl<A: SignedDistance, B: SignedDistance> SignedDistance for IntersectionSdf<A, B> {
    #[inline]
    fn distance(&self, point: Vec3A) -> f32 {
        self.a.distance(point).max(self.b.distance(point))
    }
}

/// Difference of two SDFs (A minus B)
#[derive(Clone, Debug)]
pub struct DifferenceSdf<A: SignedDistance, B: SignedDistance> {
    pub a: A,
    pub b: B,
}

impl<A: SignedDistance, B: SignedDistance> DifferenceSdf<A, B> {
    pub fn new(a: A, b: B) -> Self {
        Self { a, b }
    }
}

impl<A: SignedDistance, B: SignedDistance> SignedDistance for DifferenceSdf<A, B> {
    #[inline]
    fn distance(&self, point: Vec3A) -> f32 {
        self.a.distance(point).max(-self.b.distance(point))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sphere_sdf() {
        let sphere = SphereSdf::new(Vec3A::ZERO, 1.0);

        // Center should be inside (negative)
        assert!(sphere.distance(Vec3A::ZERO) < 0.0);

        // Surface should be zero
        assert!((sphere.distance(Vec3A::X) - 0.0).abs() < 0.001);

        // Outside should be positive
        assert!(sphere.distance(Vec3A::X * 2.0) > 0.0);
    }

    #[test]
    fn test_box_sdf() {
        let box_sdf = BoxSdf::new(Vec3A::ZERO, Vec3A::ONE);

        // Center should be inside
        assert!(box_sdf.distance(Vec3A::ZERO) < 0.0);

        // On surface
        assert!((box_sdf.distance(Vec3A::X).abs()) < 0.001);

        // Outside
        assert!(box_sdf.distance(Vec3A::X * 2.0) > 0.0);
    }

    #[test]
    fn test_inverted_sdf() {
        let sphere = SphereSdf::new(Vec3A::ZERO, 1.0);
        let inverted = InvertedSdf::new(sphere);

        // Center should now be outside (positive)
        assert!(inverted.distance(Vec3A::ZERO) > 0.0);

        // Outside the original sphere should now be inside (negative)
        assert!(inverted.distance(Vec3A::X * 2.0) < 0.0);
    }
}
