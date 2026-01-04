//! # Mesh File Loader
//!
//! This module provides functions for loading mesh files in OBJ and STL formats.

use super::mesh::Triangle;
use glam::Vec3A;
use std::fs::File;
use std::io::{BufReader, Read};
use std::path::Path;

/// Error type for mesh loading operations
#[derive(Debug)]
pub enum LoadError {
    /// I/O error during file reading
    Io(std::io::Error),
    /// Error parsing file format
    Parse(String),
    /// Invalid or unsupported file format
    InvalidFormat(String),
}

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LoadError::Io(e) => write!(f, "I/O error: {}", e),
            LoadError::Parse(s) => write!(f, "Parse error: {}", s),
            LoadError::InvalidFormat(s) => write!(f, "Invalid format: {}", s),
        }
    }
}

impl std::error::Error for LoadError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            LoadError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for LoadError {
    fn from(e: std::io::Error) -> Self {
        LoadError::Io(e)
    }
}

impl From<tobj::LoadError> for LoadError {
    fn from(e: tobj::LoadError) -> Self {
        LoadError::Parse(format!("OBJ load error: {:?}", e))
    }
}

/// Load mesh from OBJ file
///
/// Uses the `tobj` crate for robust OBJ parsing with automatic triangulation.
///
/// # Arguments
/// * `path` - Path to the OBJ file
///
/// # Returns
/// Vector of triangles on success, or LoadError on failure
pub fn load_obj<P: AsRef<Path>>(path: P) -> Result<Vec<Triangle>, LoadError> {
    let (models, _materials) = tobj::load_obj(
        path.as_ref(),
        &tobj::LoadOptions {
            triangulate: true,
            single_index: false, // Keep separate indices for positions and normals
            ..Default::default()
        },
    )?;

    let mut triangles = Vec::new();

    for model in models {
        let mesh = &model.mesh;
        let positions = &mesh.positions;
        let normals = &mesh.normals;
        let indices = &mesh.indices;
        let normal_indices = &mesh.normal_indices;
        let has_normals = !normals.is_empty() && !normal_indices.is_empty();

        // Process triangles
        for (face_idx, chunk) in indices.chunks(3).enumerate() {
            if chunk.len() == 3 {
                let i0 = chunk[0] as usize;
                let i1 = chunk[1] as usize;
                let i2 = chunk[2] as usize;

                let p0 = i0 * 3;
                let p1 = i1 * 3;
                let p2 = i2 * 3;

                // Bounds check for positions
                if p0 + 2 >= positions.len() || p1 + 2 >= positions.len() || p2 + 2 >= positions.len()
                {
                    continue;
                }

                let v0 = Vec3A::new(positions[p0], positions[p0 + 1], positions[p0 + 2]);
                let v1 = Vec3A::new(positions[p1], positions[p1 + 1], positions[p1 + 2]);
                let v2 = Vec3A::new(positions[p2], positions[p2 + 1], positions[p2 + 2]);

                // Use explicit normals if available (with separate normal indices)
                if has_normals {
                    let ni_base = face_idx * 3;
                    if ni_base + 2 < normal_indices.len() {
                        let ni0 = normal_indices[ni_base] as usize * 3;
                        let ni1 = normal_indices[ni_base + 1] as usize * 3;
                        let ni2 = normal_indices[ni_base + 2] as usize * 3;
                        
                        if ni0 + 2 < normals.len() && ni1 + 2 < normals.len() && ni2 + 2 < normals.len() {
                            let n0 = Vec3A::new(normals[ni0], normals[ni0 + 1], normals[ni0 + 2]);
                            let n1 = Vec3A::new(normals[ni1], normals[ni1 + 1], normals[ni1 + 2]);
                            let n2 = Vec3A::new(normals[ni2], normals[ni2 + 1], normals[ni2 + 2]);
                            // Average the vertex normals to get face normal
                            let avg_normal = (n0 + n1 + n2).normalize_or_zero();
                            triangles.push(Triangle::with_normal(v0, v1, v2, avg_normal));
                            continue;
                        }
                    }
                }
                
                // Fallback to computed normal
                triangles.push(Triangle::new(v0, v1, v2));
            }
        }
    }

    if triangles.is_empty() {
        return Err(LoadError::InvalidFormat(
            "No triangles found in OBJ file".into(),
        ));
    }

    Ok(triangles)
}

/// Load mesh from binary STL file
///
/// # Arguments
/// * `path` - Path to the STL file
///
/// # Returns
/// Vector of triangles on success, or LoadError on failure
pub fn load_stl<P: AsRef<Path>>(path: P) -> Result<Vec<Triangle>, LoadError> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);

    // Skip 80-byte header
    let mut header = [0u8; 80];
    reader.read_exact(&mut header)?;

    // Read triangle count (4 bytes, little-endian)
    let mut count_bytes = [0u8; 4];
    reader.read_exact(&mut count_bytes)?;
    let triangle_count = u32::from_le_bytes(count_bytes) as usize;

    let mut triangles = Vec::with_capacity(triangle_count);

    for _ in 0..triangle_count {
        // Each triangle: 12 bytes normal + 36 bytes vertices + 2 bytes attribute
        let mut data = [0u8; 50];
        reader.read_exact(&mut data)?;

        // Skip normal (bytes 0-11), read vertices (bytes 12-47)
        let v0 = Vec3A::new(
            f32::from_le_bytes(data[12..16].try_into().unwrap()),
            f32::from_le_bytes(data[16..20].try_into().unwrap()),
            f32::from_le_bytes(data[20..24].try_into().unwrap()),
        );
        let v1 = Vec3A::new(
            f32::from_le_bytes(data[24..28].try_into().unwrap()),
            f32::from_le_bytes(data[28..32].try_into().unwrap()),
            f32::from_le_bytes(data[32..36].try_into().unwrap()),
        );
        let v2 = Vec3A::new(
            f32::from_le_bytes(data[36..40].try_into().unwrap()),
            f32::from_le_bytes(data[40..44].try_into().unwrap()),
            f32::from_le_bytes(data[44..48].try_into().unwrap()),
        );

        triangles.push(Triangle::new(v0, v1, v2));
    }

    if triangles.is_empty() {
        return Err(LoadError::InvalidFormat(
            "No triangles found in STL file".into(),
        ));
    }

    Ok(triangles)
}

/// Load mesh from file, auto-detecting format by extension
///
/// Supports `.obj` and `.stl` file extensions (case-insensitive).
///
/// # Arguments
/// * `path` - Path to the mesh file
///
/// # Returns
/// Vector of triangles on success, or LoadError on failure
pub fn load_mesh<P: AsRef<Path>>(path: P) -> Result<Vec<Triangle>, LoadError> {
    let path = path.as_ref();
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|s| s.to_lowercase());

    match ext.as_deref() {
        Some("obj") => load_obj(path),
        Some("stl") => load_stl(path),
        _ => Err(LoadError::InvalidFormat(format!(
            "Unknown file extension: {:?}. Supported formats: .obj, .stl",
            ext
        ))),
    }
}

/// Transform loaded triangles by applying scale and offset
///
/// # Arguments
/// * `triangles` - Mutable slice of triangles to transform
/// * `scale` - Uniform scale factor
/// * `offset` - Translation offset
pub fn transform_triangles(triangles: &mut [Triangle], scale: f32, offset: Vec3A) {
    for i in 0..triangles.len() {
        let tri = &triangles[i];
        triangles[i] = Triangle::new(
            tri.v0 * scale + offset,
            tri.v1 * scale + offset,
            tri.v2 * scale + offset,
        );
    }
}

/// Center mesh at origin
///
/// Computes the bounding box center and translates all vertices so the
/// mesh is centered at the origin.
///
/// # Arguments
/// * `triangles` - Mutable slice of triangles to center
///
/// # Returns
/// The original center that was subtracted (useful for restoring position)
pub fn center_mesh(triangles: &mut [Triangle]) -> Vec3A {
    if triangles.is_empty() {
        return Vec3A::ZERO;
    }

    // Find bounding box
    let mut min = Vec3A::splat(f32::MAX);
    let mut max = Vec3A::splat(f32::MIN);

    for tri in triangles.iter() {
        min = min.min(tri.v0).min(tri.v1).min(tri.v2);
        max = max.max(tri.v0).max(tri.v1).max(tri.v2);
    }

    let center = (min + max) * 0.5;
    transform_triangles(triangles, 1.0, -center);
    center
}

/// Scale mesh to fit within a target size
///
/// Uniformly scales the mesh so its largest dimension equals `target_size`.
///
/// # Arguments
/// * `triangles` - Mutable slice of triangles to scale
/// * `target_size` - The desired size of the largest dimension
///
/// # Returns
/// The scale factor that was applied
pub fn scale_mesh_to_size(triangles: &mut [Triangle], target_size: f32) -> f32 {
    if triangles.is_empty() {
        return 1.0;
    }

    // Find bounding box
    let mut min = Vec3A::splat(f32::MAX);
    let mut max = Vec3A::splat(f32::MIN);

    for tri in triangles.iter() {
        min = min.min(tri.v0).min(tri.v1).min(tri.v2);
        max = max.max(tri.v0).max(tri.v1).max(tri.v2);
    }

    let size = max - min;
    let max_dim = size.x.max(size.y).max(size.z);

    if max_dim < 1e-6 {
        return 1.0;
    }

    let scale = target_size / max_dim;
    transform_triangles(triangles, scale, Vec3A::ZERO);
    scale
}

/// Get bounding box of mesh
///
/// # Arguments
/// * `triangles` - Slice of triangles
///
/// # Returns
/// Tuple of (min, max) corners of the bounding box
pub fn mesh_bounds(triangles: &[Triangle]) -> (Vec3A, Vec3A) {
    if triangles.is_empty() {
        return (Vec3A::ZERO, Vec3A::ZERO);
    }

    let mut min = Vec3A::splat(f32::MAX);
    let mut max = Vec3A::splat(f32::MIN);

    for tri in triangles.iter() {
        min = min.min(tri.v0).min(tri.v1).min(tri.v2);
        max = max.max(tri.v0).max(tri.v1).max(tri.v2);
    }

    (min, max)
}
