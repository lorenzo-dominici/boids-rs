# Boids-rs

Boids-rs is a Rust project that simulates [boids](https://en.wikipedia.org/wiki/Boids) behavior using [`rayon`](https://docs.rs/rayon/latest/rayon/) CPU parallelization. The project supports arbitrary 3D mesh environments (OBJ/STL) with Artificial Potential Field (APF) repulsion and uses [`rerun.io`](https://rerun.io) for real-time visualization.

## Features

- **Boids Simulation**: Classic flocking behavior with separation, alignment, and cohesion
- **Parallel Computation**: High-performance multi-threaded updates using [`rayon`](https://docs.rs/rayon/latest/rayon/)
- **Dual Memory Layouts**: Both Array of Structures (AoS) and Struct of Arrays (SoA) implementations
- **3D Mesh Environments**: Load arbitrary geometry from OBJ or STL files
- **Artificial Potential Fields**: Smooth environment repulsion using APF with BVH acceleration
- **Configurable Flocks**: TOML-based configuration for multiple flock types with predator-prey relationships
- **Real-time Visualization**: Integration with [`rerun.io`](https://rerun.io) for 3D visualization and metrics

## Installation

1. Clone the repository:

    ```sh
    git clone https://github.com/lorenzo-dominici/boids-rs.git
    cd boids-rs
    ```

2. Build the project:

    ```sh
    cargo build --release
    ```

## Usage

Run the simulation with a configuration file:

```sh
cargo run --release --example rerun -- <config_file.toml>
```

### Example Configurations

The project includes several example configurations in `examples/rerun/`:

| Config File | Description |
| ----------- | ----------- |
| `test-config.toml` | Basic 3-flock setup with hawks, pigeons, and bugs |
| `mesh-config.toml` | Predator-prey setup within a mesh environment |
| `complex-config.toml` | Large-scale simulation with multiple flock types |

```sh
# Run with the mesh environment config
cargo run --release --example rerun -- examples/rerun/mesh-config.toml
```

## Mesh Environments

Boids-rs supports loading 3D mesh environments from **OBJ** and **STL** files. Boids interact with the mesh boundaries using Artificial Potential Fields (APF), which provide smooth repulsion near surfaces.

### Environment Configuration

Configure the mesh environment in your TOML file:

```toml
[environment]
path = "path/to/mesh.obj"     # Path to OBJ or STL file
inverted = true               # true = boids stay INSIDE mesh, false = stay OUTSIDE
scale = 1.0                   # Scale factor applied to the mesh

# APF Repulsion Parameters
repulsion_strength = 1.5      # Base strength of repulsion force
falloff_exponent = 2.0        # Distance falloff steepness (higher = sharper near surface)
weight_exponent = 1.0         # Multi-triangle weighting (higher = closer triangles dominate)
size_margin = 0.5             # Safety margin multiplier for boid size
max_triangles = 16            # Max triangles to consider for repulsion (8-32 typical)
steering_strength = 0.5       # Turnover steering effect (0.3-0.7 typical)
```

### How APF Repulsion Works

The environment system uses **Bounding Volume Hierarchy (BVH)** for efficient spatial queries and computes repulsion by:

1. Finding the most *significant* triangles within the boid's vision range
2. Weighting each triangle's contribution by distance and alignment
3. Computing a combined repulsion force that smoothly guides boids away from surfaces

This approach handles corners and edges gracefully, preventing boids from getting stuck.

### Mesh File Requirements

- **OBJ files**: Standard Wavefront OBJ format (automatically triangulated)
- **STL files**: Both ASCII and binary formats supported
- Meshes are automatically centered at the origin on load

## Flock Configuration

Define multiple flocks with different behaviors in your TOML configuration:

```toml
[[flocks]]
flock.name = "Predators"
flock.kind.kind = "Hawk"
flock.kind.size = 3.0              # Visual size
flock.kind.color = [255, 0, 0]     # RGB color
flock.kind.preys = ["Pigeon"]      # Kinds this flock chases
flock.kind.predators = []          # Kinds this flock flees from

# Movement parameters
flock.kind.speed.min = 1.0
flock.kind.speed.max = 2.0
flock.kind.angular_speed.min = 0.0
flock.kind.angular_speed.max = 0.05
flock.kind.acceleration.min = -0.1
flock.kind.acceleration.max = 0.1
flock.kind.momentum = 0.75

# Vision and behavior
flock.kind.vision.min = 0.0
flock.kind.vision.max = 60.0       # How far boids can see
flock.kind.protected.min = 0.0
flock.kind.protected.max = 30.0    # Personal space radius
flock.kind.sep_weight = 0.05       # Separation weight
flock.kind.align_weight = 0.05     # Alignment weight
flock.kind.coh_weight = 0.05       # Cohesion weight

# Spawn configuration
boids = 100                        # Number of boids in this flock
state.min = 0.5                    # State range (affects behavior scaling)
state.max = 1.0

# Spawn points (boids distributed across all points)
[[flocks.spawns]]
position = [0.0, 0.0, 0.0]
radius = 20.0

# Optional: Biases attract boids to specific locations
[[flocks.biases]]
position = [50.0, 0.0, 0.0]
weight_range = { min = 0.001, max = 0.005 }
prob = 0.25                        # Probability each boid gets this bias
```

### Execution Modes

Control which implementations to run:

```toml
# Options: "all", "aos_seq", "soa_seq", "aos_par", "soa_par"
execution = "soa_par"  # Recommended for best performance
```

| Mode | Description |
| ---- | ----------- |
| `aos_seq` | Array of Structures, sequential |
| `soa_seq` | Struct of Arrays, sequential |
| `aos_par` | Array of Structures, parallel (rayon) |
| `soa_par` | Struct of Arrays, parallel (rayon) - **fastest** |
| `all` | Run all four modes for comparison |

## Project Structure

```tree
boids-rs/
├── src/
│   ├── lib.rs              # Library root, core traits and types
│   ├── aos.rs              # Array of Structures implementation
│   ├── soa.rs              # Struct of Arrays implementation
│   ├── setup.rs            # Flock builder and spawn configuration
│   └── environment/
│       ├── mod.rs          # MeshEnvironment and APF repulsion
│       ├── mesh.rs         # Triangle, AABB, BVH, and MeshSdf
│       ├── sdf.rs          # SignedDistance trait and primitives
│       └── loader.rs       # OBJ/STL file loading
├── examples/
│   └── rerun/
│       ├── main.rs         # Example entry point
│       ├── setup.rs        # Config loading and context setup
│       ├── simulation.rs   # Main simulation loop and logging
│       └── assets/         # Mesh files (OBJ/STL)
├── benches/
│   └── update/             # Criterion benchmarks
├── Cargo.toml
└── README.md
```

## Visualization

Install the **rerun viewer** to visualize simulations:

```sh
# Install via cargo
cargo install rerun-cli

# Or follow instructions at:
# https://rerun.io/docs/getting-started/installing-viewer
```

> **Important**: The rerun viewer version must match the `rerun` crate version in [`Cargo.toml`](Cargo.toml).

After running a simulation, open the generated `.rrd` file with:

```sh
rerun boids_<timestamp>.rrd
```

The viewer displays:

- **3D View**: Boid positions with velocity arrows, mesh wireframe, and bounding box
- **Metrics**: Iteration timing, execution mode performance comparison
- **Logs**: Simulation configuration and environment info

## Benchmarks

Run performance benchmarks with:

```sh
cargo bench
```

Benchmarks compare AoS vs SoA implementations across different boid counts and thread configurations.

## Library Usage

Use boids-rs as a library in your own projects:

```toml
[dependencies]
boids = { path = "path/to/boids-rs" }
```

```rust
use boids::{aos, soa, setup, environment};
use boids::{MeshEnvironment, RepulsionConfig, BoidCollection};

// Load a mesh environment
let config = RepulsionConfig {
    repulsion_strength: 1.5,
    falloff_exponent: 2.0,
    weight_exponent: 1.0,
    size_margin: 0.5,
    max_triangles: 16,
    steering_strength: 0.5,
};

let env = MeshEnvironment::from_file("mesh.obj", config, true)?;

// Create and update boids
let mut boids = soa::Boids::new(&initial_boids);
boids.par_update(&boids.clone(), &env);
```

## Contributing

Contributions are welcome! Feel free to:

- Open issues for bugs or feature requests
- Submit pull requests
- Suggest improvements to documentation

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgements

This project was created as a midterm project for the course "Parallel Computing", and significantly improved as part of "Laboratory of Automation", at the University of Studies of Florence.

## References

- [rerun.io](https://rerun.io) - Visualization framework for multimodal data
- [Boids Algorithm](https://vanhunteradams.com/Pico/Animal_Movement/Boids-algorithm.html) - Detailed explanation of the original boids model
- [Original Boids Paper](https://www.red3d.com/cwr/boids/) - Craig Reynolds' original work on boids
