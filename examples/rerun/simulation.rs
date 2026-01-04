//! The simulation module.

use super::setup::{Config, Context, ExecutionMode};
use boids::BoidCollection;
use indicatif::{ProgressBar, ProgressStyle};
use rerun::{Arrows3D, RecordingStream, Scalar, TextLog};

/// A struct representing the simulation.
///
/// # Fields
///
/// - `ctx`: The simulation context.
/// - `rec`: The recording stream.
#[derive(Debug)]
pub struct Simulation {
    ctx: Context,
    rec: RecordingStream,
}

impl Simulation {
    /// Creates a new `Simulation` instance with the given context and recording stream.
    ///
    /// # Arguments
    ///
    /// - `ctx`: The simulation context.
    /// - `rec`: The recording stream.
    ///
    /// # Returns
    ///
    /// A new `Simulation` instance.
    pub fn new(ctx: Context, rec: RecordingStream) -> Self {
        Self { ctx, rec }
    }

    /// Creates a new `Simulation` instance from the given configuration and recording stream.
    ///
    /// # Arguments
    ///
    /// - `config`: The configuration of the simulation.
    /// - `rec`: The recording stream.
    ///
    /// # Returns
    ///
    /// A new `Simulation` instance.
    pub fn from_config(config: Config, rec: RecordingStream) -> Self {
        Self::new(Context::new(config), rec)
    }

    /// Runs the simulation.
    ///
    /// This method runs the simulation for the specified number of iterations and logs the data to the recording stream.
    pub fn run(&mut self) {
        // Log environment info
        self.log_environment_info();
        
        // Log static environment geometry (mesh if available)
        self.log_environment_geometry();

        self.rec
            .log("/logs", &TextLog::new("Simulation started"))
            .unwrap();
        self.rec
            .log(
                "/logs",
                &TextLog::new(format!("Configs: {:?}", &self.ctx.config)),
            )
            .unwrap();

        let mut soa_t: f64 = 0.0;
        let mut aos_t: f64 = 0.0;
        let mut soa_par_t: f64 = 0.0;
        let mut aos_par_t: f64 = 0.0;
        let mut t: f64;
        let bar = ProgressBar::new(self.ctx.config.iters as u64).with_style(
            ProgressStyle::with_template(
                "  [{elapsed_precise:.cyan/blue}] [{bar:25}] {pos:>}/{len:}: {eta:.green}",
            )
            .unwrap()
            .progress_chars("=> "),
        );
        bar.enable_steady_tick(std::time::Duration::from_millis(50));
        
        let env = &self.ctx.environment;
        let mode = self.ctx.config.execution;
        
        for i in 1..=self.ctx.config.iters {
            self.rec.set_time_sequence("iteration", i);

            // Sequential SoA
            if mode == ExecutionMode::All || mode == ExecutionMode::SoaSeq {
                self.ctx.soa_seq_prev.clone_from(&self.ctx.soa_seq_next);
                t = time(|| self.ctx.soa_seq_next.update(&self.ctx.soa_seq_prev, env.as_ref()));
                soa_t += t;
                self.log(&self.ctx.soa_seq_next, soa_t, t, i, "soa_seq");
            }

            // Sequential AoS
            if mode == ExecutionMode::All || mode == ExecutionMode::AosSeq {
                self.ctx.aos_seq_prev.clone_from(&self.ctx.aos_seq_next);
                t = time(|| self.ctx.aos_seq_next.update(&self.ctx.aos_seq_prev, env.as_ref()));
                aos_t += t;
                self.log(&self.ctx.aos_seq_next, aos_t, t, i, "aos_seq");
            }

            // Parallel SoA
            if mode == ExecutionMode::All || mode == ExecutionMode::SoaPar {
                self.ctx.soa_par_prev.clone_from(&self.ctx.soa_par_next);
                t = time(|| self.ctx.soa_par_next.par_update(&self.ctx.soa_par_prev, env.as_ref()));
                soa_par_t += t;
                self.log(&self.ctx.soa_par_next, soa_par_t, t, i, "soa_par");
            }

            // Parallel AoS
            if mode == ExecutionMode::All || mode == ExecutionMode::AosPar {
                self.ctx.aos_par_prev.clone_from(&self.ctx.aos_par_next);
                t = time(|| self.ctx.aos_par_next.par_update(&self.ctx.aos_par_prev, env.as_ref()));
                aos_par_t += t;
                self.log(&self.ctx.aos_par_next, aos_par_t, t, i, "aos_par");
            }

            bar.inc(1);
        }
        bar.set_message("Finished");
        bar.set_style(
            ProgressStyle::with_template(
                "{msg:>12.green.bold} simulation lasted [{elapsed_precise}]",
            )
            .unwrap(),
        );
        bar.finish();
    }

    /// Log environment information at startup
    fn log_environment_info(&self) {
        let env = &self.ctx.config.environment;
        let mesh_env = &self.ctx.mesh_env;
        let info = format!(
            "Environment: Mesh\n  Path: {}\n  Triangles: {}\n  Inverted: {}",
            env.path,
            mesh_env.mesh().triangle_count(),
            env.inverted
        );
        self.rec.log("/logs", &TextLog::new(info)).unwrap();
    }

    /// Log static environment geometry (mesh visualization)
    fn log_environment_geometry(&self) {
        let mesh_env = &self.ctx.mesh_env;
        let triangles = mesh_env.mesh().triangles();

        // Build wireframe edges for see-through visualization
        let mut edge_positions: Vec<[f32; 3]> = Vec::with_capacity(triangles.len() * 6);
        
        for tri in triangles.iter() {
            // Edge 1: v0 -> v1
            edge_positions.push([tri.v0.x, tri.v0.y, tri.v0.z]);
            edge_positions.push([tri.v1.x, tri.v1.y, tri.v1.z]);
            // Edge 2: v1 -> v2
            edge_positions.push([tri.v1.x, tri.v1.y, tri.v1.z]);
            edge_positions.push([tri.v2.x, tri.v2.y, tri.v2.z]);
            // Edge 3: v2 -> v0
            edge_positions.push([tri.v2.x, tri.v2.y, tri.v2.z]);
            edge_positions.push([tri.v0.x, tri.v0.y, tri.v0.z]);
        }

        // Log mesh as wireframe (line segments)
        self.rec
            .log_static(
                "/environment/mesh",
                &rerun::LineStrips3D::new(
                    edge_positions.chunks(2).map(|pair| vec![pair[0], pair[1]])
                )
                .with_colors([rerun::Color::from_unmultiplied_rgba(100, 140, 180, 100)])
                .with_radii([0.05]),
            )
            .unwrap();

        // Log bounding box
        let bounds = mesh_env.mesh().bounds();
        let center = bounds.center();
        let size = bounds.size();

        self.rec
            .log_static(
                "/environment/bounds",
                &rerun::Boxes3D::from_centers_and_sizes(
                    [[center.x, center.y, center.z]],
                    [[size.x, size.y, size.z]],
                )
                .with_colors([rerun::Color::from_unmultiplied_rgba(80, 80, 80, 60)]),
            )
            .unwrap();
    }

    /// Logs the boids data and other measures to the recording stream.
    ///
    /// # Arguments
    ///
    /// - `boids`: The collection of boids.
    /// - `seconds`: The total time taken to execute the function.
    /// - `duration`: The time taken to execute the function.
    /// - `iteration`: The current iteration.
    /// - `prefix`: The prefix for the log entries.
    fn log<B: BoidCollection>(
        &self,
        boids: &B,
        seconds: f64,
        duration: f64,
        iteration: u32,
        prefix: &str,
    ) {
        self.rec.set_time_seconds("seconds", seconds);
        self.rec
            .log(format!("/{}_duration", prefix), &Scalar::new(duration))
            .unwrap();
        self.rec
            .log(
                format!("/{}_average_duration", prefix),
                &Scalar::new(seconds / iteration as f64),
            )
            .unwrap();
        self.rec
            .log(
                format!("/{}_boids", prefix),
                &Arrows3D::from_vectors(
                    boids.iter().map(|(_, _, _, _, vel)| (vel.x, vel.y, vel.z)),
                )
                .with_origins(boids.iter().map(|(_, _, _, pos, _)| (pos.x, pos.y, pos.z)))
                .with_colors(boids.iter().map(|(flock, _, _, _, _)| flock.kind.color))
                .with_radii(
                    boids
                        .iter()
                        .map(|(flock, _, _, _, _)| flock.kind.size / 2.0),
                ),
            )
            .unwrap();
    }
}

/// Measure the time taken to execute a function.
///
/// # Arguments
///
/// - `f`: The function to execute.
///
/// # Returns
///
/// The time taken to execute the function in seconds.
pub fn time(mut f: impl FnMut()) -> f64 {
    let start = std::time::Instant::now();
    f();
    start.elapsed().as_secs_f64()
}
