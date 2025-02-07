//! The simulation module.

use super::setup::{Config, Context};
use boids::{BoidCollection, SphereEnv};
use indicatif::{ProgressBar, ProgressStyle};
use rerun::{Arrows3D, Ellipsoids3D, RecordingStream, Scalar, TextLog};

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
        let env = SphereEnv::new(self.ctx.config.env_size, self.ctx.config.turnback);
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
        for i in 1..=self.ctx.config.iters {
            self.rec.set_time_sequence("iteration", i);

            // Sequential SoA
            self.ctx.soa_prev.clone_from(&self.ctx.soa_next);
            t = time(|| self.ctx.soa_next.update(&self.ctx.soa_prev, &env));
            soa_t += t;
            self.log(&self.ctx.soa_next, soa_t, t, i, "soa_seq");

            // Sequential AoS
            self.ctx.aos_prev.clone_from(&self.ctx.aos_next);
            t = time(|| self.ctx.aos_next.update(&self.ctx.aos_prev, &env));
            aos_t += t;
            self.log(&self.ctx.aos_next, aos_t, t, i, "aos_seq");

            // Parallel SoA
            self.ctx.soa_next.clone_from(&self.ctx.soa_prev);
            t = time(|| self.ctx.soa_next.par_update(&self.ctx.soa_prev, &env));
            soa_par_t += t;
            self.log(&self.ctx.soa_next, soa_par_t, t, i, "soa_par");

            // Parallel AoS
            self.ctx.aos_next.clone_from(&self.ctx.aos_prev);
            t = time(|| self.ctx.aos_next.par_update(&self.ctx.aos_prev, &env));
            aos_par_t += t;
            self.log(&self.ctx.aos_next, aos_par_t, t, i, "aos_par");

            // Decorate the environment
            self.decorate(self.ctx.config.env_size);

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

        // Print the average times for each method
        println!("======= SEQUENTIAL =======");
        println!("SoA: {:?}", soa_t / self.ctx.config.iters as f64);
        println!("AoS: {:?}", aos_t / self.ctx.config.iters as f64);
        println!("======== PARALLEL ========");
        println!("SoA: {:?}", soa_par_t / self.ctx.config.iters as f64);
        println!("AoS: {:?}", aos_par_t / self.ctx.config.iters as f64);
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

    /// Decorates the environment with bounds.
    ///
    /// # Arguments
    ///
    /// - `env_size`: The size of the environment.
    fn decorate(&self, env_size: f32) {
        let l = self.ctx.config.layout;
        self.rec
            .log(
                "/bounds",
                &Ellipsoids3D::from_radii((0..l).map(|_| env_size))
                    .with_rotation_axis_angles((0..l).map(|i| {
                        (
                            (0.0, 0.0, 1.0),
                            i as f32 * std::f32::consts::PI / (l / 2) as f32,
                        )
                    }))
                    .with_line_radii((0..l).map(|_| 0.25))
                    .with_colors((0..=l).map(|_| (129, 129, 129, 60))),
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
