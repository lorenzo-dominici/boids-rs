mod model;
mod setup;

use indicatif::{ProgressBar, ProgressStyle};
use model::{BoidCollection, SphereEnv};
pub use setup::Config;
use setup::Context;
use rerun::{Arrows3D, Ellipsoids3D, RecordingStream, Scalar, TextLog};

pub struct Simulation {
    config: Config,
    rec: RecordingStream,
}

impl Simulation {
    pub fn new(config: Config, rec: RecordingStream) -> Simulation {
        Simulation { config, rec }
    }

    fn init(& self) -> Context {
        Context::new(&self.config)
    }

    pub fn run(&self) {
        let mut ctx = self.init();
        let env = SphereEnv::new(ctx.env_size, ctx.turnback);
        self.rec.log("/logs", &TextLog::new("Simulation started")).unwrap();
        self.rec.log("/logs", &TextLog::new(format!("Configs: {:?}", &self.config))).unwrap();

        let mut soa_times = Vec::with_capacity(self.config.iters as usize);
        let mut aos_times = Vec::with_capacity(self.config.iters as usize);
        let mut soa_par_times = Vec::with_capacity(self.config.iters as usize);
        let mut aos_par_times = Vec::with_capacity(self.config.iters as usize);

        let mut soa_t: f64 = 0.0;
        let mut aos_t: f64 = 0.0;
        let mut soa_par_t: f64 = 0.0;
        let mut aos_par_t: f64 = 0.0;
        let mut t: f64;
        let bar = ProgressBar::new(self.config.iters as u64).with_style(
            ProgressStyle::with_template("[{elapsed_precise}] [{bar:40.cyan/blue}] {pos:>}/{len:} {eta:.green}").unwrap().progress_chars("=> ")
        );
        for i in 0..self.config.iters {
            self.rec.set_time_sequence("iteration", i);

            // Sequential SoA

            ctx.soa_prev = ctx.soa_next.clone();

            t = Self::time(|| ctx.soa_next.update(&ctx.soa_prev, &env));

            self.log(&ctx.soa_next, soa_t, t, "soa_seq");
            
            soa_times.push(t);

            soa_t += t;

            //Sequential AoS

            ctx.aos_prev = ctx.aos_next.clone();

            t = Self::time(|| ctx.aos_next.update(&ctx.aos_prev, &env));

            self.log(&ctx.aos_next, aos_t, t, "aos_seq");
            
            aos_times.push(t);
            
            aos_t += t;

            // Parallel SoA

            let mut soa_par = ctx.soa_prev.clone();

            t = Self::time(|| soa_par.par_update(&ctx.soa_prev, &env));

            self.log(&soa_par, soa_par_t, t, "soa_par");
            
            soa_par_times.push(t);

            soa_par_t += t;

            // Parallel AoS

            let mut aos_par = ctx.aos_prev.clone();

            t = Self::time(|| aos_par.par_update(&ctx.aos_prev, &env));

            self.log(&aos_par, aos_par_t, t, "aos_par");
            
            aos_par_times.push(t);
            
            aos_par_t += t;

            self.decorate(ctx.env_size);

            bar.inc(1);
        }
        bar.finish();

        println!("======= SEQUENTIAL =======");
        println!("SoA: {:?}", soa_times.iter().sum::<f64>() / soa_times.len() as f64);
        println!("AoS: {:?}", aos_times.iter().sum::<f64>() / aos_times.len() as f64);
        println!("======== PARALLEL ========");
        println!("SoA: {:?}", soa_par_times.iter().sum::<f64>() / soa_times.len() as f64);
        println!("AoS: {:?}", aos_par_times.iter().sum::<f64>() / aos_times.len() as f64);
    }

    fn time(mut f: impl FnMut() -> ()) -> f64 {
        let start = std::time::Instant::now();
        f();
        start.elapsed().as_secs_f64()
    }

    fn log<B: BoidCollection>(&self, boids: &B, seconds: f64, duration: f64, prefix: &str) {
        self.rec.set_time_seconds("seconds", seconds);
        self.rec.log(format!("/{}_duration", prefix), &Scalar::new(duration)).unwrap();
        self.rec.log(format!("/{}_boids", prefix), &Arrows3D::from_vectors(
            boids.iter().map(|(_, _, _, _, vel)| (vel.x, vel.y, vel.z))
        ).with_origins(
            boids.iter().map(|(_, _, _, pos, _)| (pos.x, pos.y, pos.z))
        ).with_colors(
            boids.iter().map(|(flock, _, _, _, _)| flock.kind.color)
        ).with_radii(
            boids.iter().map(|(flock, _, _, _, _)| flock.kind.size / 2.0)
        )).unwrap();
    }

    fn decorate(&self, env_size: f32) {
        let l = self.config.layout;
        self.rec.log("/bounds", &Ellipsoids3D::from_radii(
            (0..l).map(|_| env_size)
        ).with_rotation_axis_angles(
            (0..l).map(|i| ((0.0, 0.0, 1.0), i as f32 * std::f32::consts::PI / (l/2) as f32))
        ).with_line_radii(
            (0..l).map(|_| 0.25)
        ).with_colors(
            (0..=l).map(|_| (129, 129, 129, 60))
        )).unwrap();
    }
}