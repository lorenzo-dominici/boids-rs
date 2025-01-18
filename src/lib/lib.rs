mod model;
mod setup;

pub use setup::Config;
use setup::Context;
use rayon::prelude::*;
use rerun::{Arrows3D, Ellipsoids3D, RecordingStream, Scalar, TextLog};



pub struct Simulation {
    config: Config,
    rec: RecordingStream,// TODO: Implement recording
}

impl Simulation {
    pub fn new(config: Config, rec: RecordingStream) -> Simulation {
        Simulation { config, rec }
    }

    pub fn run(&self) {
        let mut ctx = self.init();
        self.rec.log("/logs", &TextLog::new("Simulation started")).unwrap();
        self.rec.log("/logs", &TextLog::new(format!("Configs: {:?}", &self.config))).unwrap();

        let mut n: u32 = 0;
        let mut t: f64 = 0.0;
        let mut iter = self.config.iters;
        while iter > 0 {
            self.rec.set_time_sequence("iteration", n);
            self.rec.set_time_seconds("seconds", t);
            let start = std::time::Instant::now();

            ctx.prev = ctx.next.clone();
            ctx.next.par_iter_mut().for_each(|boid| {
                boid.update(&ctx.prev, |pos| model::sphere_env(pos, ctx.env_size, ctx.turnback));
            });

            let duration = start.elapsed();
            self.rec.log("/duration", &Scalar::new((duration.as_nanos() as f64) / 1e9)).unwrap();
            self.rec.log("/boids", &Arrows3D::from_vectors(
                ctx.next.iter().map(|boid| (boid.vel.x, boid.vel.y, boid.vel.z))
            ).with_origins(
                ctx.next.iter().map(|boid| (boid.pos.x, boid.pos.y, boid.pos.z))
            ).with_colors(
                ctx.next.iter().map(|boid| boid.flock.kind.color)
            ).with_radii(
                ctx.next.iter().map(|boid| boid.flock.kind.size / 2.0))
            ).unwrap();

            let l = self.config.layout;

            self.rec.log("/bounds", &Ellipsoids3D::from_radii(
                (0..l).map(|_| ctx.env_size)
            ).with_rotation_axis_angles(
                (0..l).map(|i| ((0.0, 0.0, 1.0), i as f32 * std::f32::consts::PI / (l/2) as f32))
            ).with_line_radii(
                (0..l).map(|_| 0.25)
            ).with_colors(
                (0..=l).map(|_| (129, 129, 129, 60))
            )).unwrap();

            t += (duration.as_micros() as f64) / 1e6;
            n += 1;
            
            iter -= 1;
        }
    }

    fn init(&self) -> Context {
        Context::new(&self.config)
    }
}