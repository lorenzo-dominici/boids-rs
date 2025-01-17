mod model;
mod setup;

pub use setup::Config;
use setup::Context;
use rayon::prelude::*;
use rerun::{Color, Position3D, Radius, RecordingStream, Scalar, TextLog};



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
        loop {
            self.rec.set_time_sequence("iteration", n);
            self.rec.set_time_seconds("seconds", t);
            let start = std::time::Instant::now();

            ctx.prev = ctx.next.clone();
            ctx.next.par_iter_mut().for_each(|boid| {
                boid.update(&ctx.prev, |pos| model::sphere_env(pos, ctx.env_size));
            });

            let duration = start.elapsed();
            self.rec.log("/duration", &Scalar::new((duration.as_nanos() as f64) / 1e9)).unwrap();

            let mut i: u32 = 0;
            ctx.next.iter().map(|boid| (Position3D::new(boid.pos.x, boid.pos.y, boid.pos.z), Color::new(boid.flock.kind.color), Radius::new_scene_units(boid.flock.kind.size))).for_each(|(pos, col, rad)| {
                self.rec.log(format!("/boids/boid[{}]", i), &pos).unwrap();
                self.rec.log(format!("/boids/boid[{}]", i), &col).unwrap();
                self.rec.log(format!("/boids/boid[{}]", i), &rad).unwrap();
                i += 1;
            });

            t += (duration.as_micros() as f64) / 1e6;
            n += 1;
        }
    }

    fn init(&self) -> Context {
        Context::new(&self.config)
    }
}