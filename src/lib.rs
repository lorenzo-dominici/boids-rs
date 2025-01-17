mod model;
mod setup;

use pretty_env_logger;
use log::info;
pub use setup::Config;
use setup::Context;
use rayon::prelude::*;



pub struct Simulation {
    config: Config,
    rec: (),// TODO: Implement recording
}

impl Simulation {
    pub fn new(config: Config, rec: ()) -> Simulation {
        Simulation { config, rec }
    }

    pub fn run(&self) {
        let mut ctx = self.init();
        info!("Simulation started");
        info!("Config: {:?}", self.config);

        loop {
            ctx.prev = ctx.next.clone();
            ctx.next.par_iter_mut().for_each(|boid| {
                boid.update(&ctx.prev, |pos| model::sphere_env(pos, ctx.env_size));
            });

            //TODO: Implement recording
        }
    }

    fn init(&self) -> Context {
        pretty_env_logger::init();
        Context::new(&self.config)
    }
}