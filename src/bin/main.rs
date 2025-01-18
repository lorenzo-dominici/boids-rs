use time::{OffsetDateTime, format_description};
use boids::{Config, Simulation};
use rerun::{self, RecordingStream};

fn main() -> Result<(), Box<dyn std::error::Error>> {

    let (config, rec) = setup()?;

    let simulation = Simulation::new(config, rec);

    simulation.run();

    Ok(())
}

fn setup() -> Result<(Config, RecordingStream), Box<dyn std::error::Error>> {
    let path = parse_args()?;
    let config = Config::load(&path)?;
    let rec = rerun::RecordingStreamBuilder::new("boids")
        .save(format!("boids_{}.rrd", OffsetDateTime::now_utc().format(&format_description::parse("[year]-[month]-[day]_[hour]-[minute]-[second]")?)?))?;
    Ok((config, rec))
}

fn parse_args() -> Result<String, Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        return Err(format!("Usage: {} <config_path>", args[0]).into());
    }
    Ok(args[1].clone())
}