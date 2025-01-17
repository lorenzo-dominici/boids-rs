use boids_rs::{Config, Simulation};

fn main() -> Result<(), Box<dyn std::error::Error>> {

    let (config, rec) = setup()?;

    let simulation = Simulation::new(config, rec);

    simulation.run();

    Ok(())
}

fn setup() -> Result<(Config, ()), Box<dyn std::error::Error>> {
    let path = parse_args()?;
    let config = Config::load(&path)?;
    let rec = (); //TODO: Implement recording
    Ok((config, rec))
}

fn parse_args() -> Result<String, Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        return Err(format!("Usage: {} <config_path>", args[0]).into());
    }
    Ok(args[1].clone())
}