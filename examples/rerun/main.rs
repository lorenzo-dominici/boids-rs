//! # Rerun Example
//!
//! This example demonstrates how to use the `rerun` crate to record and replay the simulation.

mod setup;
mod simulation;

use rerun::{RecordingStream, RecordingStreamBuilder};
use setup::Config;
use simulation::Simulation;
use time::{format_description, OffsetDateTime};

/// The main function of the program.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Setup the configuration and recording stream
    let (config, rec) = setup()?;

    // Create a new simulation with the given configuration and recording stream
    let mut simulation = Simulation::from_config(config, rec);

    // Run the simulation
    simulation.run();

    Ok(())
}

/// Function to setup the configuration and recording stream
fn setup() -> Result<(Config, RecordingStream), Box<dyn std::error::Error>> {
    // Parse the command line arguments to get the config file path
    let path = parse_args()?;

    // Load the configuration from the specified path
    let config = Config::load(&path)?;

    // Create a new recording stream with a timestamped filename
    let rec = RecordingStreamBuilder::new("boids").save(format!(
        "boids_{}.rrd",
        OffsetDateTime::now_utc().format(&format_description::parse(
            "[year]-[month]-[day]_[hour]-[minute]-[second]"
        )?)?
    ))?;

    Ok((config, rec))
}

/// Function to parse command line arguments and return the config file path
fn parse_args() -> Result<String, Box<dyn std::error::Error>> {
    // Collect the command line arguments into a vector
    let args: Vec<String> = std::env::args().collect();

    // Check if the number of arguments is less than 2
    if args.len() < 2 {
        // Return an error if the config file path is not provided
        return Err(format!("Usage: {} <config_path>", args[0]).into());
    }

    // Return the config file path
    Ok(args[1].clone())
}
