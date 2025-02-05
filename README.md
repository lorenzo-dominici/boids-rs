# Boids-rs

Boids-rs is a Rust project that simulates boids behavior using [`rayon`](https://docs.rs/rayon/latest/rayon/) CPU parallelization. The project also utilizes [`rerun.io`](https://rerun.io) as a data viewer to visualize the simulation.

## Features

- Simulates boids behavior using Rust
- Parallel computation for improved performance using [`rayon`](https://docs.rs/rayon/latest/rayon/)
- Visualization of the simulation using [`rerun.io`](https://rerun.io)

## Installation

1. Clone the repository:

    ```sh
    git clone https://github.com/lorenzo-dominici/boids-rs.git
    cd boids-rs
    ```

2. Install the required dependencies:

    ```sh
    cargo build --release
    ```

## Usage

Run the simulation:

```sh
cargo run
```

## Visualization

To visualize the simulation, ensure you have ***rerun viewer*** installed and configured. Follow the instructions on the [rerun.io website](https://rerun.io/docs/getting-started/installing-viewer#installing-the-viewer) to set it up. The ***rerun viewer*** version must be the same of the [`rerun`](https://docs.rs/rerun/latest/rerun/) crate used, specified in the [`Cargo.toml`](Cargo.toml).

## Contributing

Contributions are welcome. Feel free to open issues, submit pull requests, or suggest new features.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgements

This project was created as midterm project for the course "Parallel Computing" at the University of Studies of Florence.

## References

- [Boids algorithm](https://vanhunteradams.com/Pico/Animal_Movement/Boids-algorithm.html)
- [rerun.io](https://rerun.io)
