# airborne-oxide

Supercharging ArduPilot: A Rust-powered Flight Controller with Mavlink Integration

## Description

airborne-oxide is an ambitious Rust program aimed at creating a high-performance, safety-critical flight controller. By leveraging Rust's performance and memory safety features, we're building a robust alternative to traditional C/C++ implementations. This project integrates with ArduPilot's ecosystem and implements Mavlink protocol support, bridging proven open-source autopilot software with Rust's modern programming paradigms.

## Current Features

- Rust-based implementation for improved performance and memory safety
- ArduPilot compatibility layer
- Basic Mavlink protocol integration
- Real-time scheduling for critical flight control loops
- Sensor data acquisition and fusion (IMU, GPS, Barometer)
- Simple PID controller for stabilization

## Roadmap

### Phase 1: Foundation (Current)
- [x] Basic project structure
- [x] ArduPilot HAL (Hardware Abstraction Layer) in Rust
- [x] Mavlink message parsing and generation
- [ ] Core flight control loops

### Phase 2: Advanced Flight Features
- [ ] Advanced flight modes (Loiter, RTL, Auto)
- [ ] Obstacle avoidance using computer vision
- [ ] Path planning algorithms
- [ ] Geofencing capabilities

### Phase 3: Ecosystem Integration
- [ ] Full ArduPilot parameter system compatibility
- [ ] Custom Ground Control Station in Rust
- [ ] Simulation environment for testing
- [ ] CI/CD pipeline for automated testing on various hardware

### Phase 4: Optimization and Expansion
- [ ] WASM module for in-browser flight control customization
- [ ] Machine learning integration for adaptive control
- [ ] Support for various flight platforms (multirotor, fixed-wing, VTOL)

## Installation

```bash
# Clone the repository
git clone https://github.com/yourusername/airborne-oxide.git

# Change into the project directory
cd airborne-oxide

# Build the project
cargo build --release

# Run tests
cargo test
```

## Usage

As airborne-oxide is currently in the early stages of development, there are no functional components available for end-users at this time. However, here's what you can do:
For Developers

Explore the codebase to understand the project structure.
Check the docs folder for design documents and architectural plans.
Look for TODO comments in the code for areas that need work.

## Contributing

We welcome contributions to airborne-oxide! Here's how you can help:

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

Please ensure your code adheres to our style guide and passes all tests.

## Contact

Daniel Biocchi - daniel@biocchi.ca

Project Link: [https://github.com/danbiocchi/airborne-oxide](https://github.com/danbiocchi/airborne-oxide)


## Getting Involved

While we're not ready for flight tests, we welcome contributions in:

1. Rust implementation of ArduPilot HAL
2. Mavlink protocol integration
3. Flight dynamics modeling
4. Safety-critical RTOS design in Rust


Let's redefine flight control systems with the power of Rust! 🦀✈️

## Acknowledgements

- ArduPilot Community
- Mavlink Protocol
- Rust Embedded Community


## License

This project is licensed under the [Apache License 2.0].
