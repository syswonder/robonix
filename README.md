# Robonix

**Robonix** is an open-source embodied intelligence framework built with Rust and ROS2, implementing the EAIOS (Embodied AI Operating System) architecture.

## Architecture

Robonix follows the EAIOS architecture with four core components:

- **Task Manager**: Global scheduling and control core, responsible for task parsing, planning, and execution coordination
- **Skill Library**: Stores reusable skills that can be called at runtime
- **Service Registry**: Manages standardized algorithm capabilities (perception, planning, evaluation, verification)
- **Primitive Abstraction Layer**: Provides standardized hardware capability mapping, managing access to actuators and sensors

## Quick Start

See [robonix quickstart](rust/README.md)

## License

See LICENSE file for details.
