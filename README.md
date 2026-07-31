# robot-pingpong

> robot table tennis machine (Rust)

Rust rewrite of the original C++ stereo-vision table-tennis robot controller.

## Architecture

```
Cameras → Vision (mask / DLT / triangulation)
       → Predictor (Kalman + Y/Z trajectory)
       → LinearMotor (table Y) + Dynamixel Arm (Z / hit)
       → Visualizer
```

## Build

```bash
# Library + unit tests (no cameras/motors required for tests)
cargo test

# Dry-run binary (no hardware)
cargo run --features dry-run

# Full hardware binary
cargo run --release
```

### Features

| Feature | Purpose |
|---------|---------|
| `dry-run` | Skip cameras/motors; exercise predictor path only |
| `ajinextek` | Enable AJINEXTEK AXL/AXM linear-motor bindings (Windows) |

### Dependencies

- OpenCV 4.x (`libopencv-dev`)
- Dynamixel Protocol 2 bus on `PORT_NAME` (`/dev/ttyUSB0` on Linux, `COM8` on Windows)
- Optional AJINEXTEK motion library for the linear axis

### Calibration files

Interactive calibration writes:

- `mask.yml` — per-camera polygon masks
- `points.yml` — six table landmarks per camera

## Modules

- `vision` — capture, orange-ball segmentation, stereo DLT, visualization
- `predictor` — Kalman filter, linear Y / quadratic Z prediction, hit timing
- `dynamixel` — Protocol 2 serial driver for MX-28 / MX-64
- `control` — arm inverse kinematics + linear slide axis
