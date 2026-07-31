# Rust Rewrite Implementation Plan

> **For agentic workers:** Implemented inline in the cloud-agent session.

**Goal:** Rewrite the C++ robot table-tennis controller in Rust with equivalent control loop and algorithms.

**Architecture:** Module layout mirrors C++ (`vision`, `predictor`, `dynamixel`, `control`, `utils`). Dynamixel Protocol 2 is implemented in pure Rust; LinearMotor uses a stub unless `ajinextek` is enabled.

**Tech Stack:** Rust 2021, OpenCV (`opencv` crate), nalgebra, serialport, anyhow.

## Global Constraints

- Preserve coordinate conventions and prediction formulas from the C++ code
- Keep `mask.yml` / `points.yml` calibration workflow
- Unit tests must pass without cameras/motors (`cargo test --features dry-run`)

---

### Task 1: Scaffold + pure algorithms

- [x] Cargo project, constants, regression, timer, predictor, arm IK, Dynamixel CRC
- [x] Unit tests for regression / IK / CRC / predictor

### Task 2: Hardware + vision port

- [x] Dynamixel bus/motor wrappers
- [x] LinearMotor stub + ajinextek feature hook
- [x] Capture / Tracker / DLT / Visualizer / Vision / main loop

### Task 3: Verify

- [x] `cargo test --features dry-run`
- [x] `cargo build` (full binary compile)
