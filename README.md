# ble_mouse

Joystick-controlled **Bluetooth Low Energy (BLE)** wireless mouse implemented in **embedded Rust**.

## Overview
- **Joystick-to-BLE HID Control:** Reads joystick input and translates movement/actions into BLE HID mouse reports.
- **Embedded Rust Stack:** Built with `Cargo`, `build.rs`, `rust-toolchain.toml`, and `clippy.toml` for reproducible builds and lint checks.
- **Clean Project Layout:** Organizes application logic under `src/` and Cargo/target settings under `.cargo/`.

## Repository Structure
- `src/` — Embedded Rust application source code
- `.cargo/` — Cargo configuration and target/runner settings
- `Cargo.toml` & `Cargo.lock` — Crate dependencies and locked versions
- `build.rs` — Build script configuration
- `rust-toolchain.toml` — Pinned Rust toolchain and targets
- `clippy.toml` — Clippy lint configuration
- `.gitignore` — Ignored build/target artifacts

## Build & Flash
1. Ensure the toolchain specified in `rust-toolchain.toml` is installed via `rustup`.
2. Check lints and build the project:
   ```bash
   cargo clippy
   cargo build --release
