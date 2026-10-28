# Allegro Hand V6 UI — Jazzy Customer Source Distribution

Release date: **2026-10-22**.

> Historical record of the earlier UI-only ZIP distribution. For the current combined F/V GitHub repository, use the [root installation guide](../../README.md#installation) and [current UI guide](README.md). The separate-workspace/archive instructions below are not the installation procedure for this repository.

## Platform and Installation

- Ubuntu 24.04 LTS and ROS 2 Jazzy.
- Distribution root: `allegro_hand_v6_ui/`.
- Install and build the separately supplied Jazzy base V6 release first, including `allegro_hand_common`, `allegro_hand_v6_bringup`, `allegro_hand_v6_description`, and `allegro_hand_v6_hardware`.
- Extract this distribution under a separate UI workspace's `src/` directory. Keep the four packages together inside `src/allegro_hand_v6_ui/`.
- Follow [README.md](README.md) for dependency installation, build commands, environment loading, and application launch instructions.

This is a source distribution. Build it on the customer's Jazzy PC; prebuilt executables and a ROS installation are not included.

## Included Packages

| Package | Purpose |
|---|---|
| `allegro_hand_v6_dashboard` | Qt dashboard, embedded RViz, tactile markers, and direct camera views |
| `allegro_hand_v6_gui` | Pose editing, saved poses, sequences, and servo requests |
| `allegro_hand_v6_web_plot` | Browser monitoring with HTTP and WebSocket bridge |
| `allegro_hand_vision_sensor` | Camera discovery, image capture, and finger mapping |

Package names and ROS launch commands retain their existing names. The distribution directory name does not change the ROS interfaces.

## Configuration and Changes

- Jazzy installation instructions and independent base/UI workspace setup.
- Documented Modbus TCP endpoint: **192.168.1.100:502**. The base hardware stack controls the connection; the physical-hand example passes `io_interface_descriptor:=modbus_tcp:192.168.1.100:502` explicitly.
- Current left/right V6 F/V robot-frame support, pressure arrow placement, fingertip markers, and closer default camera framing.
- Window title **V6 Dashboard**.
- Dashboard tabs for compact/portrait windows, pose GUI scrolling, and high-DPI display support.
- Synthetic mapping tests use a preshape speed compatible with their 30 ms feedback period; runtime motion settings and tracking-error limits are unchanged.

Camera assignments must be generated for the customer's physical hand. Only the empty `finger_serials.example.yaml` schema is included. Automatic mapping moves real joints; review the mapping instructions and motion configuration before use.

## Verification

The [Jazzy and display validation record](docs/JAZZY_VALIDATION.md) documents successful UI builds, 59 tests, eight desktop resolution/scale combinations, four browser viewport configurations, and synthetic HTTP/WebSocket data checks. Physical hand operation, USB capture throughput, native Wayland, and GPU-specific behavior remain deployment checks.

The customer ZIP was additionally extracted and checked for source checksums, configuration symlinks, and discovery of all four packages under `allegro_hand_v6_ui/`. All four UI packages built with Jazzy using `--symlink-install`; dashboard and vision-sensor tests reported **59 tests, 0 errors, 0 failures, and 0 skipped**.

`FILES.sha256` in the extracted distribution verifies the shipped source files. The separate `SHA256SUMS` file verifies the ZIP archive before extraction. Git history, build/install/log directories, Python caches, and machine-specific camera mappings are excluded from the customer archive. Existing package license and third-party notice files are retained.
