# Changelog

Release versions follow semantic versioning (MAJOR.MINOR.PATCH) and use Git tags with a `v` prefix.

## [1.0.0] - 2026-10-26

Initial GitHub release of the Allegro Hand V6 V software for Ubuntu 24.04 and ROS 2 Jazzy, based on the complete Jazzy source bundle dated 2026-10-22.

- Include the common libraries, V6 hardware driver, robot descriptions, controller bringup, and UI applications in one workspace.
- Document V-model installation and operation, including camera-to-finger mapping before opening the dashboard.
- Use F and V for model names and retain the shared F/V software and model assets.
- Include optional CSV recording with a configured 500 Hz control rate.
- Set package and Python distribution versions to 1.0.0.

The original software and model validation scope is recorded in [VALIDATION.md](VALIDATION.md). This release adds documentation and version metadata; physical hardware testing was not repeated.
