# Complete Jazzy Bundle Validation

Validation date: **2026-10-22**.

## Final DAE Reference Refresh (2026-10-22)

Authoritative source: `references/V6_URDF_to_Customer_final/V6_URDF_to_Customer`.
All 146 F/V left/right DAE files now match that source byte for byte;
112 differing files were replaced and the other 34 already matched. Material
colors are taken directly from the final exports, including the silver power
section. Before/after rendered comparisons were inspected for all four variants.

- All 146 STL files match the final reference byte for byte and remain unchanged.
- All 146 DAE meshes match their STL geometry within 0.2 micrometers, with
  matching triangle counts and finite vertex coordinates.
- All DAE assets use meter units and Z_UP, have no scene transforms, have
  resolvable material/effect bindings, and need no external image files.
- Existing URDF, code, 500 Hz logging configuration and separate Humble
  workspace retain their preceding values.
- The archive preserves symbolic links and is extracted for verification of
  all regenerated per-file and archive SHA256 checksums.

This asset-only refresh does not repeat the complete Jazzy build or physical
device tests. The following sections record checks of earlier source snapshots.

## Final Customer Power-Section Color Update (2026-10-22)

Compared against `V6_URDF_to_Customer/V6_URDF_to_Customer` for all four model/hand
variants. All 146 current STL files are byte-identical to that final reference.
The final reference/current DAE geometry matches STL within 0.2 micrometers.

Only the silver power-section material assignment is applied to the four
`Palm.dae` assets: 24,056 triangles per model/hand use the reference's silver
material (RGBA `0.75 0.75 0.77 1`). Geometry source buffers, normals, units,
existing materials and non-power surface colors are verified unchanged.
All other model assets and URDF files retain their previous bytes. Visual
before/after comparisons confirm the power-section color on all four variants.

The refreshed archive is extracted and its regenerated per-file and archive
SHA256 checksums are verified during packaging. This asset-color update does
not repeat the combined Jazzy build or physical-device tests described below.

## Source Refresh on 2026-10-22

The refreshed source includes the F/V left/right mesh updates and
500 Hz CSV recording with IMU fields excluded. Post-refresh checks completed:

- All 146 model/hand STL files match their respective reference exports byte for byte.
- All 146 visual DAE files match their STL geometry within 0.2 micrometers;
  existing material definitions, scenes and units are preserved.
- All four model/hand URDF trees and mesh paths validate, with 20 control joints each.
- The description package builds and installs successfully using local ROS 2 Humble.
- Five CSV logger tests pass, including 62-column output without IMU, 500 Hz
  producer input, shutdown draining, overflow and disk failure handling.
- Two SDK fast-block decoder/read-failure tests pass without hardware access.
- 32 bringup launch combinations passed during the logging integration checks.
- The refreshed ZIP and its regenerated per-file and archive SHA256 checksums
  are verified during packaging.

A full combined Jazzy build and physical-device 500 Hz measurements were not
repeated after this source refresh. The original Jazzy validation record below
applies to the preceding source snapshot.

## Original Bundle Validation (Before This Refresh)

## Environment

- Isolated Ubuntu 24.04 / ROS 2 Jazzy environment based on `osrf/ros:jazzy-desktop`.
- Only `/opt/ros/jazzy/setup.bash` was sourced before the combined build.
- All 12 source packages came from this bundle's `src/` tree.
- System dependencies were installed using `rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y`.
- No Humble installation, separate Allegro workspace, or private Isaac plugin was used.

## Dependency Corrections Included

- Corrected unresolved hardware dependency keys to `pybind11-dev` and `libmodbus-dev`.
- Declared the directly used `rclcpp`, `pkg-config`, and Python development dependencies.
- Kept `wr_topic_based_ros2_control` as a documented, separately installed requirement for optional Isaac mode. Physical and mock modes do not require it.

`rosdep install` completed successfully. `rosdep check --from-paths src --ignore-src --rosdistro jazzy` reported **All system dependencies have been satisfied**, without skipping dependency keys.

## Build and Runtime Checks

| Check | Result |
|---|---|
| Recursive discovery of five common, three V6, and four UI packages | Pass — 12 packages |
| Combined `colcon build --symlink-install --executor sequential` with `BUILD_TESTING=ON` | Pass — 12 packages |
| Bringup launch default `modbus_tcp:192.168.1.100:502` | Pass |
| Left/right V6 F and V URDFs, each with 20 control joints | Pass |
| Custom connection descriptor override | Pass |
| Built Python SDK default IP and port | Pass — `192.168.1.100:502` |
| Hardware plugin initialization and interface export without device connection | Pass |
| Mock bringup: joint-state broadcaster and position controller active | Pass |
| Mock `/joint_states` contains 20 joints | Pass |

The bundled UI source snapshot also passed 59 dashboard/vision-sensor tests during the preceding UI source-release validation. The [UI validation record](src/allegro_hand_v6_ui/docs/JAZZY_VALIDATION.md) describes display, scaling, and browser checks. This combined-workspace run focused on dependency installation, the complete build, and mock integration.

Builds may report supported legacy interface-export API deprecation warnings. These did not prevent compilation or plugin initialization.

## Scope

No physical hardware connection or motion test was performed. Real-device network configuration, camera permissions and USB throughput, native Wayland, and GPU-specific behavior require checks on the deployment PC. Optional Isaac mode and USD conversion need their separate simulator dependencies.

Git history, build/install/log outputs, Python caches, development memos, and machine-specific camera mappings are excluded from the archive. Verify the ZIP with the supplied archive checksum and extracted files with `FILES.sha256`.
