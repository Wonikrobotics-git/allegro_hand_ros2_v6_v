# Jazzy and Display Validation

Validation date: **2026-10-22**.

## Environment and Scope

The UI was built and exercised in an isolated `osrf/ros:jazzy-desktop` container:

| Component | Version/environment |
|---|---|
| OS | Ubuntu 24.04.4 LTS |
| ROS 2 | Jazzy |
| `rclcpp` | 28.1.18 |
| `rviz_common` | 14.1.20 |
| Qt Widgets | 5.15.13 |
| OpenCV | 4.6.0 |
| GUI backend | X11/Xvfb with software OpenGL |

Only Jazzy was sourced for the container build. The separately supplied `allegro_hand_v6_description` package provided the V6 model assets. No Humble headers or libraries were used. Hardware-interface packages were outside this UI verification scope.

## Build, Dependencies, and Tests

- All four UI packages built successfully with Jazzy: dashboard, pose GUI, web plot, and vision sensor.
- The V6 description package also built successfully with Jazzy.
- `rosdep check` reported that all system dependencies were satisfied after the installed description environment was sourced.
- Dashboard and vision-sensor tests completed successfully. `colcon test-result` reported **59 tests, 0 errors, 0 failures, and 0 skipped**.
- A separate GUI verification executable instantiated the actual dashboard and pose-control window classes against the built Jazzy libraries. RViz loaded the V6 URDF and its link properties; screenshots also confirmed native 3D rendering.
- A supplementary build of the changed dashboard and pose GUI succeeded on the developer PC's Humble installation. This does not change the Jazzy release target.

## Desktop Resolution and Scaling Matrix

Qt uses logical coordinates after display scaling. The application initially fits its preferred window size within the available screen; these checks did not force a fixed 1850×1000 window.

| Screen pixels | Scale | Qt logical screen | Initial dashboard layout | Result |
|---|---|---|---|---|
| 1280×720 | 100% | 1280×720 | Side by side | Pass |
| 1366×768 | 100% | 1366×768 | Side by side | Pass |
| 1920×1080 | 100% | 1920×1080 | Side by side | Pass |
| 2560×1080 | 100% | 2560×1080 | Side by side | Pass |
| 1200×1600 | 100% | 1200×1600 | Panel tabs | Pass |
| 1920×1080 | 150% | 1280×720 | Side by side | Pass |
| 3840×2160 | 200% | 1920×1080 | Side by side | Pass |
| 1280×720 | 150% | Approximately 853×480 | Panel tabs | Pass |

For each configuration, the checks covered:

1. Dashboard and pose-control startup windows fitting within the available screen.
2. A loaded RViz robot model and usable 3D/sensor panels.
3. Switching compact-mode tabs without recreating or reparenting the native RViz rendering window.
4. Resizing into compact mode and recovering the side-by-side layout when sufficient space was available.
5. Vertical pose/sequence layout in narrow windows.
6. Reaching Servo ON, Save Current, Start, and Clear Log through the pose GUI's scroll area without clicking or issuing motion commands.

The sensor canvas uses a uniform scale, and camera images use `Qt::KeepAspectRatio`. Five synthetic 640×480 camera images were additionally rendered in sensor cards at 100% and 150% scaling. Pixel-bound checks verified the 4:3 image ratio within raster-rounding tolerance.

## Browser Monitor Checks

The web page was tested with an isolated headless Chrome profile. The following actual CSS viewports passed checks for horizontal document overflow, header bounds, and chart bounds:

| CSS viewport | Device pixel ratio | Result |
|---|---|---|
| 800×513 | 1.0 | Pass |
| 1920×993 | 1.0 | Pass |
| 800×1113 | 1.0 | Pass |
| 1280×633 | 1.5 | Pass |

The installed Jazzy web bridge was also exercised separately: its HTTP server returned the dashboard page on port 8080, and its WebSocket server on port 9091 delivered synthetic ROS messages containing 20 joint values and 18 tactile values. The browser layout checks and bridge data checks were independent; a live browser-to-hardware session was outside this validation.

## Interpretation and Limits

The results support Jazzy software compatibility and the listed desktop display configurations. The dashboard keeps the model and camera aspect ratios when the window changes shape; compact views intentionally show one panel at a time. The pose GUI intentionally allows scrolling instead of cutting off controls.

The tested desktop range extends down to approximately 853×480 logical pixels. The applications have a 480×400 logical minimum window size; displays with less available logical space are outside this validation. Physical USB capture throughput, hardware motion, native Wayland rendering, and GPU-driver-specific behavior were not tested. Synthetic joint states, pressure messages, and camera images were used for the GUI checks.

Follow the [repository installation instructions](../README.md) with a matching Jazzy base release when reproducing the application setup. On each deployment PC, verify its camera permissions, base controller interfaces, and display backend separately.
