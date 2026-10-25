# Allegro Hand V6 ROS 2 UI Suite

`allegro_hand_v6_ui` provides desktop visualization, pose control, browser monitoring, and vision-sensor tools for the five-finger, 20-joint **Allegro Hand V6**.

This repository is released separately from the base V6 packages. Install and build the base release first, then build this repository in a separate ROS 2 workspace. The UI applications connect to the running V6 stack through ROS 2 topics, services, and TF. The hardware driver, robot descriptions, and bringup files remain in the base release.

The customer source distribution is named **allegro_hand_v6_ui** and targets **Ubuntu 24.04 / ROS 2 Jazzy**. See [RELEASE_NOTES.md](RELEASE_NOTES.md) for delivery contents and validation scope.

---

## Directory and Package Structure

```text
allegro_hand_v6_ui/
├── README.md
├── RELEASE_NOTES.md                # Customer release contents and requirements
├── docs/                           # Jazzy compatibility and display validation record
├── allegro_hand_v6_dashboard/       # Integrated Qt dashboard with embedded RViz
│   ├── assets/                    # Left/right hand images
│   ├── config/                    # Pressure-channel layouts and marker settings
│   ├── include/                   # Dashboard, sensor layout, and capture interfaces
│   ├── launch/                    # dashboard.launch.py
│   ├── src/                       # Qt panels, RViz integration, direct camera capture
│   └── test/                      # Layout, marker placement, and frame-correction tests
├── allegro_hand_v6_gui/             # Joint pose and servo command panel
│   ├── include/                   # Main window interface
│   ├── poses/                     # Example and saved pose YAML files
│   └── src/                       # Pose editing, sequences, and ROS command publishing
├── allegro_hand_v6_web_plot/        # Browser-based joint and sensor monitor
│   ├── scripts/                   # ROS-to-WebSocket bridge and HTTP server
│   └── web/                       # Browser charts and optional logo
└── allegro_hand_vision_sensor/      # Camera discovery, capture, and finger mapping
    ├── allegro_hand_vision_sensor/ # Python capture, mapping, and transport modules
    ├── config/                    # Motion templates and example serial mapping
    ├── docs/                      # Camera backend documentation
    ├── launch/                    # Capture and automatic mapping entry points
    ├── resource/                  # ament package registration
    └── test/                      # Capture, mapping, and synthetic pipeline tests
```

Each package has its own `package.xml`, build configuration, and README.

| Package | Role | Entry point |
|---|---|---|
| `allegro_hand_v6_dashboard` | Displays the live 3D hand, tactile markers, pressure readings, and five camera views in one **V6 Dashboard** window. Opens cameras directly. | `ros2 launch allegro_hand_v6_dashboard dashboard.launch.py` |
| `allegro_hand_v6_gui` | Saves and sends 20-joint poses, runs pose sequences, and publishes servo requests. | `ros2 run allegro_hand_v6_gui allegro_hand_v6_gui_node` |
| `allegro_hand_v6_web_plot` | Plots joint positions, commanded positions, effort, temperature, and tactile data in a browser when the corresponding inputs are available. | `ros2 run allegro_hand_v6_web_plot web_plot_bridge` |
| `allegro_hand_vision_sensor` | Discovers USB cameras, publishes images, and maps camera serial numbers to fingers. | `vision_sensor.launch.py` / `mapping.launch.py` |

## Prerequisites

The release target is **Ubuntu 24.04 and ROS 2 Jazzy**. The commands below use Jazzy. Use base and UI releases intended for the same ROS distribution; do not source a Humble-built base installation in a Jazzy terminal.

Prepare the following before building the UI:

- A working installation of the separately released `allegro_hand_common` and `allegro_hand_v6` packages, including `allegro_hand_v6_bringup`, `allegro_hand_v6_description`, and `allegro_hand_v6_hardware`.
- ROS 2 development tools, `colcon`, and initialized `rosdep`.
- A graphical desktop for the Qt/RViz applications.
- USB vision sensors and permission to access their video devices when camera features are required.

The package manifests declare Qt 5, RViz, OpenCV, libudev, yaml-cpp, and Python dependencies. Camera capture uses OpenCV/V4L2 and udev; the external `digit-interface` SDK is not required.

## Installation and Build

### 1. Prepare the Base V6 Workspace

Download the base packages and follow their release instructions to install dependencies and build them. This README assumes their workspace is `~/allegro_v6_ws` and its environment file is `~/allegro_v6_ws/install/setup.bash`. Replace that path if your base installation is elsewhere.

Expected workspace arrangement:

```text
~/allegro_v6_ws/                    # Base release: built independently
├── src/
│   ├── allegro_hand_common/
│   └── allegro_hand_v6/
└── install/setup.bash

~/v6_ui_ws/                         # UI release: separate workspace
└── src/
    └── allegro_hand_v6_ui/                 # This repository, including all four packages
```

The repository can stay nested under `src/allegro_hand_v6_ui`; its packages do not need to be moved individually into `src/`.

### 2. Build the UI Workspace

Clone this repository using the release's repository URL, or extract its release archive, into `~/v6_ui_ws/src/allegro_hand_v6_ui`. In a new terminal:

For the customer ZIP distribution, run the following from the directory containing the downloaded archive and `SHA256SUMS`:

```bash
sha256sum -c SHA256SUMS
mkdir -p ~/v6_ui_ws/src
unzip allegro_hand_v6_ui_jazzy_20261022.zip -d ~/v6_ui_ws/src
```

The archive already contains the top-level `allegro_hand_v6_ui/` directory. Verify the extracted source files if required:

```bash
cd ~/v6_ui_ws/src/allegro_hand_v6_ui
sha256sum -c FILES.sha256
```

Then install dependencies and build:

```bash
source /opt/ros/jazzy/setup.bash
source ~/allegro_v6_ws/install/setup.bash

cd ~/v6_ui_ws
rosdep install --from-paths src/allegro_hand_v6_ui \
  --ignore-src --rosdistro jazzy -r -y

colcon build --symlink-install --packages-select \
  allegro_hand_vision_sensor \
  allegro_hand_v6_dashboard \
  allegro_hand_v6_gui \
  allegro_hand_v6_web_plot

source install/setup.bash
```

If `rosdep` has never been initialized on the PC, run `sudo rosdep init` once, followed by `rosdep update`, before dependency installation.

The base workspace supplies the V6 dependencies as an **underlay**. The UI workspace is an **overlay**; building it does not require copying the base packages into this repository. Installed assets are located through the ament package index, without a fixed source path or a V5 reference checkout.

### 3. Load the Environment in Each Terminal

Use this order in every terminal that starts a base or UI application:

```bash
source /opt/ros/jazzy/setup.bash
source ~/allegro_v6_ws/install/setup.bash
source ~/v6_ui_ws/install/setup.bash
```

Use the same `ROS_DOMAIN_ID` and compatible ROS middleware settings for all processes. The examples below assume a single hand on one PC.

## Quick Start: Dashboard Without Hardware

After loading the environment in both terminals:

```bash
# Terminal 1: base V6 bringup with mock joint feedback
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  model:=v6_v hand:=left \
  ros2_control_hardware_type:=mock_components gui:=false
```

```bash
# Terminal 2: independently launched UI
ros2 launch allegro_hand_v6_dashboard dashboard.launch.py hand:=left
```

The 3D and 2D left-hand views should appear. Physical pressure readings and camera images require connected devices; mock bringup verifies model and joint-state integration.

The dashboard's `hand` argument selects its 2D image and sensor layout. The **3D model comes from the base bringup's `/robot_description` and TF**, so specify the same hand in both commands. Use `hand:=right` in both terminals for a right hand. Select `model:=v6_f` in bringup for the F model, or `model:=v6_v` for the V model.

Stop any previous bringup or robot-state publisher before starting another publisher on the same model and TF topics.

## Run with a Physical Hand

Start the base stack first, using the model, side, and communication address of the connected hand:

```bash
# Terminal 1: activates the physical hardware interface
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  model:=v6_v hand:=left \
  ros2_control_hardware_type:=hardware \
  io_interface_descriptor:=modbus_tcp:192.168.1.100:502 \
  gui:=false
```

```bash
# Terminal 2: monitoring dashboard
ros2 launch allegro_hand_v6_dashboard dashboard.launch.py hand:=left
```

The address above is an example; use the descriptor supported by your base release and physical connection. Start the dashboard after the base controller is active. The UI does not configure the hand's network interface or replace the hardware driver.

### Dashboard Configuration

| Argument | Default | Purpose |
|---|---|---|
| `hand` | `left` | Selects the left/right 2D image and layout. |
| `pressure_topic` | `/allegro_hand/tactile_pressures` | `Float64MultiArray` tactile input. |
| `robot_description_topic` | `/robot_description` | Robot description published by the base stack. |
| `fixed_frame` | `world` | RViz reference frame; must exist in TF. |
| `mapping_file` | User configuration path below | Camera serial-to-finger mapping. |
| `sensor_config` / `image_path` | Assets selected by `hand` | Custom layout/image; relative paths start at the dashboard package's share directory. |
| `view_distance` | `0.70` | Orbit-camera distance; smaller values bring the hand closer. |
| `view_focal_z` | `0.11` | Orbit-camera focal height. |

For example:

```bash
ros2 launch allegro_hand_v6_dashboard dashboard.launch.py \
  hand:=left view_distance:=0.60
```

The default tactile input contains 18 channels: three per finger in thumb/index/middle/ring/pinky order, followed by three palm channels. Palm wire indices 15, 16, and 17 represent the upper, middle, and bottom sensors. Use `pressure_channel_count` and `pressure_indices` when your base release supplies a different palm layout.

The dashboard attaches pressure markers to the received model's sensor/link frames. Arrows extend outward from the contact surfaces; fingertip spheres are centered on the fingertip sensor frames. The status line identifies the received model and hand and reports missing, legacy, or mismatched descriptions.

## Window Size, Aspect Ratio, and Display Scaling

The dashboard starts within the primary screen's available area. Wide windows show the 3D and sensor panels side by side. Compact or portrait-shaped windows provide **3D Hand** and **Sensors** tabs; changing tabs keeps the existing RViz scene and subscriptions. Enlarging the window restores the side-by-side layout.

The hand canvas and camera images keep their aspect ratios. The sensor heading adapts to the available width, and Qt high-DPI scaling is enabled for the desktop applications. In the pose GUI, narrow windows stack the pose and sequence panels vertically; scrolling keeps the controls accessible when the content is taller than the window.

Jazzy validation covered eight screen/scale combinations, including 1280×720, FHD, ultrawide, portrait, and 4K at 200%. See the [Jazzy and display validation record](docs/JAZZY_VALIDATION.md) for exact configurations, results, and scope. Screen scaling is normally controlled through the desktop settings; no application-specific scale setting is required.

## Camera Capture and Finger Mapping

The dashboard captures camera images directly. **Stop standalone capture and mapping processes before opening the dashboard**, so two applications do not open the same cameras.

To inspect cameras independently:

```bash
ros2 launch allegro_hand_vision_sensor vision_sensor.launch.py
```

This publishes detected serials on `/vision_sensor/connected_serials` and images on `/vision_sensor/s_<serial UTF-8 hex>/image_raw`. Stop capture with `Ctrl+C` before proceeding to the dashboard.

For a physical hand that has not been mapped:

1. Connect the hand and its cameras, start physical bringup, and check that the position controller is active.
2. Review the installed `config/mapping_motion.yaml` or prepare a custom motion file. **Automatic mapping publishes real joint commands and moves the hand through contact poses.**
3. With the dashboard closed, run:

   ```bash
   ros2 launch allegro_hand_vision_sensor mapping.launch.py hand:=left
   ```

4. The launch starts camera capture, then guarded-sweep mapping after a default five-second delay. Wait for completion and verify that every finger is resolved. Investigate `FAILED` or `UNRESOLVED` results before relying on the assignment.
5. Stop the mapping launch with `Ctrl+C`, then start the dashboard with the same hand and mapping file.

The default mapping location is:

```text
${XDG_CONFIG_HOME:-~/.config}/allegro_hand_vision_sensor/finger_serials.yaml
```

Mapping and dashboard must use the same OS user and configuration location. For a custom path, pass the same `mapping_file:=/absolute/path/finger_serials.yaml` to both launches. The mapping is specific to the connected hand and camera serials; verify it after replacing a camera or changing hands. A missing mapping results in temporary camera placement. Machine-specific mappings are excluded from the release; `config/finger_serials.example.yaml` shows the file format.

## Optional Pose Control GUI

With the base controller running:

```bash
mkdir -p ~/v6_poses
ros2 run allegro_hand_v6_gui allegro_hand_v6_gui_node \
  --ros-args -p pose_directory:=$HOME/v6_poses
```

The GUI reads `/joint_states`, saves the current 20-joint pose, sends selected poses, and runs timed pose sequences. The explicit pose directory above provides a writable location. Without an override, the GUI uses the installed package's `poses/` directory, which includes an example pose.

Pose commands are published to `/allegro_hand_position_controller/commands`. Servo buttons publish `Float64MultiArray` requests to `/allegro_hand_servo_controller/commands`: `[1.0]` requests ON and `[0.0]` requests OFF. The base stack must provide a matching subscriber/controller for these requests; the GUI's requested state is not hardware feedback. The position-only base bringup does not by itself establish this servo interface. Closing the GUI stops its sequence and publishes an OFF request.

Use one joint-command source at a time: stop automatic mapping or another pose-sequence publisher before sending poses from this GUI.

## Optional Browser Monitor

```bash
ros2 run allegro_hand_v6_web_plot web_plot_bridge
```

Open **http://localhost:8080**. The bridge serves the page over HTTP and streams data over WebSocket port **9091**. The browser charts joint angles, desired positions, effort, and available sensor values. The effort trace uses `JointState.effort`; it is not a separate current measurement.

| Input | Required message type |
|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` |
| `/allegro_hand_position_controller/commands` | `std_msgs/msg/Float64MultiArray` |
| `/allegro_hand_v6/joint_temperatures` | `std_msgs/msg/Float32MultiArray` |
| `/allegro_hand_v6/tactile_sensors` | `std_msgs/msg/Float32MultiArray` |

Temperature and tactile tabs require these publishers. The base driver used by the dashboard publishes `/allegro_hand/tactile_pressures` as **Float64MultiArray**, so connecting that stream to the web monitor requires a message-type conversion bridge. A topic-name remap alone does not convert the type. Joint plots remain available independently.

The logo is optional. Place `WR_EN_high.png` in `allegro_hand_v6_web_plot/web/` before building to include it; no V5 repository is required.

## Verification and Troubleshooting

```bash
ros2 pkg prefix allegro_hand_v6_dashboard
ros2 pkg prefix allegro_hand_v6_description
ros2 control list_controllers
ros2 topic list
```

The dashboard should resolve to the UI workspace, while the description package should resolve to the base installation. For a blank or incorrect model, check both workspaces' setup files, matching hand/model arguments, `/robot_description`, and TF. The dashboard's `hand` argument does not change an already-running base publisher.

For missing pressure readings, compare the publisher's topic and message type with the selected UI's inputs. For missing camera images, check device permissions, USB connections, serial mapping, and whether another process is using the cameras.

After modifying this repository, rebuild the affected UI package, source `~/v6_ui_ws/install/setup.bash`, and restart its application. Rebuild the base workspace separately when its driver or robot descriptions change.

To run the dashboard tests without launching hardware-control tests from the base release:

```bash
cd ~/v6_ui_ws
colcon test --packages-select allegro_hand_v6_dashboard
colcon test-result --test-result-base build/allegro_hand_v6_dashboard --verbose
```

## Package Documentation

- [Dashboard](allegro_hand_v6_dashboard/README.md)
- [Vision sensor capture and mapping](allegro_hand_vision_sensor/README.md)
- [Pose control GUI — Korean](allegro_hand_v6_gui/README.md)
- [Browser monitor — Korean](allegro_hand_v6_web_plot/README.md)

Refer to the separately downloaded base release's README for hardware setup, supported communication descriptors, controller configuration, and model installation.
