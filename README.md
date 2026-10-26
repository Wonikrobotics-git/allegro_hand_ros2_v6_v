# Allegro Hand V6 V — ROS 2 Jazzy

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

ROS 2 software for the **Allegro Hand V6 V**, a five-finger robotic hand with 20 actuated joints. This repository includes the hardware driver, robot models, controller bringup, and UI applications in one workspace.

**Version: 1.0.0** · **Ubuntu 24.04 LTS / ROS 2 Jazzy**

Release versions use semantic versioning and Git tags such as `v1.0.0`. See [CHANGELOG.md](CHANGELOG.md) for release history.

## F and V Models

The F and V models share the same control software. Select the model that matches your physical hand when launching bringup:

| Model | Bringup argument | Camera mapping |
|---|---|---|
| V6 F | `model:=v6_f` | **Not required.** Proceed directly to bringup and the UI. |
| **V6 V** (this repository) | `model:=v6_v` | Follow the camera guide when using the finger cameras. |

All examples below use the **V model, right hand**. For a left hand, use `hand:=left` in both bringup and the dashboard. For a physical V model, complete camera-to-finger mapping before opening the dashboard with its camera views.

## Documentation

| Guide | Contents |
|---|---|
| [Bringup and hardware setup](src/allegro_hand_v6/allegro_hand_v6_bringup/README.md) | Ethernet setup, mock and physical operation, launch arguments, and controller checks. |
| [UI usage](src/allegro_hand_v6_ui/README.md) | Dashboard, pose control GUI, browser monitor, and their ROS interfaces. |
| [Dashboard manual](src/allegro_hand_v6_ui/allegro_hand_v6_dashboard/README.md) | Visualization, pressure display, layouts, and dashboard settings. |
| [CSV recording guide (Korean)](src/allegro_hand_v6/allegro_hand_v6_bringup/CSV_LOGGING.md) | Recording options, data columns, and rate checks. |
| [Validation record](VALIDATION.md) | Recorded build and mock checks, source refreshes, and verification scope. |

## Installation

Install ROS 2 Jazzy Desktop on Ubuntu 24.04 before continuing. The base packages and UI are included here; **build them together in this workspace**. The UI README also describes a separate UI distribution, whose overlay installation steps are unnecessary for this combined repository.

### 1. Clone the Repository

```bash
git clone https://github.com/Wonikrobotics-git/allegro_hand_ros2_v6_v.git \
  ~/allegro_hand_ros2_v6_v
cd ~/allegro_hand_ros2_v6_v
```

### 2. Install Dependencies

In a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y build-essential python3-colcon-common-extensions python3-rosdep

# Initialize rosdep once per PC, if it has not already been initialized.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --rosdistro jazzy

cd ~/allegro_hand_ros2_v6_v
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
```

### 3. Build

```bash
cd ~/allegro_hand_ros2_v6_v
colcon build --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

Use Jazzy environments throughout. Load the following in **every terminal** used to run the hand or UI:

```bash
source /opt/ros/jazzy/setup.bash
source ~/allegro_hand_ros2_v6_v/install/setup.bash
```

## Quick Start Without Hardware

Start mock bringup in terminal 1:

```bash
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  model:=v6_v hand:=right \
  ros2_control_hardware_type:=mock_components gui:=false
```

Open the dashboard in terminal 2:

```bash
ros2 launch allegro_hand_v6_dashboard dashboard.launch.py hand:=right
```

The dashboard displays the robot model and joint feedback from bringup. Its `hand` argument selects the 2D hand layout; the 3D model comes from bringup. Physical pressure readings require a connected hand. Camera mapping is unnecessary for this mock check. For physical V-model cameras, follow the mapping steps below.

## Run with a Physical Hand

Follow the [hardware setup guide](src/allegro_hand_v6/allegro_hand_v6_bringup/README.md#hardware-setup) to connect power and Ethernet and configure the PC's network address. Stop mock bringup before starting the hardware driver.

In terminal 1, use the actual IP address and port of your hand. The following command activates the physical hardware interface:

```bash
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  model:=v6_v hand:=right \
  ros2_control_hardware_type:=hardware \
  io_interface_descriptor:=modbus_tcp:192.168.1.100:502 \
  gui:=false
```

For a physical hand with cameras, complete [camera-to-finger mapping](#camera-to-finger-mapping) first, then open the dashboard in terminal 2. Sending poses from the pose GUI commands real joints.

For pose saving, sequences, and browser plots, see [UI usage](src/allegro_hand_v6_ui/README.md#optional-pose-control-gui). Use the same hand setting and `ROS_DOMAIN_ID` across applications.

## Camera-to-Finger Mapping

The V model uses camera serial numbers to assign images to fingers. For initial setup, start physical bringup as shown above and keep the dashboard closed. Review the mapping motion configuration before starting: **automatic mapping commands real joints and moves the hand through contact poses**.

In terminal 2:

```bash
ros2 launch allegro_hand_vision_sensor mapping.launch.py hand:=right
```

The launch starts camera capture and then the mapper. Wait for completion and confirm that every finger is resolved. Stop the mapping launch with `Ctrl+C` before opening the dashboard, so capture processes do not compete for the same cameras:

```bash
ros2 launch allegro_hand_v6_dashboard dashboard.launch.py hand:=right
```

Mapping and dashboard use the same default configuration file:

```text
${XDG_CONFIG_HOME:-~/.config}/allegro_hand_vision_sensor/finger_serials.yaml
```

Use the same OS user and hand setting for both applications. Verify the assignment after replacing a camera or changing hands. For device permissions, custom mapping files, and unresolved fingers, see the [camera capture and mapping guide](src/allegro_hand_v6_ui/README.md#camera-capture-and-finger-mapping).

## Optional CSV Recording

Add the following arguments to the physical bringup command:

```text
csv_logging:=true csv_directory:=./data_logs
```

Recording is disabled by default. CSV files contain timestamps, status counters, 20 joint positions, 20 measured currents, and 18 pressure values. The configured control rate is **500 Hz**; actual acquisition rate depends on the device, transport, and PC. See the [CSV recording guide](src/allegro_hand_v6/allegro_hand_v6_bringup/CSV_LOGGING.md) for details.

## Repository Layout

```text
src/
├── allegro_hand_common/   # Shared messages, I/O, utilities, and controllers
├── allegro_hand_v6/       # Bringup, robot descriptions, and hardware driver
└── allegro_hand_v6_ui/    # Dashboard, pose GUI, browser monitor, and camera tools
```

The F and V repositories share the same software and model assets. Use `model:=v6_v` for this V-model workflow; camera discovery and finger mapping are provided by `allegro_hand_vision_sensor`.

## Contributors

Maintained and released by [Wonikrobotics-official](https://github.com/Wonikrobotics-official).

- [shy0914](https://github.com/shy0914) — development and documentation contributions.

## License

Licensed under the [MIT License](LICENSE), except where an individual file or package specifies another license. Existing copyright and license notices are retained, including BSD and Apache-2.0 notices.

Copyright (c) 2026 WonikRobotics_official.

See also the dashboard's [third-party notices](src/allegro_hand_v6_ui/allegro_hand_v6_dashboard/THIRD_PARTY_NOTICES.md) for its external libraries.
