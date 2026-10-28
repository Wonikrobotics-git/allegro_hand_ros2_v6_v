# Allegro Hand V6 Dashboard

Display the live 3D hand, joint feedback, and available pressure readings. The V model also displays USB camera images. The dashboard monitors the hand; use the [pose GUI](../allegro_hand_v6_gui/README.md#first-pose-and-sequence) to send joint targets.

## Install and Build

Use the [complete repository installation guide](../../../README.md#installation) for Ubuntu 24.04 / ROS 2 Jazzy. The base packages and UI are already included in one workspace. In every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/allegro_hand_ros2_v6_v/install/setup.bash
```

## First Check Without Hardware

```bash
# Terminal 1
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  model:=v6_v hand:=right ros2_control_hardware_type:=mock_components gui:=false
```

```bash
# Terminal 2
ros2 launch allegro_hand_v6_dashboard dashboard.launch.py hand:=right
```

Expect the matching 3D hand and mock joint feedback. Physical pressures require the hand. The dashboard's `hand` argument selects its 2D layout; bringup supplies the 3D model and TF. For a left hand, set `hand:=left` in both commands.

## Run with a Physical Hand

Follow [Ethernet and controller setup](../../allegro_hand_v6/allegro_hand_v6_bringup/README.md#hardware-setup). Stop mock bringup, then run:

```bash
# Terminal 1: connected hand and controller
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  model:=v6_v hand:=right ros2_control_hardware_type:=hardware \
  io_interface_descriptor:=modbus_tcp:192.168.1.100:502 gui:=false
```

Use the hand's actual IP and side. Check `ros2 control list_controllers` and `/joint_states` before opening the dashboard.

- **F model:** open the dashboard directly. Camera setup and mapping are not required.
- **V model:** complete [camera checks](../allegro_hand_vision_sensor/docs/CAMERA_SETUP.md) and the [mapping procedure](../allegro_hand_vision_sensor/README.md#map-cameras-to-fingers) first. That physical bringup must explicitly use `model:=v6_v`. Stop mapping and standalone capture with `Ctrl+C` before opening the dashboard; it opens the cameras directly.

```bash
# Terminal 2: after any V-model mapping/capture has stopped
ros2 launch allegro_hand_v6_dashboard dashboard.launch.py hand:=right
```

Confirm the model/hand shown by the dashboard matches the connected device. For a left hand, use `hand:=left` throughout. Keep the same `ROS_DOMAIN_ID` and mapping path across applications.

## Camera Assignments — V Model Only

A missing mapping file can produce temporary camera placement. Verify all five finger assignments after mapping; `DONE` alone does not establish success. The default file is `$HOME/.config/allegro_hand_vision_sensor/finger_serials.yaml`, or the equivalent under an absolute `XDG_CONFIG_HOME`. Pass the same custom `mapping_file:=/absolute/path/finger_serials.yaml` to mapping and dashboard when needed. A new physical hand or replacement camera needs its own verified assignment.

## Configuration and Troubleshooting

See the [UI configuration table](../README.md#dashboard-configuration) for `hand`, `pressure_topic`, `mapping_file`, layouts, and view settings. The default pressure topic is `/allegro_hand/tactile_pressures` (`Float64MultiArray`).

| Symptom | Checks |
|---|---|
| Blank/wrong 3D model | Source this workspace; verify bringup model/hand, `/robot_description`, and TF. Stop old robot-state publishers. |
| Missing pressure readings | Check the physical connection, pressure topic/type, and any channel-layout override. |
| Missing or misplaced V-model images | Follow [camera diagnostics](../allegro_hand_vision_sensor/docs/CAMERA_SETUP.md#troubleshooting); verify process ownership and mapping. |
| GUI closed but hand still holds position | The dashboard does not control torque. See [ending a session](../allegro_hand_v6_gui/README.md#5-end-a-session). |

## Frame integrity

The camera reader detects adjacent-row BGR mean differences exceeding both an absolute threshold of 15 and eight times the median. Candidates within five rows form one boundary. For one boundary, it swaps the upper and lower pieces and checks the result again. Frames with multiple boundaries or failed correction are discarded. The last accepted frame remains visible for up to one second; afterward the card shows `NO SIGNAL`. After three consecutive rejected frames, the camera is reopened.

Adjust `vision_sensor_tear_absolute_threshold`, `vision_sensor_tear_ratio_threshold`, `vision_sensor_tear_reopen_count`, and `vision_sensor_timeout` through dashboard launch arguments if needed.

## Tests and pressure layout

```bash
cd ~/allegro_hand_ros2_v6_v
colcon build --symlink-install --packages-up-to allegro_hand_v6_dashboard --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select allegro_hand_v6_dashboard
colcon test-result --test-result-base build/allegro_hand_v6_dashboard --verbose
```

If the physical V-model pressure channel count or order differs from the defaults, set `pressure_channel_count` and `pressure_indices`. The default pressure topic is `/allegro_hand/tactile_pressures` and can be changed with `pressure_topic`.

[Korean README](README_KR.md)
