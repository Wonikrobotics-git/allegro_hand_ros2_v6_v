# Allegro Hand V6 Bringup

Launch files and controller configuration for Allegro Hand V6 on **Ubuntu 24.04 / ROS 2 Jazzy**. Bringup starts the controller manager, robot state publisher, joint-state broadcaster, position controller, and optional RViz window.

For installation and the combined base/UI build, follow the [repository README](../../../README.md).

## Load the Workspace

In every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/allegro_hand_ros2_v6_v/install/setup.bash
```

The commands below use a right V-model hand. Set `hand:=left` for a left hand. Pass `model:=v6_v` explicitly for the V model; the shared launch file defaults to `v6_f`.

## Mock Bringup

Run without connecting to a physical hand:

```bash
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  model:=v6_v hand:=right \
  ros2_control_hardware_type:=mock_components gui:=false
```

In another terminal, open the dashboard:

```bash
ros2 launch allegro_hand_v6_dashboard dashboard.launch.py hand:=right
```

Alternatively, use `gui:=true` in bringup to open standalone RViz. Mock mode supplies joint feedback; it does not supply real pressure measurements or camera images.

## Hardware Setup

1. Connect the hand to the specified 24 V DC supply and connect its Ethernet interface to the PC.
2. Configure the PC's Ethernet interface on the hand's subnet. For a hand at `192.168.1.100`, an example PC address is `192.168.1.10/24`. Use a different address from the hand.
3. Check connectivity with `ping 192.168.1.100`, substituting your device's address.
4. Stop any previous mock or hardware bringup instance before starting the driver.

Start physical bringup with the connected hand's model, side, and endpoint:

```bash
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  model:=v6_v hand:=right \
  ros2_control_hardware_type:=hardware \
  io_interface_descriptor:=modbus_tcp:192.168.1.100:502 \
  gui:=false
```

This activates the physical hardware interface. Pose commands move real joints. With the dashboard closed, follow the [camera capture and mapping guide](../../allegro_hand_v6_ui/README.md#camera-capture-and-finger-mapping) for initial V-model camera setup. Stop mapping/capture when finished, then start the dashboard separately:

```bash
ros2 launch allegro_hand_v6_dashboard dashboard.launch.py hand:=right
```

**The F model does not require camera capture or finger mapping.** The V model's camera workflow is documented in the [UI guide](../../allegro_hand_v6_ui/README.md#camera-capture-and-finger-mapping).

## Launch Arguments

| Argument | Default | Purpose |
|---|---|---|
| `model` | `v6_f` | Robot variant: `v6_f` (F) or `v6_v` (V). |
| `hand` | `right` | Robot side: `right` or `left`. |
| `ros2_control_hardware_type` | `hardware` | Use `hardware` for the physical hand or `mock_components` for a device-free check. |
| `io_interface_descriptor` | `modbus_tcp:192.168.1.100:502` | Physical hand's communication endpoint. |
| `hand_id` | `1` | Modbus slave/hand ID. |
| `gui` | `true` | Open standalone RViz; use `false` when launching the dashboard separately. |
| `use_sim_time` | `false` | Use the ROS simulation clock. |
| `csv_logging` | `false` | Enable CSV recording during physical hardware reads. |
| `csv_directory` | `./data_logs` | CSV output directory, relative to the launch working directory. |

Optional `isaac` mode requires a separately installed Jazzy-compatible `wr_topic_based_ros2_control` plugin.

To inspect available arguments:

```bash
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py --show-args
```

## Check Controllers and Joint Feedback

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

The `joint_state_broadcaster` and `allegro_hand_position_controller` should be active, and `/joint_states` should contain 20 joints. The position command topic is `/allegro_hand_position_controller/commands`.

For a missing or incorrect model, check that the workspace is sourced and that bringup and dashboard use the same `hand`. The dashboard receives its 3D model from `/robot_description` and TF; changing the dashboard's hand setting does not change the model published by bringup.

For connection failures, check the hand's power, cable, PC subnet, device IP/port, and whether another driver instance is connected.

## CSV Recording and UI

Enable recording by adding `csv_logging:=true csv_directory:=./data_logs` to the physical bringup command. See [CSV_LOGGING.md (Korean)](CSV_LOGGING.md) for the format and acquisition-rate checks. Mock mode does not produce hardware CSV recordings.

See the [UI guide](../../allegro_hand_v6_ui/README.md) for dashboard settings, pose control, browser monitoring, and input-topic requirements. Use the installation instructions in the repository root for this combined workspace.
