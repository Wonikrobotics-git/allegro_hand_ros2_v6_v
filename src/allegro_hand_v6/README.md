# Allegro Hand V6 Base Packages

The base packages provide robot descriptions, hardware communication, and controller bringup for the five-finger, 20-joint Allegro Hand V6.

| Package | Purpose |
|---|---|
| `allegro_hand_v6_bringup` | Controller launch files, settings, and CSV recording. |
| `allegro_hand_v6_description` | F/V left/right robot models and mesh assets. |
| `allegro_hand_v6_hardware` | ROS 2 hardware interface and SDK. |

## Install and Run

Use the [repository installation guide](../../README.md#installation) for Ubuntu 24.04 / ROS 2 Jazzy. The common, base, and UI packages are all in the same workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/allegro_hand_ros2_v6_v
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

For execution, follow the [bringup guide](allegro_hand_v6_bringup/README.md). It uses `model:=v6_v hand:=right`; set the actual side and endpoint for your hand. The position controller enables torque automatically. GUI Servo requests are not connected to a physical torque-control receiver in this release.

- [First pose and sequence](../allegro_hand_v6_ui/allegro_hand_v6_gui/README.md#first-pose-and-sequence)
- [CSV recording](allegro_hand_v6_bringup/CSV_LOGGING.md)
- [V-model camera setup](../allegro_hand_v6_ui/allegro_hand_vision_sensor/docs/CAMERA_SETUP.md) — F-model users skip this.
