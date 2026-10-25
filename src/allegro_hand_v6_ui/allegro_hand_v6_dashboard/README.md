# Allegro Hand V6 Dashboard

The release target is Ubuntu 24.04 / ROS 2 Jazzy. Install the matching base V6 release first, then build this UI repository as a separate workspace overlay. See the [repository README](../README.md) for installation and launch instructions. Use the same `hand` argument in bringup and dashboard; the live 3D model follows the base stack's robot description and TF.

`allegro_hand_v6_dashboard` shows three palm pressure channels, five vision sensors, and the RViz 3D hand model in one Qt window. C++ threads open each camera directly through udev and OpenCV/V4L2. Palm pressure is read from `/allegro_hand/tactile_pressures`; connected cameras remain visible when pressure data is absent. The `digit-interface` SDK is not required.

## Install and build

Use Ubuntu 24.04 / ROS 2 Jazzy. Build the separately supplied base V6 release first, then follow the [repository installation instructions](../README.md) to build this UI overlay. Keep all four UI packages under `src/allegro_hand_v6_ui`. Load the following environment in each terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/allegro_v6_ws/install/setup.bash
source ~/v6_ui_ws/install/setup.bash
```

## First use: map cameras to fingers

The mapping file is named `finger_serials.yaml` and contains serials for one physical hand. When it is missing or empty, the dashboard temporarily assigns detected cameras to finger cards, so placement may vary. Map the serials on the physical hand before using the dashboard:

1. Connect the hand and all five cameras. Check the preshape and flexion direction in the shared `mapping_motion.yaml`. The legacy `mapping_motion_left.yaml` and `mapping_motion_right.yaml` paths are symbolic links to that file. Mapping defaults to a guarded sweep that moves real joints gradually while detecting contact. To use customized settings, pass `motion_file:=/absolute/path/to/motion.yaml` to `mapping.launch.py`.
2. Start the hand and controller in terminal 1:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/allegro_v6_ws/install/setup.bash
   source ~/v6_ui_ws/install/setup.bash
   ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py model:=v6_v hand:=left
   ```

3. After the controller is active, start capture and sweep mapping with one command in terminal 2 on the same ROS domain. It launches `vision_sensor.launch.py` automatically, then starts the mapper after a default delay of 5 seconds. The default contact threshold is `contact_threshold:=10.0`. Separate camera capture is unnecessary:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/allegro_v6_ws/install/setup.bash
   source ~/v6_ui_ws/install/setup.bash
   ros2 launch allegro_hand_vision_sensor mapping.launch.py hand:=left
   ```

4. Wait for `DONE` and its save path, then press `Ctrl+C` in terminal 2 to stop the mapping launch and capture. If the mapper reports `FAILED` or any `UNRESOLVED` finger, check the connections, preshape, contact response, and joint tracking error before rerunning. Confirm the YAML `hand` value and all five finger serials. For a right hand, use `right` consistently for every `hand` argument. See the [vision sensor manual](../allegro_hand_vision_sensor/README.md) for options and log explanations.

The default mapping path on every PC is `${XDG_CONFIG_HOME:-~/.config}/allegro_hand_vision_sensor/finger_serials.yaml` (the home directory is used when `XDG_CONFIG_HOME` is unset). The mapper creates its parent directory. Run both launches as the same OS user with the same `XDG_CONFIG_HOME` value. For another location, pass the same `mapping_file:=/absolute/path/finger_serials.yaml` to both. Existing mappings kept in a source-tree `config/finger_serials.yaml` must be copied to the new path or supplied with `mapping_file`.

## Run with a physical hand

Source `/opt/ros/jazzy/setup.bash` and the workspace's `install/setup.bash` in both terminals, using the same ROS domain.

```bash
# Terminal 1: hand, controller, robot_description, and TF
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py model:=v6_v hand:=left

# Terminal 2: cameras, palm pressure, and live 3D hand
ros2 launch allegro_hand_v6_dashboard dashboard.launch.py hand:=left
```

Start the dashboard separately. If the hand's communication address differs from the default `192.168.1.100:502`, set `io_interface_descriptor` on the bringup command; for a hand at `192.168.1.110`, pass `io_interface_descriptor:=modbus_tcp:192.168.1.110:502`. The dashboard opens the USB cameras directly, so stop the mapper's `vision_sensor_capture` and any other camera program before starting it. The mapping YAML and dashboard `hand` argument must agree. Map each different physical hand on its own PC or copy that hand’s mapping file.

## Frame integrity

The camera reader detects adjacent-row BGR mean differences exceeding both an absolute threshold of 15 and eight times the median. Candidates within five rows form one boundary. For one boundary, it swaps the upper and lower pieces and checks the result again. Frames with multiple boundaries or failed correction are discarded. The last accepted frame remains visible for up to one second; afterward the card shows `NO SIGNAL`. After three consecutive rejected frames, the camera is reopened.

Adjust `vision_sensor_tear_absolute_threshold`, `vision_sensor_tear_ratio_threshold`, `vision_sensor_tear_reopen_count`, and `vision_sensor_timeout` through dashboard launch arguments if needed.

## Tests and pressure layout

```bash
colcon test --packages-select allegro_hand_v6_dashboard
colcon test-result --test-result-base build/allegro_hand_v6_dashboard --verbose
```

If the physical V-model pressure channel count or order differs from the defaults, set `pressure_channel_count` and `pressure_indices`. The default pressure topic is `/allegro_hand/tactile_pressures` and can be changed with `pressure_topic`.

[Korean README](README_KR.md)
