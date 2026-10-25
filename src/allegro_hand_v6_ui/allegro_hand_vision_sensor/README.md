# Allegro Hand Vision Sensor

`allegro_hand_vision_sensor` discovers and captures vision sensors through udev and OpenCV/V4L2 and maps their serial numbers to fingers. It does not require the external `digit-interface` SDK. The hardware's `ID_MODEL=DIGIT` udev value is still used for discovery. Neither `/dev/videoN` numbers nor a particular set of serials is hardcoded.

## Install and verify

Use Ubuntu 24.04 / ROS 2 Jazzy. Build the separately supplied base V6 release first, then follow the [repository installation instructions](../README.md) to build this UI overlay. Keep all four UI packages under `src/allegro_hand_v6_ui`. Load the following environment in each terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/allegro_v6_ws/install/setup.bash
source ~/v6_ui_ws/install/setup.bash
```

## Capture images

```bash
ros2 launch allegro_hand_vision_sensor vision_sensor.launch.py
```

By default, all connected cameras publish to `/vision_sensor/s_<serial UTF-8 hex>/image_raw`. Detected serials are published on `/vision_sensor/connected_serials`. Use `target_serial:=<serial>` to inspect one camera, or `--show-args` to inspect launch arguments without starting capture.

**The dashboard opens cameras directly.** Do not run this capture node alongside `allegro_hand_v6_dashboard`. The mapping launch below starts its own capture node.

## Map cameras to fingers

Prepare the physical hand connection and device permissions first. Load the environment shown above in terminal 1.

```bash
# Terminal 1: start the physical hand and controller
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py model:=v6_v hand:=left
```

If the hand's communication address differs from the default `192.168.1.100:502`, set `io_interface_descriptor` on bringup. For example, start a right hand at `192.168.1.110` with the following command.

```bash
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py model:=v6_v hand:=right io_interface_descriptor:=modbus_tcp:192.168.1.110:502
```

After the controller is active, use another terminal with the same workspace environment and ROS domain:

```bash
# Terminal 2: start camera capture and automatic mapping
source /opt/ros/jazzy/setup.bash
source ~/allegro_v6_ws/install/setup.bash
source ~/v6_ui_ws/install/setup.bash
ros2 launch allegro_hand_vision_sensor mapping.launch.py hand:=left
```

`mapping.launch.py` runs contact-based guarded-sweep mapping by default; `mapping_sweep.launch.py` provides the same behavior and arguments. Both hands use the shared angles in `config/mapping_motion.yaml` as the reference for the preshape and sweep direction. In the default `sweep_joints:=flex` mode, the thumb and abduction joints move to the preshape, then the probed finger flexes gradually toward contact. Motion stops on contact, the tracking-error limit, or `s_max`. `hand: both` allows these poses on either hand; the launch argument `hand:=left` or `hand:=right` still identifies the mapping result and display orientation. The legacy `mapping_motion_left.yaml` and `mapping_motion_right.yaml` paths are symbolic links to the shared file, so edit the shared file once to update both hands. The mapper publishes real joint position commands; check the contact poses before running it. Motion settings load from the installed package share directory. After editing, rebuild the package or pass `motion_file:=/absolute/path/to/motion.yaml`. Custom files can also use `hand: both`; existing files with `hand: left` or `hand: right` remain specific to that hand. Wait for `DONE` and the save path, then stop capture. If it reports `FAILED` or any `UNRESOLVED` finger, check the cameras and poses and run it again.

Both mapping launches start `vision_sensor.launch.py` automatically, then start the mapper after a default delay of 5 seconds. The mapper continues waiting for camera images, an active controller, and joint feedback before moving. The default contact threshold is `contact_threshold:=10.0`. Start sensor capture and sweep mapping together with:

```bash
ros2 launch allegro_hand_vision_sensor mapping.launch.py hand:=right
```

Override sensitivity with, for example, `contact_threshold:=12.0`, or change the startup delay with `mapper_start_delay:=5.0`. If capture is already running, use `capture:=false mapper_start_delay:=0.0` to reuse it. `mapping_sweep.launch.py` accepts the same arguments.

### Main mapping arguments

| Argument | Default | Meaning |
|---|---|---|
| `contact_threshold` | `10.0` | Contact threshold for image change from the baseline; not force in newtons |
| `contact_frames` | `3` | Consecutive frames required to detect contact |
| `mapper_start_delay` | `5.0` | Delay before starting the mapper, in seconds |
| `sweep_joints` | `flex` | Sweep only the probed finger's flexion joints; `all` sweeps the thumb and that finger together |
| `s_max` | `1.3` | Sweep progress limit; `1.0` corresponds to the reference contact angles in the YAML |
| `approach_velocity` | `0.1` | Approach speed after `s_fast` (default `0.7`), in rad/s |
| `max_tracking_error` | `0.3` | Limit on commanded versus actual joint angle error, in radians |

### Logs and results

- `contact_preshape` / `blocked_preshape`: contact or tracking error during preshape skips that finger and returns to ready. Check these logs if the hand suddenly returns during preparation.
- `Tracking block`: identifies the joint, commanded angle, actual angle, and error. It does not by itself establish a joint-range violation.
- `MATCH`: a sustained, unique sensor response was confirmed during observation. `UNRESOLVED` means that finger was not mapped; `DONE` alone does not mean every finger succeeded.

Thumb inference requires all five sensors to participate and confirmed index, middle, ring, and pinky mappings. Mapping can continue with fewer sensors, but the thumb stays unresolved. Sensors connected after the run starts are ignored until the next run.

Each run appends a record to `sweep_report_<hand>.yaml` beside the mapping file. Override the path with `report_file:=/absolute/path/report.yaml`. Capture stays running after `DONE`; press `Ctrl+C` in the mapping terminal before opening the dashboard.

By default, the result is written to `${XDG_CONFIG_HOME:-~/.config}/allegro_hand_vision_sensor/finger_serials.yaml`, which the dashboard also reads. The mapper creates the parent directory. Run mapping and dashboard as the same OS user with the same `XDG_CONFIG_HOME` value. The mapping is machine-specific and excluded from Git and the installed package; only `finger_serials.example.yaml` is distributed. To use another location, pass the same `mapping_file:=/absolute/path/finger_serials.yaml` to both mapping and dashboard launches. Copy any existing source-tree `config/finger_serials.yaml` to the new path or pass its path explicitly. Stop terminal 2's capture before starting the dashboard; see the `allegro_hand_v6_dashboard` README for the full order.

A replacement camera may be discovered with a new serial, but the old finger assignment is not transferred automatically. A different physical hand also needs its own mapping. Verify and repeat mapping after replacement.

## Tests

```bash
colcon test --packages-select allegro_hand_vision_sensor
colcon test-result --test-result-base build/allegro_hand_vision_sensor --verbose
```

The SDK replacement and its validation record are kept in [docs/SDK_REPLACEMENT.md](docs/SDK_REPLACEMENT.md).

[Korean README](README_KR.md)
