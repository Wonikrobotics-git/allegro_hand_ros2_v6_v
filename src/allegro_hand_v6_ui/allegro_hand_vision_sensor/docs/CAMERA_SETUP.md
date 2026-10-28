# V-Model Camera Setup and Diagnostics

This guide applies to **V-model USB cameras**. F-model users skip camera capture and mapping. Ethernet connects the hand controller; camera images arrive separately through USB/V4L2. Do the camera-only checks below before starting automatic mapping, which moves real joints.

## 1. Load the Workspace and Diagnostic Tools

Build the [complete repository](../../../../README.md#installation), then source:

```bash
source /opt/ros/jazzy/setup.bash
source ~/allegro_hand_ros2_v6_v/install/setup.bash
sudo apt install -y v4l-utils acl psmisc
```

Close the dashboard, mapping/capture launches, and other camera applications. Only one process should open each camera at a time.

## 2. Check USB Enumeration

```bash
v4l2-ctl --list-devices
```

Multiple `/dev/video*` nodes can belong to one sensor; do not count nodes as cameras. Use the same udev properties as the capture code to list unique serials:

```bash
python3 - <<'PY'
import pyudev
serials = set()
for device in pyudev.Context().list_devices(subsystem='video4linux'):
    if device.get('ID_MODEL') != 'DIGIT':
        continue
    if ':capture:' not in device.get('ID_V4L_CAPABILITIES', ''):
        continue
    serial = device.get('ID_SERIAL_SHORT')
    if serial and device.device_node:
        serials.add(serial)
        print(serial, device.device_node)
print('Unique camera serials:', len(serials))
PY
```

For the full five-finger mapping workflow, expect **five unique serials**. The thumb assignment requires all five sensors and resolved mappings for the other four fingers. The device filter currently requires `ID_MODEL=DIGIT`, a capture capability, and a serial; a generic USB camera is not automatically a supported sensor. If a serial appears with multiple capture nodes, investigate the device properties rather than selecting an arbitrary node.

## 3. Check Access and Competing Processes

Use an actual capture node reported above; `/dev/video0` below is only an example:

```bash
ls -l /dev/video0
getfacl /dev/video0
id -nG
fuser -v /dev/video0
v4l2-ctl --device=/dev/video0 --all
```

Read/write access may be granted by a desktop-session ACL or by the device's group. If access is denied and the device belongs to group `video`, add your user to that group:

```bash
sudo usermod -aG video "$(id -un)"
```

Log out and back in, reconnect the cameras, and recheck access. Group membership changes do not update an already running desktop session. If another group or a custom udev rule controls the device, use that system's access policy. Run ROS applications as your normal user; running them with `sudo` changes both the environment and mapping-file location.

If `fuser` shows another camera process, close it normally before retrying. Do not terminate unrelated processes merely because they are listed.

## 4. Verify Camera Frames Without Moving the Hand

In terminal 1:

```bash
ros2 launch allegro_hand_vision_sensor vision_sensor.launch.py
```

In terminal 2, after sourcing the same workspace:

```bash
ros2 topic echo /vision_sensor/connected_serials --once
ros2 topic list | grep '/vision_sensor/.*/image_raw'
```

Capture logs should report streaming sensors. To display one image topic from that list:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select the exact `image_raw` topic in the topic selector. If that optional viewer is unavailable, install `ros-jazzy-rqt-image-view`. Confirm each sensor supplies a live, illuminated image. A changing frame should respond when its visible surface changes; a detected serial alone does not establish valid imagery.

To isolate a sensor, stop capture with `Ctrl+C` and relaunch with its actual serial:

```bash
ros2 launch allegro_hand_vision_sensor vision_sensor.launch.py target_serial:=YOUR_CAMERA_SERIAL
```

The image topic uses the serial's **UTF-8 hexadecimal encoding**, not the literal serial. The launch default `led_control:=-1` preserves the camera's current illumination; it does not turn a dark LED on. Check the hand/sensor's supported illumination setup if frames are dark. Use explicit LED controls only for firmware you have verified; the dashboard itself has no equivalent LED-control launch argument.

## 5. Map, Verify, Then Open the Dashboard

1. Stop the camera-only capture with `Ctrl+C`.
2. Start physical bringup with `model:=v6_v`, the correct hand side, and the actual IP/port. Confirm active controllers and 20-joint feedback.
3. Review the installed `config/mapping_motion.yaml` or pass a validated custom `motion_file`. With the dashboard closed, start:

   ```bash
   ros2 launch allegro_hand_vision_sensor mapping.launch.py hand:=right
   ```

4. Wait for completion. Check all five finger assignments; `DONE` alone is insufficient if any result is `UNRESOLVED` or `FAILED`. Check the saved YAML and `sweep_report_right.yaml` beside it. For a left hand, use `hand:=left` and the corresponding report.
5. Stop the mapping launch with `Ctrl+C`, including its capture node, then start:

   ```bash
   ros2 launch allegro_hand_v6_dashboard dashboard.launch.py hand:=right
   ```

The default mapping file is `$HOME/.config/allegro_hand_vision_sensor/finger_serials.yaml`, or the same suffix under an absolute `XDG_CONFIG_HOME`. Mapping and dashboard must use the same OS user, hand, and file. For a custom location, pass `mapping_file:=/absolute/path/finger_serials.yaml` to both launches.

## Troubleshooting

| Symptom | Checks |
|---|---|
| No camera serials | Check USB/power connections, udev properties, and capture capabilities; the Ethernet connection alone supplies no camera images. |
| Fewer than five unique serials | Check each connection, hub power, USB bandwidth, and duplicate serial properties; test sensors individually. |
| Cannot open a video node | Check ACL/group access and whether another application owns it. |
| Serial detected but image missing | Confirm an `image_raw` stream, accepted V4L2 format, and usable illumination. Read capture errors. |
| Only one sensor works at a time | Compare individual and simultaneous capture; inspect USB bandwidth/power and process ownership. |
| Mapper keeps waiting | Check camera frames, `/joint_states`, and active `allegro_hand_position_controller` in the same ROS domain. |
| `Tracking block` / `blocked_preshape` | Review actual versus commanded positions and the motion template; do not increase limits merely to bypass the failure. |
| `DONE` with an unresolved finger | Inspect every assignment and sweep report, correct the cause, and repeat mapping. |
| Dashboard images on wrong fingers | Confirm the correct physical hand, hand side, OS user, and mapping path. Remap after sensor replacement. |
| Images disappear when dashboard starts | Stop standalone capture and mapping; the dashboard opens cameras directly. |

See [mapping arguments and result interpretation](../README.md#map-cameras-to-fingers).
