# Allegro Hand V6 Pose Control GUI

Save and send 20-joint poses and run timed pose sequences on Ubuntu 24.04 / ROS 2 Jazzy. Build the complete repository using the [root installation guide](../../../README.md#installation).

## Start the GUI

Load the same workspace in every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/allegro_hand_ros2_v6_v/install/setup.bash
```

In terminal 1, start with mock feedback:

```bash
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  model:=v6_v hand:=right ros2_control_hardware_type:=mock_components gui:=false
```

In terminal 2, use a writable pose directory dedicated to this model and side:

```bash
mkdir -p ~/v6_poses/v_right
ros2 run allegro_hand_v6_gui allegro_hand_v6_gui_node \
  --ros-args -p pose_directory:=$HOME/v6_poses/v_right
```

An empty directory initially shows no saved poses. For a left hand, change the bringup side and use a separate directory such as `f_left` or `v_left`. The GUI subscribes to the running hand; it has no separate `model` or `hand` parameter.

## First Pose and Sequence

### 1. Check feedback and save a baseline

1. Wait for **Joint state: ready (20/20)**. In another sourced terminal, verify:

   ```bash
   ros2 control list_controllers
   ros2 topic echo /joint_states --once
   ```

   `joint_state_broadcaster` and `allegro_hand_position_controller` should be active; the message should contain 20 joint names and positions.
2. Enter `baseline` in **Saved Poses** and click **Save Current**. The GUI writes `baseline.yaml` to the configured directory. A name may contain letters, numbers, `_`, and `-`; feedback must be complete and less than two seconds old.
3. Select `baseline` and click **Move**. This republishes the saved current position, so no visible movement is expected.

### 2. Make a small mock-only movement

Keep terminal 1 in **mock_components** mode. After saving the baseline, run:

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
pose_dir = Path.home() / 'v6_poses' / 'v_right'
pose = yaml.safe_load((pose_dir / 'baseline.yaml').read_text())
index = pose['joint_names'].index('joint11')
pose['position'][index] += 0.05
(pose_dir / 'mock_step.yaml').write_text(yaml.safe_dump(pose, sort_keys=False))
PY
```

Click **Refresh**, select `mock_step`, and click **Move**. Mock joint feedback should show the changed `joint11` position; a separately running dashboard should reflect it. Select `baseline` and **Move** to restore the mock position.

**This example is for mock operation only.** For a physical hand, use pose files validated for that model, side, and fixture. The example zero pose shipped in `poses/1.yaml` is a format example, not a universally safe physical pose. Do not change physical joint targets based only on this mock example.

### 3. Run and stop a sequence

1. Add `baseline`, then `mock_step`, using **Add to Sequence**.
2. Set **Pose interval** (for example, `2.0 s`) and **Repeat count**, then click **Start**.
3. Click **Stop** to stop sending subsequent poses. The hand/controller retains its last commanded target; Stop does not disable torque or undo a command already sent.

The first pose is sent immediately. The interval sets the delay between target publications, not the movement speed. Move sends one position target; the GUI does not interpolate a trajectory or verify arrival. Sequence lists are not saved across GUI sessions; pose YAML files are retained.

### 4. Use a physical hand

Stop mock bringup, then follow the [physical bringup guide](../../allegro_hand_v6/allegro_hand_v6_bringup/README.md#hardware-setup). Use `model:=v6_v`, the correct side, and the actual communication endpoint. Verify active controllers and 20-joint feedback before sending a validated pose.

Torque is enabled automatically when the position controller starts. **No Servo ON button press is required.** The supplied bringup has no receiver for `/allegro_hand_servo_controller/commands`, so the GUI Servo ON/OFF buttons do not control physical torque. Their **Requested** label is not device feedback.

Use one joint-command source at a time: finish and stop V-model mapping before using the pose GUI; stop other motion publishers as well.

### 5. End a session

Stop the sequence, close the GUI, and stop any mapping/capture process before closing the dashboard. End bringup with `Ctrl+C` in its terminal. The hardware cleanup attempts to disable torque; check its shutdown output and the physical device. Closing only the GUI does not disable torque, and this software sequence is not an emergency-stop mechanism.

## Pose Files and Joint Order

Each YAML file contains 20 finite positions in radians. Saved files include `joint_names`; the GUI uses names to reorder values into this controller order:

```text
joint00 joint01 joint02 joint03    # thumb
joint10 joint11 joint12 joint13    # index
joint20 joint21 joint22 joint23    # middle
joint30 joint31 joint32 joint33    # ring
joint40 joint41 joint42 joint43    # pinky
```

If `joint_names` is omitted, `position` must already be in that order. Unknown or missing names, an incorrect value count, or NaN/Infinity are rejected. Double-clicking a saved pose is equivalent to Move. **Save Current** records feedback; it does not provide a joint-angle editor.

## ROS Interfaces

| Direction | Topic | Message | Meaning |
|---|---|---|---|
| Subscribe | `/joint_states` | `sensor_msgs/msg/JointState` | Feedback for all 20 joints. |
| Publish | `/allegro_hand_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | 20 position targets in radians. |
| Publish | `/allegro_hand_servo_controller/commands` | `std_msgs/msg/Float64MultiArray` | Servo requests only; unhandled by the supplied bringup. |

Parameters: `joint_state_topic`, `position_command_topic`, `servo_command_topic`, and `pose_directory`. Without `pose_directory`, the installed package's `poses/` folder is used and may not be writable. Prefer the explicit directory shown above.

## Troubleshooting

| Symptom | Check |
|---|---|
| Joint state stays waiting | Source this workspace, start bringup, and confirm `/joint_states` has 20 named, finite positions. |
| Save Current fails | Check fresh feedback, valid filename, and write permission on `pose_directory`. |
| Pose list is empty | Save a baseline or copy validated YAML files into that directory, then Refresh. |
| Move has no effect | Verify the position controller is active, the selected pose differs from feedback, and no other publisher is replacing commands. |
| Sequence stops early | Read the GUI log for missing or invalid pose files. |
| Servo OFF does not release the hand | The request topic has no receiver in this release; it is not a physical torque-off control. |

[Korean reference](README_KR.md)
