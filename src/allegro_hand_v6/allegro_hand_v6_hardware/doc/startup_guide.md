# Allegro Hand V6 Hardware Quick Start

For this combined source distribution, use the [root installation guide](../../../../README.md#installation). It contains the correct repository URL, Jazzy dependencies, and workspace build steps.

Load the workspace in every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/allegro_hand_ros2_v6_v/install/setup.bash
```

1. Complete the [power/Ethernet setup](../../allegro_hand_v6_bringup/README.md#hardware-setup) and stop any mock bringup.
2. Start the driver with the connected hand's actual model, side, and endpoint:

   ```bash
   ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
     model:=v6_v hand:=right ros2_control_hardware_type:=hardware \
     io_interface_descriptor:=modbus_tcp:192.168.1.100:502 gui:=false
   ```

3. Check `ros2 control list_controllers` and `ros2 topic echo /joint_states --once` in another sourced terminal. Expect an active position controller and 20 joint positions.
4. F-model users can open the dashboard directly. V-model users first complete [camera checks and mapping](../../../allegro_hand_v6_ui/allegro_hand_vision_sensor/docs/CAMERA_SETUP.md).
5. Follow the [English pose walkthrough](../../../allegro_hand_v6_ui/allegro_hand_v6_gui/README.md#first-pose-and-sequence) for mock practice, validated physical poses, and session shutdown.

Torque is enabled automatically when the position controller starts. The GUI Servo buttons are requests only and have no physical receiver in this release.
