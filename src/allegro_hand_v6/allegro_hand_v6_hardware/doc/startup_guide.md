# Allegro Hand V6 ROS 2 Quick Start Guide

### 1. Create a ROS 2 Workspace

```bash
mkdir -p ~/allegro_ws/src
```

### 2. Install Required Dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-xacro \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  libmodbus-dev \
  libfmt-dev \
  libspdlog-dev
```

### 3. Clone Allegro Hand ROS 2 Repository

```bash
cd ~/allegro_ws/src
git clone -b jazzy https://github.com/Wonikrobotics-git/allegro_hand_ros2.git
```

### 4. Build Workspace

```bash
cd ~/allegro_ws
colcon build --symlink-install
source install/setup.bash
```

### 5. Hardware Setup & Network Configuration

1. Power on Allegro Hand V6 (24V DC).
2. Connect the Ethernet cable between your PC and Allegro Hand V6.
3. Configure your PC static IP to `192.168.1.x` (e.g., `192.168.1.10`, subnet mask `255.255.255.0`).
4. Verify connection:
   ```bash
   ping 192.168.1.100
   ```

### 6. Start the ROS 2 Driver Node

```bash
source ~/allegro_ws/install/setup.bash
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py hand:=right
```
*(Optional parameters: `hand:=left`, `io_interface_descriptor:=modbus_tcp:192.168.1.100:502`)*

### 7. Run Finger Wave Motion Example

Open a new terminal and execute:

```bash
source ~/allegro_ws/install/setup.bash
ros2 run allegro_hand_v6_bringup finger_wave.py
```
