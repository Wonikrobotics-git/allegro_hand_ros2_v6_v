# allegro_hand_v6_hardware

`allegro_hand_v6_hardware` 패키지는 Wonik Robotics의 **Allegro Hand V6 (5손가락 20관절)**를 위한 `ros2_control` 하드웨어 인터페이스 및 전용 통신 SDK(`v6_sdk`)를 제공합니다.

이 패키지는 `ros2_control` 프레임워크의 하드웨어 인터페이스 플러그인(`hardware_interface::SystemInterface`)으로 동작하며, ROS 2 제어기(Controller)와 실물 Allegro Hand V6 하드웨어(Modbus TCP/RTU) 간의 통신 브리지 역할을 수행합니다. 20개 관절의 위치(`position`) 및 토크(`effort`) 제어 인터페이스를 수출하고, 18개 촉각 센서 압력 및 BNO055 IMU 데이터를 실시간으로 수신 및 발행합니다.

---

## 1. 주요 기능 (Key Features)

* **`ros2_control` 통합**: `controller_manager`에 의해 동적으로 로드 및 관리되는 표준 `hardware_interface::SystemInterface` 플러그인(`AllegroHandV6HardwareInterface` / `AllegroHandV6System`).
* **State Interfaces**: 
  * 20개 관절: `position`, `velocity` (SDK 내부 1차 저역통과 필터 적용), `effort`, `temperature`
  * BNO055 IMU 센서: `orientation.w`, `orientation.x`, `orientation.y`, `orientation.z`
  * 18개 촉각 압력 센서: `tactile_pressure/pressure_0` ~ `pressure_17` (hPa 단위)
* **Command Interfaces**: 20개 관절에 대한 `position` 및 `effort` 명령 인터페이스를 모두 수출하여 위치 제어 및 토크 기반 하이브리드 제어 지원.
* **실시간 센서 토픽 스트리밍**: `realtime_tools::RealtimePublisher`를 통해 고속 제어 루프에서 지연 없이 `~/imu/data` 및 `~/tactile_pressures` 토픽 발행.
* **진단 정보 퍼블리싱 (`Diagnostics`)**: `diagnostic_updater`를 활용해 20개 모터 온도, 18개 센서 온도, 오류 비트마스크, 통신 주기/지터 모니터링 정보를 `/diagnostics` 토픽으로 주기적 발행.
* **실시간 파라미터 동적 튜닝**: 관절별 최대 토크 한계값(`torque_limit`) 및 초기 위치(`initial_position`)를 ROS 2 파라미터 서버를 통해 실시간으로 조회/변경.

---

## 2. 관절 명칭 정의 (Joint Name Definitions)

Allegro Hand V6는 5개의 손가락(엄지, 검지, 중지, 약지, 새끼)과 손가락당 4개의 관절로 구성되어 **총 20개의 관절**을 가집니다. SDK 및 URDF에서 사용하는 관절 명칭 규칙은 다음과 같습니다.

| 손가락 (Finger) | 관절 0 (Base) | 관절 1 (Proximal) | 관절 2 (Medial) | 관절 3 (Distal) |
| :--- | :--- | :--- | :--- | :--- |
| **Thumb (엄지)** | `joint00` | `joint01` | `joint02` | `joint03` |
| **Index (검지)** | `joint10` | `joint11` | `joint12` | `joint13` |
| **Middle (중지)** | `joint20` | `joint21` | `joint22` | `joint23` |
| **Ring (약지)** | `joint30` | `joint31` | `joint32` | `joint33` |
| **Pinky (새끼)** | `joint40` | `joint41` | `joint42` | `joint43` |

URDF 설정 시 `device_prefix` 파라미터를 지정하면 모든 관절 이름 앞에 접두사가 추가됩니다 (예: `right_joint00`).

---

## 3. 플러그인 파라미터 (Plugin Parameters)

URDF/Xacro 파일 내 `<ros2_control>` 태그의 `<hardware>` 섹션에서 다음과 같은 파라미터를 설정할 수 있습니다.

```xml
<ros2_control name="allegro_hand" type="system">
    <hardware>
        <plugin>allegro_hand_v6_hardware/AllegroHandV6System</plugin>
        <param name="io_interface_descriptor">modbus_tcp:192.168.1.100:502</param>
        <param name="hand_id">1</param>
        <param name="device_prefix"></param>
    </hardware>
    ...
</ros2_control>
```

* **`io_interface_descriptor`** (`string`, 필수, 기본값: `modbus_tcp:192.168.1.100:502`)
  * 실물 Allegro Hand V6와의 통신 디스크립터 형식입니다.
    * **Modbus TCP**: `modbus_tcp:<IP_ADDRESS>[:<PORT>]` (예: `modbus_tcp:192.168.1.100:502`)
    * **Modbus RTU**: `modbus_rtu:<SERIAL_PORT>[,<BAUDRATE>]` (예: `modbus_rtu:/dev/ttyUSB0,2000000`)
    * **고속 시리얼 / Protobuf (Placeholder)**: `serial:<SERIAL_PORT>[,<BAUDRATE>]` (예: `serial:/dev/ttyUSB0,2000000`)
* **`hand_id`** (`int`, 선택, 기본값: `1`)
  * Modbus Slave ID 번호입니다.
* **`device_prefix`** (`string`, 선택, 기본값: `""`)
  * 양손 설정 등에서 관절 명칭과 노드 명칭을 구분하기 위한 접두사입니다.

---

## 4. 의존성 패키지 (Dependencies)

* `ament_cmake`
* `rclcpp`
* `rclcpp_lifecycle`
* `hardware_interface`
* `pluginlib`
* `realtime_tools`
* `sensor_msgs`
* `std_msgs`
* `diagnostic_updater`
* `spdlog`
* `libmodbus-dev` (시스템 라이브러리)
* `allegro_hand_utility`

---

## 5. 빌드 및 테스트 (Build & Test)

Colcon을 사용하여 워크스페이스 루트에서 빌드합니다.

```bash
# 패키지 빌드
colcon build --symlink-install --packages-select allegro_hand_v6_hardware

# 빌드 환경 반영
source install/setup.bash
```

### 단위 테스트 실행

```bash
# 1. SDK 통신, 피드백, 압력/IMU, 진단, 게인 R/W 단위 테스트
/colcon_ws/install/allegro_hand_v6_hardware/lib/allegro_hand_v6_hardware/test_v6_sdk

# 2. ros2_control 하드웨어 인터페이스 라이프사이클 및 플러그인 로드 단위 테스트
/colcon_ws/install/allegro_hand_v6_hardware/lib/allegro_hand_v6_hardware/test_v6_hardware

# 3. 실물 핸드 독립 모션 테스트 CLI
ros2 run allegro_hand_v6_hardware test_sdk_motion
```

---

## 6. 사용 방법 (Usage)

### 6.1. URDF 설정 (URDF Configuration)

로봇의 URDF/Xacro 파일에 `<ros2_control>` 태그를 추가하고 본 하드웨어 인터페이스 플러그인을 지정합니다.

```xml
<ros2_control name="allegro_hand" type="system">
    <hardware>
        <plugin>allegro_hand_v6_hardware/AllegroHandV6System</plugin>
        <param name="io_interface_descriptor">modbus_tcp:192.168.1.100:502</param>
        <param name="hand_id">1</param>
    </hardware>
    
    <!-- 20개 관절 정의 -->
    <joint name="joint00">
        <command_interface name="position"/>
        <command_interface name="effort"/>
        <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
    </joint>
    ...
</ros2_control>
```

### 6.2. 컨트롤러 실행 (Launching the Controller)

`ros2_control_node`와 제어기 스포너를 포함하는 런치 파일을 실행합니다. 상세 실행 예시는 `allegro_hand_v6_bringup` 패키지를 참조하십시오.

```bash
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py hand:=right ros2_control_hardware_type:=hardware io_interface_descriptor:=modbus_tcp:192.168.1.100:502
```

### 6.3. 상태 모니터링 (Checking Status)

컨트롤러가 실행 중일 때 터미널에서 다음 명령어로 하드웨어 상태 및 센서 데이터를 실시간 모니터링할 수 있습니다.

#### 1. 관절 상태 확인 (`/joint_states`)
`joint_state_broadcaster`에 의해 20개 관절의 위치, 속도, 노력이 발행됩니다.

```bash
ros2 topic echo /joint_states
```

#### 2. 촉각 압력 센서 확인 (`/allegro_hand/tactile_pressures`)
18개 촉각 센서(손가락 15개 + 손바닥 3개)의 실시간 압력값(단위: hPa)이 발행됩니다.

```bash
ros2 topic echo /allegro_hand/tactile_pressures
```

#### 3. IMU 데이터 확인 (`/allegro_hand/imu/data`)
BNO055 IMU의 쿼터니언 방향 데이터가 실시간 발행됩니다.

```bash
ros2 topic echo /allegro_hand/imu/data
```

#### 4. 진단 메시지 확인 (`/diagnostics`)
모터 온도, 센서 온도, 오류 플래그, 통신 주기가 `/diagnostics` 토픽으로 발행됩니다.

```bash
# 터미널에서 직접 확인
ros2 topic echo /diagnostics

# GUI 도구를 통한 모니터링 (권장)
ros2 run rqt_runtime_monitor rqt_runtime_monitor
# 또는
ros2 run rqt_robot_monitor rqt_robot_monitor
```

---

### 6.4. 파라미터 관리 (Parameters)

하드웨어 인터페이스는 제어 루프의 실시간성을 방해하지 않도록 별도의 보조 노드를 통해 ROS 2 파라미터 서버로 토크 제한값 및 초기 위치를 실시간 조회/튜닝할 수 있도록 지원합니다.

#### 파라미터 그룹
* **`torque_limit`** (관절별 최대 토크 한계값):
  * 모터 보호 및 악력 한계 설정을 위한 최대 토크값 (단위: Nm)
  * 예: `torque_limit/Thumb/joint00`
* **`initial_position`** (관절별 초기 위치):
  * 제어기 활성화 시 적용되는 기본 초기 자세 (단위: rad)
  * 예: `initial_position/Thumb/joint00`

#### 파라미터 조회 및 변경

```bash
# 1. 설정 가능한 전체 파라미터 목록 확인
ros2 param list /allegro_hand

# 2. 특정 관절의 토크 한계값 조회
ros2 param get /allegro_hand torque_limit/Thumb/joint00

# 3. 특정 관절의 토크 한계값 실시간 변경 (0.2 Nm)
ros2 param set /allegro_hand torque_limit/Thumb/joint00 0.2
```

#### GUI 기반 튜닝 도구 (`rqt_reconfigure` / `rig_reconfigure`)

```bash
# rqt_reconfigure 실행
ros2 run rqt_reconfigure rqt_reconfigure

# rig_reconfigure 실행 (빌드되어 있는 경우)
ros2 run rig_reconfigure rig_reconfigure
```

#### 파라미터 영구 저장

```bash
# 실시간 변경된 파라미터 값을 YAML 파일로 덤프 저장
ros2 param dump /allegro_hand --output-dir ./config
```

---

### 7. Python SDK 

ROS 2 없이 Python 환경(NumPy / PyTorch / Gym / Isaac Lab)에서 하드웨어를 직접 제어하고 데이터를 수집할 수 있는 고성능 CPython 바인딩(`allegro_hand_v6_sdk`)을 제공합니다.

#### 기본 사용 예시

```python
import allegro_hand_v6_sdk as sdk
import numpy as np

# 1. 통신 옵션 설정
opts = sdk.Options()
opts.comm_type = "tcp"
opts.ip_address = "192.168.1.100"
opts.tcp_port = 502

# 2. 핸드 인스턴스 생성 및 연결
hand = sdk.HandV6ModBus(opts)
print("Device Info:", hand.get_device_info())

# 3. 실시간 상태 읽기 (NumPy ndarray 반환)
positions, velocities, torques, imu_quat, pressures = hand.read_states()
print("Joint Positions (rad):", positions)
print("IMU Quaternion [w,x,y,z]:", imu_quat)
print("Tactile Pressures (18 sensors, hPa):", pressures)

# 4. 목표 제어 명령 쓰기 (위치/토크)
target_pos = np.zeros(20, dtype=np.float64)
hand.write_commands(position=target_pos)

# 5. 시스템 진단 상태 읽기
diag = hand.read_diagnostics()
print(f"Motor Temps: {diag.motor_temperatures}")
print(f"Torque Enabled: {diag.torque_enabled}")
```

#### 단독 검증 및 모션 스크립트 실행

```bash
source install/setup.bash

# 1. SDK 기본 기능 및 센서 데이터 읽기 검증
python3 src/ros_pkgs/src/hands/allegro_hand_v6/allegro_hand_v6_hardware/test/test_sdk.py

# 2. 5핑거 핑거 웨이브(Finger Wave) 연속 모션 테스트 (100 Hz 제어)
python3 src/ros_pkgs/src/hands/allegro_hand_v6/allegro_hand_v6_hardware/test/test_sdk_motion.py --ip 192.168.1.100 --rate 100 --freq 8.0
```
