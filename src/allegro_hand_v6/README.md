# Allegro Hand V6 ROS 2 Package Suite

본 디렉토리는 5지 20관절(5-Finger, 20-DOF) 다지 로봇 손인 **Allegro Hand V6**의 기구학 URDF 모델, ROS 2 컨트롤러 런치 환경, 그리고 물리 MCU와 2Mbps 시리얼 통신을 수행하는 `ros2_control` C++ 하드웨어 인터페이스 패키지를 포함하는 최상위 스위트임.

## ROS 2 Jazzy에서 빌드 및 실행

Ubuntu 24.04 / ROS 2 Jazzy 기준. 아래 명령은 `ros_pkgs/src/hands`에서 실행한다.
Humble로 빌드한 기존 `build`, `install` 환경은 source하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/jazzy build \
  --base-paths allegro_hand_common allegro_hand_v6 \
  --build-base build/jazzy --install-base install/jazzy \
  --symlink-install
source install/jazzy/setup.bash
```

V6와 공통 패키지만 빌드하므로 V4 전용 grasp 라이브러리 및 Pinocchio가 필요하지 않다.
Jazzy 하드웨어 플러그인은 `on_init(const hardware_interface::HardwareComponentInterfaceParams&)` API를 사용한다.

의존성 설치가 필요한 PC에서는 빌드 전에 다음 명령을 실행한다. `rosdep`을 처음 쓰는
PC는 먼저 `sudo rosdep init`, `rosdep update`를 실행해야 한다.

```bash
rosdep install --from-paths allegro_hand_common allegro_hand_v6 \
  --ignore-src --rosdistro jazzy -r -y
```

`wr_topic_based_ros2_control`은 `ros2_control_hardware_type:=isaac`에서만 사용한다.
실제 장치와 mock 실행에는 필요하지 않으며, Isaac 연동 시 Jazzy 호환 버전을 별도로 준비한다.

장치 없이 controller manager와 관절 상태 발행을 확인하려면:

```bash
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  ros2_control_hardware_type:=mock_components gui:=false
```

실제 장치 실행 예시. IP/포트, `hand`와 `model`은 연결한 장치에 맞춘다.
이 명령은 실제 하드웨어 인터페이스를 활성화한다.

```bash
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  ros2_control_hardware_type:=hardware \
  io_interface_descriptor:=modbus_tcp:192.168.1.100:502 \
  model:=v6_f hand:=right gui:=false
```

플러그인 로딩·초기화·인터페이스 등록만 검사하는 장치 없는 테스트:

```bash
ros2 run allegro_hand_v6_hardware test_v6_hardware \
  --gtest_filter=AllegroHandV6InterfaceTest.PluginAliasAllegroHandV6System
```

`test_v6_sdk`, `PluginLoadAndLifecycle`, `test_sdk_motion`은 실제 장치에 접근하므로
연결 상태와 주변 환경을 확인한 뒤 별도로 실행한다. 전체 `colcon test`도 이 테스트들을 실행한다.

---

## 500 Hz CSV 데이터 로깅

실제 장치 런치에 `csv_logging:=true csv_directory:=./data_logs` 추가.
제어/수집 주기는 `500 Hz`이며, 레퍼런스에서 IMU 4열을 제외한 62열 CSV 및 KST 밀리초 시각으로 저장한다.
로깅 기본값은 꺼짐. 실행 예시·열 순서·수집률 확인은
[CSV_LOGGING.md](allegro_hand_v6_bringup/CSV_LOGGING.md) 참고.

## 📁 디렉토리 및 패키지 구성

```
allegro_hand_v6/
├── allegro_hand_v6_bringup/       # ROS 2 런치 파일, 컨트롤러 파라미터 및 RViz 시각화 환경
├── allegro_hand_v6_description/   # Xacro/URDF 로봇 모델, 메시 파일 및 관절 좌표계 정의
└── allegro_hand_v6_hardware/      # ros2_control C++ 하드웨어 인터페이스 플러그인 및 드라이버
    ├── include/                   # 하드웨어 인터페이스 및 C++ 물리 드라이버 헤더
    ├── src/                       # C++ 구현체 및 Protobuf(hand_data_st.proto) 스키마
    ├── test/                      # 스탠드얼론 하드웨어 ROS 2 테스트 노드 (test_v6_hardware.cpp)
    └── test_cli/                  # 독립형 Python 상주 쉘 및 Rich TUI 모니터링 세트
```

### 1. `allegro_hand_v6_bringup`
- Allegro Hand V6 구동을 위한 `ros2_control_node`, `joint_state_broadcaster`, `forward_command_controller` 등의 런치 파일 및 설정 YAML 제공.

### 2. `allegro_hand_v6_description`
- 5개 손가락(Thumb, Index, Middle, Ring, Pinky 각 4관절)의 물리적 메쉬(STL/DAE) 및 Xacro 기구학 모델 정의.

### 3. `allegro_hand_v6_hardware`
- **`ros2_control` C++ 플러그인 (`AllegroHandV6System`)**: `hardware_interface::SystemInterface` 기반으로 구현되어 position 및 effort 제어 인터페이스를 수출함.
- **물리 통신 계층 연동**: `allegro_hand_common/allegro_hand_io` 추상 클래스를 통해 2Mbps 시리얼(`serial:/dev/ttyUSB0,2000000`) 및 Modbus(Placeholder) 통신 디스크립터를 동적으로 파싱함.
- **토크 및 전류 단위 변환**: `torque_constant` ($N\cdot m/mA$, 기본값: `0.00115`) 파라미터 및 `torque_limit` ($N\cdot m$, 기본값: `0.345`) 파라미터를 통해 ROS 2 표준 물리 단위와 MCU 통신 단위($mA$) 간 상호 변환 및 펌웨어 한계치 전송.

