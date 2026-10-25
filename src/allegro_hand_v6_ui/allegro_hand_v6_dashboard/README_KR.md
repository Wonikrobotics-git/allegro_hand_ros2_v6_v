# Allegro Hand V6 대시보드

## Jazzy 설치와 실행

Ubuntu 24.04 / ROS 2 Jazzy 기준. 별도 기본 V6 패키지를 먼저 빌드한 뒤 [저장소 설치 안내](../README.md)에 따라 UI 전용 워크스페이스에 설치한다. UI 패키지는 `src/allegro_hand_v6_ui` 안에 그대로 둔다. 각 터미널에서 다음 환경을 불러온다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/allegro_v6_ws/install/setup.bash
source ~/v6_ui_ws/install/setup.bash
```

두 터미널에서 위 환경을 source한 뒤 다음과 같이 실행한다. 아래 bringup은 실제 장치 없이 동작한다.

```bash
# 터미널 1: 최신 V6 V 왼손 모델, TF, mock 관절 상태
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  model:=v6_v hand:=left ros2_control_hardware_type:=mock_components gui:=false

# 터미널 2: 대시보드
ros2 launch allegro_hand_v6_dashboard dashboard.launch.py hand:=left
```

대시보드의 `hand`는 2D 이미지·센서 배치를 선택한다. 3D 모델은 bringup이 `/robot_description`으로
발행하는 URDF와 TF를 사용하므로 **bringup에도 `hand:=left`를 지정해야 한다**.
`model:=v6_f`는 F 모델, `model:=v6_v`는 V 모델을 선택한다.
대시보드 상단에 실제 수신한 모델·손 방향이 표시된다. `Waiting`은 URDF 미수신,
`Legacy or custom URDF`는 구형 또는 별도 URDF, 손 방향 불일치 메시지는 bringup 인자 확인이 필요함을 뜻한다.
구형 `R10_Link` 계열 모델이 계속 나오면 기존 robot_state_publisher/bringup을 종료한 뒤 새 환경에서 다시 시작한다.
기존 publisher와 새 bringup을 같은 토픽에 동시에 실행하지 않는다.

`sensor_config:=config/tactile_layout_left.yaml`, `image_path:=assets/V6_left_new.png`처럼
패키지 share 기준 상대경로도 지정할 수 있다. URDF mesh의 `package://allegro_hand_v6_description/...`
경로는 ROS 패키지 경로이므로 유지한다. 압력 마커는 수신 URDF의 링크 목록에 맞춰 기존 `R*_Link`를
새 센서·손가락 프레임으로 변환한다. 손바닥은 `palm_s1..3`, F 모델 손가락은 `_s1..3`,
V 모델 손끝은 `_s1` 센서 프레임을 사용한다. 화살표는 센서 표면 바깥쪽을 향하며,
V 모델의 중간·기저 압력 채널은 해당 손가락 링크 앞면에 배치한다.
구형 URDF에는 기존 설정을 유지한다. 기본 카메라 거리는 `0.70`, 초점 높이는 `0.11`이다.
`view_distance:=0.60`처럼 실행 인자로 확대 정도를 조절할 수 있다.

`allegro_hand_v6_dashboard`는 손바닥 압력 3채널, 영상 센서 5개, RViz 3D 손 모델을 하나의 Qt 창에 표시합니다. C++ 스레드가 udev와 OpenCV/V4L2로 각 카메라를 직접 엽니다. 손바닥 압력은 `/allegro_hand/tactile_pressures`에서 읽으며 압력 데이터가 없어도 연결된 카메라 영상은 표시합니다. `digit-interface` SDK는 필요하지 않습니다.

## 최초 사용: 카메라와 손가락 매핑

매핑 파일 이름은 `finger_serials.yaml`이며 실제 손 한 대의 시리얼을 담습니다. 파일이 없거나 비어 있으면 대시보드가 발견한 카메라를 손가락 카드에 임시로 배치하므로 위치가 달라질 수 있습니다. 대시보드 사용 전에 실제 손에서 시리얼 대응을 만듭니다.

1. 손과 카메라 5개를 연결합니다. 양손 공통 설정인 `mapping_motion.yaml`의 준비 자세와 굽힘 방향을 확인합니다. `mapping_motion_left.yaml`과 `mapping_motion_right.yaml`은 공통 파일을 가리키는 심볼릭 링크입니다. 기본 매핑은 실제 관절을 점진적으로 움직이며 접촉을 감지하는 sweep 방식입니다. 수정한 설정을 사용할 때는 `mapping.launch.py`에 `motion_file:=/absolute/path/to/motion.yaml`을 전달합니다.
2. 터미널 1에서 손과 컨트롤러를 시작합니다.

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/allegro_v6_ws/install/setup.bash
   source ~/v6_ui_ws/install/setup.bash
   ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py model:=v6_v hand:=left
   ```

3. 컨트롤러 활성화 후 같은 ROS 도메인의 터미널 2에서 다음 명령 하나로 센서 캡처와 sweep 매핑을 시작합니다. `vision_sensor.launch.py`가 자동으로 실행되고 기본 5초 뒤 매퍼가 시작됩니다. 접촉 판정 기본값은 `contact_threshold:=10.0`입니다. 별도로 센서 캡처를 실행할 필요가 없습니다.

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/allegro_v6_ws/install/setup.bash
   source ~/v6_ui_ws/install/setup.bash
   ros2 launch allegro_hand_vision_sensor mapping.launch.py hand:=left
   ```

4. `DONE`과 저장 경로를 확인한 뒤 터미널 2에서 `Ctrl+C`로 매핑 launch와 센서 캡처를 종료합니다. `FAILED` 또는 `UNRESOLVED` 손가락이 있으면 연결과 준비 자세, 접촉 응답, 관절 추종 오차를 확인하고 다시 실행합니다. 결과 YAML의 `hand` 값과 손가락 5개의 시리얼을 확인합니다. 오른손은 모든 `hand` 인자에 `right`를 사용합니다. 세부 옵션과 로그 해석은 [비전 센서 매뉴얼](../allegro_hand_vision_sensor/README_KR.md)을 참고합니다.

모든 PC에서 기본 매핑 경로는 `${XDG_CONFIG_HOME:-~/.config}/allegro_hand_vision_sensor/finger_serials.yaml`입니다. `XDG_CONFIG_HOME`이 없으면 홈 디렉터리를 사용합니다. 매퍼가 상위 디렉터리를 만듭니다. 두 launch를 같은 OS 사용자와 같은 `XDG_CONFIG_HOME` 값으로 실행합니다. 다른 위치를 사용하려면 양쪽에 같은 `mapping_file:=/absolute/path/finger_serials.yaml`을 지정합니다. 기존 소스 트리의 `config/finger_serials.yaml`은 새 경로로 복사하거나 `mapping_file`로 직접 지정해야 합니다.

## 실제 손과 실행

두 터미널에서 같은 ROS 도메인을 사용하고 `/opt/ros/jazzy/setup.bash`와 워크스페이스의 `install/setup.bash`를 source합니다.

```bash
# 터미널 1: 손, 컨트롤러, robot_description, TF
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py model:=v6_v hand:=left

# 터미널 2: 카메라, 손바닥 압력, 실시간 3D 손
ros2 launch allegro_hand_v6_dashboard dashboard.launch.py hand:=left
```

대시보드는 별도로 실행합니다. 손의 통신 주소가 기본값 `192.168.1.100:502`와 다르면 bringup 명령에 `io_interface_descriptor`를 지정합니다. 예를 들어 `192.168.1.110`의 손에는 `io_interface_descriptor:=modbus_tcp:192.168.1.110:502`를 전달합니다. 대시보드는 USB 카메라를 직접 열므로 매퍼의 `vision_sensor_capture`와 다른 카메라 프로그램을 먼저 종료합니다. 매핑 YAML과 대시보드의 `hand` 인자를 일치시킵니다. 서로 다른 실제 손은 각 PC에서 다시 매핑하거나 해당 손의 매핑 파일을 복사합니다.

## 프레임 무결성

카메라 리더는 인접 행의 BGR 평균 차이가 절대 기준 15와 중앙값의 8배를 모두 넘는 경계를 찾습니다. 5행 이내 후보는 하나의 경계로 묶습니다. 경계가 하나이면 위아래 조각을 교환하고 재검사합니다. 복수 경계 또는 보정 실패 프레임은 폐기합니다. 마지막 승인 프레임을 최대 1초 표시한 뒤 `NO SIGNAL`로 바꿉니다. 연속 3프레임이 폐기되면 카메라를 다시 엽니다.

필요하면 대시보드 launch의 `vision_sensor_tear_absolute_threshold`, `vision_sensor_tear_ratio_threshold`, `vision_sensor_tear_reopen_count`, `vision_sensor_timeout` 인자로 기준을 조정합니다.

## 테스트와 압력 채널 설정

```bash
colcon test --packages-select allegro_hand_v6_dashboard
colcon test-result --test-result-base build/allegro_hand_v6_dashboard --verbose
```

실제 V 모델의 압력 채널 수나 순서가 기본값과 다르면 `pressure_channel_count`와 `pressure_indices`를 지정합니다. 기본 압력 토픽은 `/allegro_hand/tactile_pressures`이고 `pressure_topic`으로 변경할 수 있습니다.

[English README](README.md)
