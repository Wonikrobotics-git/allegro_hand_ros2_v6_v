# Allegro Hand Vision Sensor

`allegro_hand_vision_sensor`는 udev와 OpenCV/V4L2로 영상 센서를 탐색·취득하고 시리얼 번호를 손가락에 매핑합니다. 외부 `digit-interface` SDK는 필요하지 않습니다. 하드웨어의 udev 값 `ID_MODEL=DIGIT`은 장치 탐색에 사용합니다. `/dev/videoN` 번호나 특정 시리얼 목록은 고정하지 않습니다.

## 설치와 확인

Ubuntu 24.04 / ROS 2 Jazzy 기준. 별도 기본 V6 패키지를 먼저 빌드한 뒤 [저장소 설치 안내](../README.md)에 따라 UI 전용 워크스페이스에 설치합니다. UI 패키지는 `src/allegro_hand_v6_ui` 안에 그대로 둡니다. 각 터미널에서 다음 환경을 불러옵니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/allegro_v6_ws/install/setup.bash
source ~/v6_ui_ws/install/setup.bash
```

## 영상 취득

```bash
ros2 launch allegro_hand_vision_sensor vision_sensor.launch.py
```

기본값은 연결된 모든 카메라의 영상을 `/vision_sensor/s_<시리얼 UTF-8 hex>/image_raw`에 발행합니다. 발견한 시리얼은 `/vision_sensor/connected_serials`에 발행합니다. 카메라 하나를 확인할 때는 `target_serial:=<시리얼>`, 캡처 없이 launch 인자만 확인할 때는 `--show-args`를 사용합니다.

**대시보드는 카메라를 직접 엽니다.** 이 캡처 노드와 `allegro_hand_v6_dashboard`를 함께 실행하지 않습니다. 아래 매핑 launch는 자체 캡처 노드를 시작합니다.

## 카메라와 손가락 매핑

실제 손 연결 설정과 장치 접근 권한을 먼저 준비합니다. 터미널 1에서도 위의 환경을 source합니다.

```bash
# 터미널 1: 실제 손과 컨트롤러 시작
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py model:=v6_v hand:=left
```

손의 통신 주소가 기본값 `192.168.1.100:502`와 다르면 bringup에 `io_interface_descriptor`를 지정합니다. 예를 들어 `192.168.1.110`의 오른손은 다음과 같이 실행합니다.

```bash
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py model:=v6_v hand:=right io_interface_descriptor:=modbus_tcp:192.168.1.110:502
```

컨트롤러 활성화 후 같은 워크스페이스 환경과 ROS 도메인의 다른 터미널에서 실행합니다.

```bash
# 터미널 2: 카메라 캡처와 자동 매핑 시작
source /opt/ros/jazzy/setup.bash
source ~/allegro_v6_ws/install/setup.bash
source ~/v6_ui_ws/install/setup.bash
ros2 launch allegro_hand_vision_sensor mapping.launch.py hand:=left
```

`mapping.launch.py`는 기본적으로 접촉 기반 sweep 매핑을 실행하며, `mapping_sweep.launch.py`도 같은 동작과 옵션을 제공합니다. 왼손·오른손 모두 공통 설정인 `config/mapping_motion.yaml`의 각도를 준비 자세와 sweep 방향의 기준으로 사용합니다. 기본 `sweep_joints:=flex`에서는 엄지와 벌림 관절을 준비 자세로 이동한 뒤 검사할 손가락의 굽힘 관절을 접촉까지 점진적으로 움직입니다. 접촉, 추종 오차 한계 또는 `s_max`에 도달하면 멈춥니다. `hand: both`는 이 자세를 양손에 적용한다는 뜻이며, launch의 `hand:=left` 또는 `hand:=right`는 매핑 결과와 화면 표시 방향을 지정합니다. 기존 `mapping_motion_left.yaml`과 `mapping_motion_right.yaml`은 공통 파일을 가리키는 심볼릭 링크입니다. 각도를 바꾸려면 공통 파일 하나만 수정합니다. 매퍼는 실제 관절 위치 명령을 발행하므로 실행 전 접촉 자세를 확인합니다. 동작 설정은 설치된 패키지의 share 디렉터리에서 읽습니다. 설정을 수정했다면 패키지를 다시 빌드하거나 `motion_file:=/absolute/path/to/motion.yaml`을 전달합니다. 사용자 지정 파일도 `hand: both`로 양손에 적용할 수 있으며, `hand: left` 또는 `hand: right`인 기존 파일은 해당 손에만 적용됩니다. `DONE`과 저장 경로를 확인한 뒤 캡처를 종료합니다. `FAILED` 또는 `UNRESOLVED` 손가락이 있으면 카메라와 자세를 확인하고 다시 실행합니다.

두 매핑 launch 모두 `vision_sensor.launch.py`를 자동으로 실행한 뒤 기본 5초 후 매퍼를 시작합니다. 이후에도 카메라 영상, 활성 컨트롤러, 관절 피드백이 준비될 때까지 기다립니다. 접촉 판정 기본값은 `contact_threshold:=10.0`이며, 다음 명령 하나로 센서 캡처와 sweep 매핑을 시작할 수 있습니다.

```bash
ros2 launch allegro_hand_vision_sensor mapping.launch.py hand:=right
```

민감도를 바꾸려면 `contact_threshold:=12.0`처럼 지정합니다. 시작 대기 시간은 `mapper_start_delay:=5.0`으로 조절합니다. 캡처가 이미 실행 중이면 `capture:=false mapper_start_delay:=0.0`을 지정하여 기존 영상을 사용합니다. `mapping_sweep.launch.py`에서도 동일한 옵션을 사용할 수 있습니다.

### 주요 매핑 옵션

| 옵션 | 기본값 | 의미 |
|---|---|---|
| `contact_threshold` | `10.0` | 기준 영상 대비 변화 점수의 접촉 판정 기준. 힘(N) 단위가 아님 |
| `contact_frames` | `3` | 접촉으로 판단하는 연속 프레임 수 |
| `mapper_start_delay` | `5.0` | 캡처 시작 후 매퍼 실행까지 대기 시간(초) |
| `sweep_joints` | `flex` | 검사할 손가락의 굽힘 관절만 sweep. `all`은 엄지와 해당 손가락을 함께 sweep |
| `s_max` | `1.3` | sweep 진행 상한. `1.0`은 YAML에 저장된 기준 접촉 각도 |
| `approach_velocity` | `0.1` | `s_fast`(기본 `0.7`) 이후의 접근 속도(rad/s) |
| `max_tracking_error` | `0.3` | 이동 명령과 실제 관절 각도의 오차 한계(rad) |

### 로그와 결과 확인

- `contact_preshape` / `blocked_preshape`: 준비 자세 이동 중 접촉 또는 추종 오차를 감지하여 해당 손가락을 건너뛰고 ready 자세로 복귀합니다. 움직이다 갑자기 돌아오는 경우 이 로그를 확인합니다.
- `Tracking block`: 문제 관절과 명령·실제 각도·오차를 출력합니다. 이 로그만으로 관절 범위를 벗어났다고 단정할 수 없습니다.
- `MATCH`: 관찰 구간에서도 지속적이고 고유한 센서 응답을 확인한 손가락입니다. `UNRESOLVED`는 매핑되지 않은 손가락이며, `DONE`만으로 모든 매핑이 성공했다는 뜻은 아닙니다.

5개 센서가 모두 참여하고 검지·중지·약지·소지가 확인되어야 남은 센서를 엄지로 추론합니다. 일부 센서만 연결된 경우에도 매핑은 진행하지만 엄지는 미확정으로 남습니다. 실행 시작 후 추가 연결된 센서는 현재 매핑에 포함되지 않으므로 다음 실행에서 다시 확인합니다.

실행별 sweep 기록은 매핑 파일과 같은 디렉터리의 `sweep_report_<hand>.yaml`에 누적되며, `report_file:=/absolute/path/report.yaml`로 위치를 바꿀 수 있습니다. `DONE` 후에도 캡처는 실행 중이므로 대시보드를 켜기 전에 매핑 터미널에서 `Ctrl+C`로 종료합니다.

결과는 기본적으로 `${XDG_CONFIG_HOME:-~/.config}/allegro_hand_vision_sensor/finger_serials.yaml`에 저장하며 대시보드도 같은 파일을 읽습니다. 매퍼가 상위 디렉터리를 만듭니다. 매핑과 대시보드를 같은 OS 사용자와 같은 `XDG_CONFIG_HOME` 값으로 실행합니다. 이 장비별 파일은 Git과 설치 패키지에서 제외하며 `finger_serials.example.yaml`만 배포합니다. 다른 위치를 사용하려면 매핑과 대시보드 launch 양쪽에 같은 `mapping_file:=/absolute/path/finger_serials.yaml`을 지정합니다. 기존 소스 트리의 `config/finger_serials.yaml`은 새 경로로 복사하거나 경로를 직접 지정합니다. 대시보드 실행 전에 터미널 2의 캡처를 종료합니다. 전체 순서는 `allegro_hand_v6_dashboard` README를 참고합니다.

교체한 카메라는 새 시리얼로 발견될 수 있지만 기존 손가락 대응은 자동 승계되지 않습니다. 다른 실제 손에도 별도 매핑이 필요합니다. 교체 후 매핑을 확인하고 다시 실행합니다.

## 테스트

```bash
colcon test --packages-select allegro_hand_vision_sensor
colcon test-result --test-result-base build/allegro_hand_vision_sensor --verbose
```

SDK 교체 과정과 당시 검증 기록은 [docs/SDK_REPLACEMENT.md](docs/SDK_REPLACEMENT.md)에 보존했습니다.

[English README](README.md)
