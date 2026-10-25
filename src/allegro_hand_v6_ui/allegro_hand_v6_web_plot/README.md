# Allegro Hand V6 Web Plot

Allegro Hand V6의 관절 위치, 목표 위치, effort, 온도, 촉각 값을 웹 브라우저에서 실시간 확인하는 ROS 2 monitoring 패키지이다. Python bridge가 ROS 토픽을 수집해 WebSocket JSON으로 변환하고 정적 HTML 화면이 이를 그래프로 표시한다.

## 디렉터리 및 파일 구성

```text
allegro_hand_v6_web_plot/
├── scripts/
│   └── web_plot_bridge.py
├── web/
│   └── index.html
├── CMakeLists.txt
├── package.xml
└── README.md
```

| 경로 | 기능 |
|---|---|
| `scripts/` | ROS-to-Web bridge Python node 보관 |
| `web/` | 브라우저 정적 page와 설치 시 복사되는 logo 보관 |
| `scripts/web_plot_bridge.py` | ROS subscriber, HTTP server, WebSocket server, 30 Hz JSON broadcast 구현 |
| `web/index.html` | 관절 선택 UI, 10초 시계열 canvas plot, 촉각 bar 화면 구현 |
| `CMakeLists.txt` | Python executable과 web asset 설치 |
| `package.xml` | ROS 2 package 정보와 Python runtime 의존성 |
| `README.md` | 패키지 구조, interface, web monitor 실행법 문서 |

`scripts/__pycache__/`와 `*.pyc`는 Python이 만든 cache이며 패키지 기능 파일이 아니다.

Logo는 선택 사항이다. `web/WR_EN_high.png`를 추가하면 다른 web asset과 함께 설치된다.
파일이 없어도 빌드와 모니터링이 가능하며 외부 V5 소스 트리를 참조하지 않는다.

## 패키지 담당 기능

- 네 개 ROS 2 토픽을 하나의 상태 구조로 수집한다.
- `/joint_states` 배열을 관절 이름 기준으로 `joint00..joint43` 순서로 정렬한다.
- 정적 web 파일을 HTTP로 제공한다.
- 최신 상태를 WebSocket JSON으로 약 30 Hz 전송한다.
- Browser에서 최근 10초 데이터를 관절별로 선택해 표시한다.
- 촉각 데이터는 별도 Sensors 탭에서 bar로 표시한다.

데이터 흐름:

```text
ROS 2 topics
    ↓
web_plot_bridge.py
    ├── HTTP :8080 ─────→ index.html
    └── WebSocket :9091 → 실시간 JSON → browser chart
```

실행 node와 executable 이름:

| 구분 | 이름 |
|---|---|
| Executable | `web_plot_bridge` |
| Node | `allegro_hand_v6_web_plot` |

## ROS 2 토픽

이 패키지는 ROS 토픽을 발행하지 않고 다음 토픽을 구독한다.

| 방향 | 토픽 | 자료형 | 사용하는 필드 | 기능 |
|---|---|---|---|---|
| Subscribe | `/joint_states` | `sensor_msgs/msg/JointState` | `name`, `position`, `effort` | 현재 관절 위치와 effort 수집 |
| Subscribe | `/allegro_hand_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | `data[0:20]` | 최근 20관절 목표 위치 수집 |
| Subscribe | `/allegro_hand_v6/joint_temperatures` | `std_msgs/msg/Float32MultiArray` | `data[0:20]` | 최근 20관절 온도 수집 |
| Subscribe | `/allegro_hand_v6/tactile_sensors` | `std_msgs/msg/Float32MultiArray` | `data` 전체 | 촉각 센서 값 수집 |

### 관절 데이터 순서와 단위

- 관절 이름 순서: `joint00..joint03`, `joint10..joint13`, ..., `joint40..joint43`
- `position`: `/joint_states.position`, 단위 rad
- `desired`: position controller command, 단위 rad
- `current`: 내부 JSON key와 브라우저 탭 이름이다. 실제 데이터는 `/joint_states.effort`이며 현재 V6 hardware에서는 토크 `N·m`이다.
- `temperature`: `/allegro_hand_v6/joint_temperatures`, 단위 °C
- `pressure`: 촉각 배열 전체. Bridge는 길이나 단위를 변환하지 않는다.

`/joint_states`의 resource 순서는 보장되지 않으므로 이름을 기준으로 재정렬한다. 누락된 관절이나 NaN 값은 WebSocket JSON에서 `null`로 전달된다.

## WebSocket JSON 형식

WebSocket server는 다음 형태의 최신 snapshot을 전송한다.

```json
{
  "names": ["joint00", "joint01", "...", "joint43"],
  "position": [0.0, 0.0, "... 20 values ..."],
  "current": [0.0, 0.0, "... 20 values ..."],
  "desired": [0.0, 0.0, "... 20 values ..."],
  "temperature": [25.0, 25.0, "... 20 values ..."],
  "pressure": ["... all received tactile values ..."],
  "timestamps": {
    "joint": 0.0,
    "desired": 0.0,
    "temperature": 0.0,
    "pressure": 0.0
  }
}
```

각 timestamp는 ROS clock 기준 초 단위이며 해당 종류의 마지막 message 수신 시각이다. 아직 수신하지 않은 데이터 종류는 `null`이다.

## Browser 화면

| 탭 | 표시 내용 |
|---|---|
| `Angles` | 현재 position 실선과 desired position 점선 |
| `Current` | `/joint_states.effort` 값 |
| `Temperature` | 20관절 온도 |
| `Sensors` | 촉각 배열 bar |

- 그래프 window는 최근 10초이다.
- Thumb, Index, Middle, Ring, Pinky 단위 또는 개별 관절을 선택할 수 있다.
- 초기 선택은 Thumb의 네 관절이다.
- Y축은 현재 보이는 유효 값 범위에 따라 자동 조절된다.
- WebSocket 연결이 끊기면 1.5초 후 재연결한다.

### 현재 촉각 bar 동작

Browser는 받은 pressure 배열을 5개 손가락 group에 가능한 한 균등하게 나눈다. Dashboard의 18채널 YAML layout을 읽지 않으므로 손바닥 센서와 `thumb → index → middle → pinky → ring → palm` 의미를 정확히 재현하지 않는다.

또한 bar 높이는 현재 `0..30` 범위를 기준으로 계산한다. `1000..1200 hPa` V6 데이터를 직접 넣으면 bar가 상한에 포화된다. 원본 숫자는 그대로 표시된다. 정확한 V6 촉각 위치와 hPa 색상 mapping은 `allegro_hand_v6_dashboard`를 사용한다.

## ROS parameter 및 network port

| Parameter | 기본값 | 기능 |
|---|---:|---|
| `http_port` | `8080` | `index.html`과 asset을 제공하는 HTTP port |
| `ws_port` | `9091` | 실시간 JSON WebSocket server port |

두 server는 `0.0.0.0`에 bind하므로 같은 network의 다른 장치에서도 접속할 수 있다.

> **보안 주의:** HTTP와 WebSocket에 인증 및 TLS가 없다. 신뢰할 수 있는 내부 network에서만 사용하고 외부 network에 직접 노출하지 않는다.

`web/index.html`의 WebSocket URL은 현재 port `9091`로 고정되어 있다. `ws_port` parameter만 변경하면 browser가 새 port를 알 수 없으므로 `index.html`의 URL도 함께 수정해야 한다.

## 실행 방법

### 기본 bringup 실행 후 별도 모니터 시작

```bash
# 터미널 1
ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py model:=v6_v hand:=left gui:=false

# 터미널 2
ros2 run allegro_hand_v6_web_plot web_plot_bridge
```

실행 후 같은 PC에서 다음 주소를 연다.

```text
http://localhost:8080
```

### Web Plot node만 실행

```bash
ros2 run allegro_hand_v6_web_plot web_plot_bridge
```

Port 지정 예:

```bash
ros2 run allegro_hand_v6_web_plot web_plot_bridge \
  --ros-args -p http_port:=8088
```

설치와 각 터미널의 환경 설정은 [저장소 README](../README.md)를 따른다. Web bridge는 기본 bringup과 별도로 실행한다.

## 빌드

```bash
colcon build --packages-select allegro_hand_v6_web_plot --symlink-install
```

Runtime에는 `rclpy`, `ament_index_python`, `sensor_msgs`, `std_msgs`, Python `websockets`가 필요하다.
