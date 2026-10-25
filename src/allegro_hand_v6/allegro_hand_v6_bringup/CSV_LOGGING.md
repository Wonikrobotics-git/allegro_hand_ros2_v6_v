# 500 Hz CSV 데이터 로깅

`references/hands/allegro_hand_v6`의 CSV 로거에서 IMU 4열을 제외한 62열을 저장한다.
나머지 열 순서, 파일명, 단위, 숫자 정밀도, KST 시각 형식은 같다. 로깅 기본값은 꺼짐.

## 빌드 및 실행

현재 소스는 ROS 2 Jazzy 기준. `hands` 디렉토리에서 실행한다.
`--base-paths`로 실제 소스만 선택하여 `references`의 중복 패키지를 제외한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/jazzy build \
  --base-paths allegro_hand_common allegro_hand_v6 \
  --build-base build/jazzy --install-base install/jazzy \
  --symlink-install --packages-up-to allegro_hand_v6_bringup
source install/jazzy/setup.bash

ros2 launch allegro_hand_v6_bringup allegro_hand.launch.py \
  model:=v6_f hand:=right gui:=false \
  io_interface_descriptor:=modbus_tcp:192.168.1.100:502 \
  csv_logging:=true csv_directory:=./data_logs
```

`model`, `hand`, IP/포트는 연결한 장치에 맞춘다. `v6_f`/`v6_v`, 좌/우 모두
같은 CSV 스키마를 사용한다. USB 카메라 영상은 포함하지 않는다.
`mock_components`/`isaac`는 실제 SDK 읽기를 하지 않으므로 이 CSV를 생성하지 않는다.
경로에 공백이 있으면 `"csv_directory:=./my data"`처럼 인자 전체를 인용한다.

활성화마다 `allegro_hand_right_YYYYMMDD_HHMMSS_NNNNNNNNN.csv` 생성.
좌수는 `left`. 파일명 손 방향은 SDK가 읽은 장치 정보를 사용한다.
시작 메시지에서 절대 저장 경로 확인 가능. 기존 파일은 덮어쓰지 않는다.
`Ctrl+C` 종료 시 모터 토크를 끈 뒤 대기 중인 로그를 저장하고
`파일명.csv.summary.json`에 최종 카운터를 기록한다.

## 출력 형식 (IMU 제외)

| 열 | 내용 |
|---|---|
| `timestamp` | SDK 읽기 종료 직후 PC 시각. KST, `YYYY-MM-DD HH:MM:SS.sss` |
| `sequence` | 1부터 시작하는 읽기 시도 번호 |
| `read_ok` | 62개 fast-block 레지스터를 모두 읽으면 `1`, 실패하면 `0` |
| `dropped_total` | 버퍼 초과 또는 저장 중단/실패로 큐에 넣지 못한 누적 행 수 |
| `joint00_position_rad` … `joint43_position_rad` | 20관절 위치, rad |
| `joint00_current_ma` … `joint43_current_ma` | 20관절 부호 있는 실측 전류, mA |
| `pressure_0_hpa` … `pressure_17_hpa` | 펌웨어 순서의 압력 18채널, hPa |

관절 순서는 엄지 → 검지 → 중지 → 약지 → 소지, 손가락마다 4관절.
관절 prefix를 설정하면 CSV 관절 열 이름에도 같은 prefix가 붙는다.
전류는 SDK 토크를 `TORQUE_CONSTANT`로 나눈 뒤 반올림하여 원래 mA 정수로 복원한다.
58개 측정값은 한 번의 fast-block 읽기 결과다. IMU와 저속 진단 온도는 CSV에 포함하지 않는다.
SDK의 fast-block 읽기는 IMU를 포함한 기존 62개 레지스터를 그대로 사용한다.
읽기 실패 시에도 `read_ok=0` 행을 남기되 측정값 58칸은 비운다.

시각 예: `2026-10-22 10:30:15.123`. 소수점 아래 정확히 3자리,
밀리초 미만은 버림. ROS `/clock` 및 프로세스 timezone 설정과 무관하다.
이 시각은 장치 내부 샘플 시각이 아닌 PC 수신 시각이다.
절대 시각의 정확도는 PC 시계 동기화에 따르며, 시계 보정 시 행 간격이 변할 수 있다.
Unix/monotonic 시각 열은 추가하지 않는다.

## 500 Hz 동작 및 확인

`config/single_hand/ros2_controllers.yaml`의 `update_rate: 500`으로
2 ms마다 하드웨어 읽기와 기록을 시도한다. 별도 타이머의 복제 샘플이나
ROS 토픽 재수집 없이 제어 루프에서 직접 수집한다.
8192행 사전 할당 큐에 샘플을 복사하고, 별도 스레드에서 CSV 변환과 파일 쓰기를 수행한다.
로거의 생산자 경로에는 디스크 I/O, 동적 할당, mutex 대기가 없다.
큐가 가득 차면 제어 루프를 멈추지 않고 새 행을 버리며 카운터를 증가시킨다.
저장 스레드는 매초 및 정상 종료 시 stdio 버퍼를 flush한다.
강제 종료/전원 차단 시 버퍼 데이터가 손실될 수 있다. 행별 `fsync`는 수행하지 않는다.

실제 수집률은 장치 응답·통신·OS 스케줄링에 영향을 받으므로 실장비에서 확인한다.
행을 인위적으로 늘려 500 Hz로 표시하지 않는다. 정상 구간의 timestamp 간격은 약 2 ms,
평균 수집률은 `(행 수 - 1) / (마지막 시각 - 첫 시각)`으로 확인한다.
밀리초 표기와 PC 시계 보정 때문에 짧은 구간보다 수 초 이상 구간으로 평가한다.
`sequence` 누락, `read_ok=0`, summary의 `dropped_samples`, `failed_reads`,
`write_failed`도 함께 확인한다. 저장 오류와 큐 초과는 ROS 상태 타이머에서도 보고한다.

장치 없이 CSV 형식·500 Hz 입력·종료 저장·버퍼 초과·디스크 오류 및
SDK fast-block 단위 변환·읽기 실패 테스트:

```bash
colcon --log-base log/jazzy test \
  --build-base build/jazzy --install-base install/jazzy \
  --packages-select allegro_hand_v6_hardware \
  --ctest-args -R '^test_(csv_logger|sdk_read)$' --output-on-failure
```
