# 이전 시도 기록 (참고용)

아래 내용은 롤백된 첫 번째 시도의 기록입니다. 현재 시험 패키지의 검증 결과가 아닙니다.
현재 상태와 알려진 LED 오류는 ../README.md를 참고하세요.

---

# VISION SENSOR SDK 교체 기록

## 범위

2026-09-15: `allegro_hand_vision_sensor`의 장치 탐색, 연결, 영상 수신, 해제를
`video.py`의 pyudev/OpenCV 직접 호출로 교체했습니다. SDK import, 설치 요구사항,
비공개 카메라 핸들 접근을 제거했습니다. SDK 소스/문서/udev 규칙을 배포물에 복사하지 않았습니다.
다른 패키지(`allegro_hand_vision_sensor_test` 포함)와 기존 설치된 SDK는 수정하지 않았습니다.
이전 build/install 산출물은 새 소스 상태를 반영하도록 재빌드해야 합니다.

## 구현 근거와 한계

- 장치 열거와 속성 접근: https://pyudev.readthedocs.io/en/latest/api/pyudev.html
- V4L2 노드와 재연결 시 장치 번호 변경:
  https://docs.kernel.org/userspace-api/media/v4l/open.html
- 영상 읽기·설정·해제:
  https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html
- 표준 zoom 제어 자체의 정의:
  https://docs.kernel.org/userspace-api/media/v4l/ext-ctrls-camera.html

이 작업 전에 설치된 digit-interface 0.2.1 소스를 검토했습니다.
따라서 소스를 보지 않은 개발자가 수행하는 엄격한 클린룸 구현이라고 주장하지 않습니다.
새 구현은 SDK의 클래스/API 구조를 재현하지 않고 위 표준 API로 작성했습니다.
영상 방향은 기존 애플리케이션의 출력 계약(세로 BGR, 미러)을 유지합니다.

장치별 호환성 정보인 VISION SENSOR 모델명, USB 시리얼 기반 선택, 펌웨어 2.00 이상에서
zoom 제어값 4095가 최대 RGB 조명이라는 사실은 이전 SDK 검토에서 확인했습니다.
이 조명 의미는 일반 V4L2 사양에 정의된 것이 아니며 독립적인 제조사 사양이나
실물 실험으로 재확인하지 못했습니다. 프로토콜 동작 정보와 저작권 보호를 받는 코드 표현을
구분하여 작성했지만, 법적 무위반 판정이나 제품 전체에 대한 상용 사용 승인은 아닙니다.
공개/상용 배포 검토에는 이 출처 기록도 함께 제공해야 합니다.

## 동작

- `ID_MODEL=DIGIT`, `ID_V4L_CAPABILITIES`에 `:capture:`가 있는 노드를 선택합니다.
  시리얼 없는 노드와 metadata 노드는 제외합니다. 중복 시리얼의 여러 capture 노드는
  임의 선택하지 않고 오류로 보고합니다.
- 연결할 때마다 다시 탐색합니다. USB 재연결 후 `/dev/videoN` 변경에 대응합니다.
- 기본 V4L2 백엔드, YUYV, 320×240, 30 FPS, BGR 변환을 요청합니다.
  필수 설정 실패는 오류로 보고하고 재시도합니다. 버퍼 1개 설정은 선택 힌트입니다.
  요청 성공은 실제 하드웨어 FPS 보증이 아니므로 실기 확인이 필요합니다.
- 최종 출력은 가로 240 × 세로 320의 미러된 portrait BGR 영상입니다.
- `force_v4l2=false`는 OpenCV 자동 백엔드이며 SDK로 되돌아가지 않습니다.
- `led_control=-2`(기본): USB revision 0x0200 이상에서 원시 zoom 값 4095를 설정합니다.
  구형 또는 revision 미확인 장치에서는 명시적인 설정을 요구합니다.
- `led_control=-1`: LED 제어를 하지 않고 현재 장치 상태를 유지합니다.
  `0..65535`: 원시 zoom 제어값입니다. 일반적인 밝기 백분율이 아닙니다.
  제어 요청 실패는 오류로 보고합니다. 구형 펌웨어의 자동 밝기 변환은 지원하지 않습니다.
- ROS 토픽·매핑·뷰어 형식 및 재연결 루프는 유지합니다.

## 검증

2026-09-15 결과:

- `colcon build --packages-select allegro_hand_vision_sensor` 성공, workspace 설치본 갱신.
- SDK import를 예외로 차단한 전체 pytest 실행: **27 passed**.
  ROS_DOMAIN_ID=187, ROS_LOCALHOST_ONLY=1, Qt offscreen 사용.
  시스템 NumPy/OpenCV를 우선하고, 시스템에 없는 pyudev는 사용자 설치본으로 검증했습니다.
- 실제 udev 열거에서 VISION SENSOR 5개와 USB revision 0x0200을 확인했습니다.
- 호스트 `/dev/video*`는 기존 프로세스가 사용 중이므로 새 코드로 열지 않았습니다.
  기존 캡처 프로세스를 중단하거나 재시작하지 않았습니다.

하드웨어 없는 테스트는 장치 필터링, 중복 식별, 영상 방향/형식, 설정 실패 시 자원 해제,
재연결 시 변경된 장치 경로 사용, 미확인 펌웨어 처리, SDK 없는 ROS 파이프라인을 대상으로 합니다.

실기 검증 필요:

1. SDK를 실행하지 않은 전원 투입 직후 센서별 조명과 영상 수신.
2. 이전 화면과 영상 방향·색상·밝기 비교.
3. 5개 센서 동시 수신의 실제 해상도, FPS, 지연, 장시간 동작.
4. USB 분리/재연결과 연결 순서 변경 후 손가락 매핑 유지.
5. 대상 PC의 udev capture 속성, USB revision, 권한 확인.

SDK 제거는 다른 의존성의 라이선스 의무를 없애지 않습니다.
