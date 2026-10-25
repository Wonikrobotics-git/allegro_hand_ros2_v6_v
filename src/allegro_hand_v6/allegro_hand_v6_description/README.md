# allegro_hand_v6_description

Allegro Hand V6 (Right / Left) 로봇 묘사(Description) 및 자산 관리 패키지.

---

## 1. 패키지 구성 및 디렉터리 레이아웃
* **`urdf/`**:
  * `allegro_hand_v6_f_right.urdf` / `allegro_hand_v6_f_left.urdf`: F 모델 정적 URDF 및 `.xacro` 래퍼.
  * `allegro_hand_v6_v_right.urdf` / `allegro_hand_v6_v_left.urdf`: V 모델 정적 URDF 및 `.xacro` 래퍼.
  * `allegro_hand_v6_right.urdf.xacro` / `allegro_hand_v6_left.urdf.xacro`: 하위 호환 브릿지 Xacro (기본 `v6_f` 모델 연동).
* **`config/`**:
  * `joint_names_allegro_hand_v6.yaml`: V6 표준 관절 명칭 정의 파일.
* **`meshes/`**:
  * `v6_f/`: F 오른손/왼손 DAE 및 STL 메쉬 자산.
  * `v6_v/`: V 오른손/왼손 DAE 및 STL 메쉬 자산.
* **`rviz/`**:
  * `allegro_hand_v6.rviz`: RViz2 가시화 설정 파일.
* **`launch/`**:
  * `display.launch.py`: RViz2 및 `joint_state_publisher_gui`를 구동하는 런치 파일 (`model:=v6_f|v6_v`, `hand:=right|left` 지원).
* **`scripts/`**:
  * 시뮬레이션 USD 변환 및 플래트닝 스크립트.
* **`usd/`**:
  * Omniverse USD 자산 저장 디렉터리.

---

## 메시 레퍼런스 대응

| 모델 / 손 | 레퍼런스 | 패키지 메시 경로 |
|---|---|---|
| F / 왼손 | `references/F_Version_L/meshes` | `meshes/v6_f/left` |
| F / 오른손 | `references/F_Version_R/meshes` | `meshes/v6_f/right` |
| V / 왼손 | `references/V_Version_L/meshes` | `meshes/v6_v/left` |
| V / 오른손 | `references/V_Version_R/meshes` | `meshes/v6_v/right` |

충돌용 STL은 해당 레퍼런스 파일을 사용한다. 화면용 DAE 146개는
`references/V6_URDF_to_Customer_final/V6_URDF_to_Customer`의 모델별·좌우별
최종 원본을 사용하며, 형상·단위·재질 연결과 기존 STL의 정합성을 확인했다.
STL 갱신 시 화면용 DAE도 함께 확인한다.
모델별 URDF 경로, 관절 이름·원점·축·제한 및 질량/관성 설정은 기존 설정을 사용한다.

전원부의 은색 재질(RGBA `0.75 0.75 0.77 1`)을 포함해 각 부품의 색상과 재질은
위 최종 DAE에 지정된 값을 그대로 사용한다.

## 2. Viewer  

```bash
# 빌드 
colcon build --symlink-install && source install/setup.bash

# 오른손 모델  
ros2 launch allegro_hand_v6_description display.launch.py hand:=right

# 왼손 모델  
ros2 launch allegro_hand_v6_description display.launch.py hand:=left
```
