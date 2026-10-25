cmake_minimum_required(VERSION 3.8)
project(allegro_hand_v6_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz script
  DESTINATION share/${PROJECT_NAME}
)

install(FILES CSV_LOGGING.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  script/common.py
  script/finger_wave.py
  script/opening_releasing.py
  script/simple_motion.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
