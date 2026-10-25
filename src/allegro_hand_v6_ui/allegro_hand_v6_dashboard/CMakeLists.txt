cmake_minimum_required(VERSION 3.8)
project(allegro_hand_v6_dashboard)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(UDEV REQUIRED IMPORTED_TARGET libudev)

set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME}_layout src/sensor_layout.cpp)
target_include_directories(${PROJECT_NAME}_layout PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_layout PUBLIC yaml-cpp)

add_library(${PROJECT_NAME}_vision_sensor src/vision_sensor_feed.cpp src/frame_roll_corrector.cpp)
target_include_directories(${PROJECT_NAME}_vision_sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_vision_sensor
  yaml-cpp Qt5::Core Qt5::Gui ${OpenCV_LIBS} PkgConfig::UDEV Threads::Threads)
target_include_directories(${PROJECT_NAME}_vision_sensor PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_vision_sensor rclcpp sensor_msgs std_msgs)

add_executable(allegro_hand_v6_dashboard_node
  src/dashboard_main.cpp
  src/dashboard_window.cpp
  src/sensor_panel.cpp)
target_include_directories(allegro_hand_v6_dashboard_node PRIVATE include)
target_link_libraries(allegro_hand_v6_dashboard_node
  ${PROJECT_NAME}_layout
  ${PROJECT_NAME}_vision_sensor
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets)
ament_target_dependencies(allegro_hand_v6_dashboard_node
  ament_index_cpp
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs)

install(TARGETS
  ${PROJECT_NAME}_layout
  ${PROJECT_NAME}_vision_sensor
  allegro_hand_v6_dashboard_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY assets config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES LICENSE THIRD_PARTY_NOTICES.md README.md README_KR.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_roll_corrector test/test_frame_roll_corrector.cpp)
  if(TARGET test_frame_roll_corrector)
    target_include_directories(test_frame_roll_corrector PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(test_frame_roll_corrector ${PROJECT_NAME}_vision_sensor ${OpenCV_LIBS})
  endif()
  ament_add_gtest(test_sensor_layout
    test/test_sensor_layout.cpp
    src/sensor_panel.cpp)
  if(TARGET test_sensor_layout)
    target_include_directories(test_sensor_layout PRIVATE include)
    target_link_libraries(test_sensor_layout
      ${PROJECT_NAME}_layout
      ${PROJECT_NAME}_vision_sensor
      Qt5::Core
      Qt5::Gui
      Qt5::Widgets)
    target_compile_definitions(test_sensor_layout PRIVATE
      TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/tactile_layout.yaml"
      LEFT_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/tactile_layout_left.yaml")
    ament_target_dependencies(test_sensor_layout rclcpp sensor_msgs std_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_layout)
ament_package()
