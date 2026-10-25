cmake_minimum_required(VERSION 3.8)
project(allegro_hand_v6_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(allegro_hand_v6_gui_node
  src/main.cpp
  src/main_window.cpp
  include/allegro_hand_v6_gui/main_window.hpp
)

target_include_directories(allegro_hand_v6_gui_node PRIVATE include)
target_link_libraries(allegro_hand_v6_gui_node Qt5::Widgets yaml-cpp)
ament_target_dependencies(allegro_hand_v6_gui_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS allegro_hand_v6_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY poses/
  DESTINATION share/${PROJECT_NAME}/poses
)

ament_package()
