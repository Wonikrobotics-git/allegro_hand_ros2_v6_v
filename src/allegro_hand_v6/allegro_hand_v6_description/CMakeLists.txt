cmake_minimum_required(VERSION 3.5.0)

project(allegro_hand_v6_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  meshes
  urdf
  config
  rviz
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

# USD assets are generated locally and are not included in the repository.
if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/usd")
  install(DIRECTORY usd DESTINATION share/${PROJECT_NAME}/)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
