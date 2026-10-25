cmake_minimum_required(VERSION 3.16)
set(CMAKE_WARN_DEPRECATED OFF)
project(allegro_hand_position_effort_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wno-cpp)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  spdlog
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# generate_parameter_library(
#   forward_command_controller_parameters
#   src/forward_command_controller_parameters.yaml
# )

# generate_parameter_library(
#   multi_interface_forward_command_controller_parameters
#   src/multi_interface_forward_command_controller_parameters.yaml
# )

generate_parameter_library(
  position_effort_controller_parameters
  src/position_effort_controller_parameters.yaml
)

add_library(allegro_hand_position_effort_controller SHARED 
  src/position_effort_controller.cpp 
)
target_compile_features(allegro_hand_position_effort_controller PUBLIC cxx_std_17)
target_include_directories(allegro_hand_position_effort_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/allegro_hand_position_effort_controller>
)
target_link_libraries(allegro_hand_position_effort_controller PUBLIC
  position_effort_controller_parameters
  spdlog::spdlog
)
ament_target_dependencies(allegro_hand_position_effort_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(allegro_hand_position_effort_controller PRIVATE "AH_POSITION_EFFORT_CONTROLLER_BUILDING_DLL")

target_compile_definitions(allegro_hand_position_effort_controller PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

pluginlib_export_plugin_description_file(controller_interface allegro_hand_position_effort_controller_plugin.xml)


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # ament_add_gmock(test_load_forward_command_controller
  #   test/test_load_forward_command_controller.cpp
  # )
  # target_link_libraries(test_load_forward_command_controller
  #   allegro_hand_position_effort_controller
  # )
  # ament_target_dependencies(test_load_forward_command_controller
  #   controller_manager
  #   hardware_interface
  #   ros2_control_test_assets
  # )

  # ament_add_gmock(test_forward_command_controller
  #   test/test_forward_command_controller.cpp
  # )
  # target_link_libraries(test_forward_command_controller
  #   allegro_hand_position_effort_controller
  # )

  # ament_add_gmock(test_load_multi_interface_forward_command_controller
  #   test/test_load_multi_interface_forward_command_controller.cpp
  # )
  # target_link_libraries(test_load_multi_interface_forward_command_controller
  #   allegro_hand_position_effort_controller
  # )
  # ament_target_dependencies(test_load_multi_interface_forward_command_controller
  #   controller_manager
  #   hardware_interface
  #   ros2_control_test_assets
  # )

  
  # ament_add_gmock(test_multi_interface_forward_command_controller
  #   test/test_multi_interface_forward_command_controller.cpp
  # )
  # target_link_libraries(test_multi_interface_forward_command_controller
  #   allegro_hand_position_effort_controller
  # )
endif()

install(
  FILES allegro_hand_position_effort_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/allegro_hand_position_effort_controller
)

install(
  TARGETS
    allegro_hand_position_effort_controller
    # forward_command_controller_parameters
    # multi_interface_forward_command_controller_parameters
    position_effort_controller_parameters
  EXPORT export_aph_custom_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_aph_custom_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
