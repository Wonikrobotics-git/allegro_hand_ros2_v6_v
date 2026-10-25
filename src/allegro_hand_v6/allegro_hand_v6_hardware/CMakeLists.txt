cmake_minimum_required(VERSION 3.8)
project(allegro_hand_v6_hardware)

# Set the default build type to Release if it's not specified.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "Defaulting build type to 'Release'")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(libmodbus REQUIRED libmodbus)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(spdlog REQUIRED) 
find_package(diagnostic_updater REQUIRED)
find_package(allegro_hand_utility REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  hardware_interface 
  pluginlib
  rclcpp_lifecycle
  realtime_tools
  spdlog 
  diagnostic_updater
  allegro_hand_utility
)

add_library(
  allegro_hand_v6_hardware 
  SHARED  
  ./src/v6_sdk.cpp 
  ./src/v6_spec.cpp 
  ./src/v6_hardware_interface.cpp
  ./src/csv_logger.cpp
)

target_compile_definitions(allegro_hand_v6_hardware PUBLIC SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

target_include_directories(allegro_hand_v6_hardware PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/allegro_hand_v6_hardware>
  ${libmodbus_INCLUDE_DIRS}
)

target_link_libraries(allegro_hand_v6_hardware ${libmodbus_LIBRARIES} Threads::Threads)

ament_target_dependencies(allegro_hand_v6_hardware ${dependencies})

# =========================================================================
# Python Bindings (pybind11)
# =========================================================================
pybind11_add_module(allegro_hand_v6_sdk
  src/pybind_v6_sdk.cpp
)
target_include_directories(allegro_hand_v6_sdk PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  ${libmodbus_INCLUDE_DIRS}
)
target_link_libraries(allegro_hand_v6_sdk PRIVATE
  allegro_hand_v6_hardware
  ${libmodbus_LIBRARIES}
  spdlog::spdlog
)

install(TARGETS allegro_hand_v6_sdk
  DESTINATION "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
)

# export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface allegro_hand_v6_hardware.xml)

# ros2_control hardware
install(TARGETS allegro_hand_v6_hardware
  EXPORT allegro_hand_v6_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(hardware_interface REQUIRED)

  ament_add_gtest(test_csv_logger test/test_csv_logger.cpp src/csv_logger.cpp)
  target_include_directories(test_csv_logger PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_csv_logger Threads::Threads)

  ament_add_gtest(test_sdk_read test/test_sdk_read.cpp src/v6_sdk.cpp src/v6_spec.cpp)
  target_include_directories(test_sdk_read PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${libmodbus_INCLUDE_DIRS})
  target_link_libraries(test_sdk_read ${libmodbus_LIBRARIES} Threads::Threads spdlog::spdlog)
  ament_target_dependencies(test_sdk_read allegro_hand_utility spdlog)
  set_target_properties(test_sdk_read PROPERTIES LINK_FLAGS
    "-Wl,--wrap=modbus_connect,--wrap=modbus_close,--wrap=modbus_write_bit,--wrap=modbus_read_registers")

  ament_add_gtest(test_v6_sdk test/test_sdk.cpp)
  target_link_libraries(test_v6_sdk allegro_hand_v6_hardware ${libmodbus_LIBRARIES})
  target_include_directories(test_v6_sdk PRIVATE ${libmodbus_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
  ament_target_dependencies(test_v6_sdk ${dependencies})
  install(TARGETS test_v6_sdk DESTINATION lib/${PROJECT_NAME})

  ament_add_gtest(test_v6_hardware test/test_interface.cpp)
  target_link_libraries(test_v6_hardware allegro_hand_v6_hardware ${libmodbus_LIBRARIES})
  target_include_directories(test_v6_hardware PRIVATE ${libmodbus_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
  ament_target_dependencies(test_v6_hardware ${dependencies})
  install(TARGETS test_v6_hardware DESTINATION lib/${PROJECT_NAME})

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# standalone motion test executable
add_executable(test_sdk_motion test/test_sdk_motion.cpp)
target_link_libraries(test_sdk_motion allegro_hand_v6_hardware ${libmodbus_LIBRARIES})
target_include_directories(test_sdk_motion PRIVATE ${libmodbus_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
ament_target_dependencies(test_sdk_motion ${dependencies})
install(TARGETS test_sdk_motion DESTINATION lib/${PROJECT_NAME})

# export
ament_export_targets(allegro_hand_v6_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp hardware_interface pluginlib rclcpp_lifecycle Threads)

ament_package()
