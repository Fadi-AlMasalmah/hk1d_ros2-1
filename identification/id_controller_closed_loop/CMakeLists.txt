cmake_minimum_required(VERSION 3.5)
project(identification_controller_cl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(hk1d_interfaces REQUIRED)
find_package(stamped_array_msgs REQUIRED)
find_package(hk1d_identification_interfaces REQUIRED)

add_library(identification_controller_cl
  SHARED
  src/identification_controller_cl.cpp
)
target_include_directories(identification_controller_cl PRIVATE include)
ament_target_dependencies(identification_controller_cl
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  trajectory_msgs
  hk1d_interfaces
  stamped_array_msgs
  hk1d_identification_interfaces  
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(identification_controller_cl PRIVATE "IDENTIFICATION_CONTROLLER_CL_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(identification_controller_cl PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  identification_controller_cl
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  trajectory_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  identification_controller_cl
)
ament_package()
