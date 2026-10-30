cmake_minimum_required(VERSION 3.8)
project(escooter_dynamics)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(escooter_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(ESCOOTER_DYNAMICS_DEPS
  escooter_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# --- libraries -------------------------------------------------------------
# Kept as two separate libraries, mirroring the ROS 1 `control` / `dynamics`
# split, but namespaced so they do not collide with other packages' targets.

add_library(control SHARED src/control.cpp)
target_include_directories(control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(control ${ESCOOTER_DYNAMICS_DEPS})

add_library(dynamics SHARED src/dynamics.cpp)
target_include_directories(dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dynamics ${ESCOOTER_DYNAMICS_DEPS})

# --- executable ------------------------------------------------------------

add_executable(escooter_dynamics_node src/escooter_dynamics_node.cpp)
target_link_libraries(escooter_dynamics_node control dynamics)
ament_target_dependencies(escooter_dynamics_node ${ESCOOTER_DYNAMICS_DEPS})

# --- install ---------------------------------------------------------------

install(
  TARGETS control dynamics
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS escooter_dynamics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/teleop_escooter.py
    scripts/keyboard_cmd_vel.py
    scripts/imu_to_euler.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(control dynamics)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${ESCOOTER_DYNAMICS_DEPS})

ament_package()
