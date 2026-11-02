# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target escooter_msgs::escooter_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${escooter_msgs_TARGETS}.
if(escooter_msgs_TARGETS AND NOT TARGET escooter_msgs::escooter_msgs)
  add_library(escooter_msgs::escooter_msgs INTERFACE IMPORTED)
  set_target_properties(escooter_msgs::escooter_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${escooter_msgs_TARGETS}")
endif()
