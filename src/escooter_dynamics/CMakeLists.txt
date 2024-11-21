cmake_minimum_required(VERSION 3.0.2)
project(escooter_dynamics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  geometry_msgs
  escooter_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES escooter_dynamics
  CATKIN_DEPENDS roscpp nodelet geometry_msgs escooter_msgs std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library( control src/control.cpp )
add_dependencies( control escooter_msgs_generate_messages_cpp )

add_library( dynamics src/dynamics.cpp )
add_dependencies( dynamics escooter_msgs_generate_messages_cpp )

add_executable( escooter_dynamics src/escooter_dynamics.cpp )
target_link_libraries( escooter_dynamics ${catkin_LIBRARIES} control dynamics )
add_dependencies( escooter_dynamics escooter_msgs_generate_messages_cpp )

install(TARGETS escooter_dynamics
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES escooter_dynamics.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})