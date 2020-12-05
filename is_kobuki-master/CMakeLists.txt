cmake_minimum_required(VERSION 2.8.3)
project(is_kobuki)
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs std_msgs roscpp ecl_exceptions ecl_threads ecl_time kobuki_msgs nav_msgs rosbag roslib actionlib actionlib_msgs )

# Declare the message files to be built
#add_message_files(FILES
#)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include include/controller
  LIBRARIES kobuki
  CATKIN_DEPENDS message_runtime geometry_msgs std_srvs std_msgs roscpp ecl_exceptions ecl_threads ecl_time kobuki_msgs nav_msgs actionlib actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
