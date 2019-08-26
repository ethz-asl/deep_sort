project(deep_sort_ros)
cmake_minimum_required(VERSION 2.8.3)

# find_package(catkin_simple REQUIRED)
# catkin_simple(ALL_DEPS_REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
  netvlad_tf
)
catkin_python_setup()

add_service_files(
  FILES
  deep_sort_result.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  netvlad_tf
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


#############
## Install ##
#############

install(PROGRAMS
  scripts/deep_sort_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# cs_install()
# cs_export()
