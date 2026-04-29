cmake_minimum_required(VERSION 3.0.2)
project(slamware_loop_slam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/scan_icp_node.py
  scripts/pose_graph_slam_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
