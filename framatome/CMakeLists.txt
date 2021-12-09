cmake_minimum_required(VERSION 2.8.3)
project(framatome)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  dynamic_reconfigure
)

catkin_python_setup()


catkin_package()
