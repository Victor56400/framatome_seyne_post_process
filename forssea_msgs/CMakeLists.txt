cmake_minimum_required(VERSION 2.8.3)
project(forssea_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        std_msgs
        geometry_msgs
        message_generation
        roscpp
        dynamic_reconfigure
        actionlib_msgs
        geographic_msgs
        )

find_package(Eigen3 REQUIRED)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
        FILES
        LBL.msg
        USBL.msg
        SVMOutput.msg
        RobotState.msg
        PoseStampedArray.msg
        Waypoint.msg
        VectorField.msg
        Sonar.msg
)

add_service_files(
        FILES
        Waypoints.srv
        EngineOrder.srv
        SetInt.srv
        SetString.srv
        SetDACVoltage.srv
        SetPoseStamped.srv
        SetGeoPoseStamped.srv
        SetIntFloat.srv
        SetFloat.srv
)

add_action_files(
        FILES
        Path.action
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        actionlib_msgs
        geographic_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
generate_dynamic_reconfigure_options(
        cfg/DegreesOfFreedom.cfg # For thrust allocation
        cfg/Motor.cfg
        cfg/Environment.cfg
        cfg/HighGains.cfg
        cfg/Rate.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}_helpers
        DEPENDS EIGEN3
        CATKIN_DEPENDS std_msgs geometry_msgs roscpp dynamic_reconfigure actionlib_msgs geographic_msgs
)

###########
## Build ##
###########
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_helpers src/helpers.cpp)
add_dependencies(${PROJECT_NAME}_helpers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}_test_helpers test/test_helpers.cpp)
target_link_libraries(${PROJECT_NAME}_test_helpers ${PROJECT_NAME}_helpers)
