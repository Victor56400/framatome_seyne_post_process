cmake_minimum_required(VERSION 2.8.3)
project(forssea_utilities)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        forssea_msgs
        dynamic_reconfigure
        )

find_package(Eigen3 REQUIRED)

find_package(yaml-cpp REQUIRED)

catkin_python_setup()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}_helpers ${PROJECT_NAME}_environment ${PROJECT_NAME}_rate ${PROJECT_NAME}_dynparam ${PROJECT_NAME}_grafcet
        CATKIN_DEPENDS roscpp forssea_msgs
        DEPENDS EIGEN3 YAML_CPP
)


###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_helpers src/helpers.cpp)
add_dependencies(${PROJECT_NAME}_helpers ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_helpers ${catkin_LIBRARIES})


add_library(${PROJECT_NAME}_dynparam src/DynamicParameters.cpp)
add_dependencies(${PROJECT_NAME}_dynparam ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_dynparam ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_library(${PROJECT_NAME}_environment src/Environment.cpp)
add_dependencies(${PROJECT_NAME}_environment ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_environment ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_rate src/Rate.cpp)
add_dependencies(${PROJECT_NAME}_rate ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_rate ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_grafcet src/grafcet.cpp)
add_dependencies(${PROJECT_NAME}_grafcet ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_grafcet ${catkin_LIBRARIES})

#############
## Testing ##
#############

catkin_add_gtest(${PROJECT_NAME}_test_helpers test/test_helpers.cpp)
target_link_libraries(${PROJECT_NAME}_test_helpers ${PROJECT_NAME}_helpers)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
