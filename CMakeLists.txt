cmake_minimum_required(VERSION 2.8.3)
project(athena)

## Using std c++11
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros
  message_generation
  roslaunch
)

## Find OpenCV-3
##find_package(OpenCV 3 REQUIRED)

## Check all *.launch files in launch directory
roslaunch_add_file_check(launch)

## Declare ROS messages and services
add_message_files(FILES
                  Object.msg
                  Mission.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Add include directories to the build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
## ${OpenCV_INCLUDE_DIRS}
)

## Add class definition
set(CLASS_DEF
    src/copter.cpp)

## Add executable files
# ==========
# Main Nodes
# ==========
add_executable(fm_changer
    src/fm_changer.cpp)
add_dependencies(fm_changer
    athena_generate_messages_cpp)
target_link_libraries(fm_changer
    ${catkin_LIBRARIES})

add_executable(mission_control
    src/mission_control.cpp
    ${CLASS_DEF})
add_dependencies(mission_control
    athena_generate_messages_cpp)
target_link_libraries(mission_control
    ${catkin_LIBRARIES})

add_executable(main
    src/main.cpp)
add_dependencies(main
    athena_generate_messages_cpp)
target_link_libraries(main
    ${catkin_LIBRARIES})

add_executable(test_pub
    src/test_pub.cpp)
add_dependencies(test_pub
    athena_generate_messages_cpp)
target_link_libraries(test_pub
    ${catkin_LIBRARIES})

add_executable(test_servo
    src/test_servo.cpp
    ${CLASS_DEF})
add_dependencies(test_servo
    athena_generate_messages_cpp)
target_link_libraries(test_servo
    ${catkin_LIBRARIES})

install(DIRECTORY
launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
