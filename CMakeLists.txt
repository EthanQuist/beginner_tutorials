cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)

## Generate messages in the 'msg' folder
#add_message_files(FILES Num.msg)
#add_message_files(DIRECTORY msg FILES Num.msg)

## Generate services in the 'srv' folder
#add_service_files(FILES AddTwoInts.srv)
#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

generate_messages(DEPENDENCIES std_msgs)


catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
