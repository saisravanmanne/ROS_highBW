cmake_minimum_required(VERSION 2.8.3)
project(highBW)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs sensor_msgs tf)

## Declare ROS messages and services
##add_message_files(FILES Num.msg)
##add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs tf)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ground_station_data_receive_Vive2 src/ground_station_data_receive_Vive.cpp)
target_link_libraries(ground_station_data_receive_Vive2 ${catkin_LIBRARIES})
add_dependencies(ground_station_data_receive_Vive2 highBW_generate_messages_cpp)

add_executable(ground_station_innerLoop2 src/ground_station_innerLoop.cpp)
target_link_libraries(ground_station_innerLoop2 ${catkin_LIBRARIES})
add_dependencies(ground_station_innerLoop2 highBW_generate_messages_cpp)

add_executable(data_read2 src/robot_3_data_send.cpp)
target_link_libraries(data_read2 ${catkin_LIBRARIES})
add_dependencies(data_read2 highBW_generate_messages_cpp)

add_executable(tesla1 src/tesla1.cpp)
target_link_libraries(tesla1 ${catkin_LIBRARIES})
add_dependencies(tesla1 highBW_generate_messages_cpp)

# %EndTag(FULLTEXT)%
