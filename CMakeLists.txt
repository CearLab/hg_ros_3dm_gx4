cmake_minimum_required(VERSION 2.8.3)

project(hg_ros_3dm_gx4)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs)
find_package(Boost REQUIRED COMPONENTS system chrono)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS Boost
  LIBRARIES hg_ros_serial
)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(hg_ros_serial src/serial.cpp)

add_executable(3dm_gx4_test src/3dm_gx4_test.cpp src/3dm_gx4.cpp)
target_link_libraries(3dm_gx4_test hg_ros_serial
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(3dm_gx4_node src/3dm_gx4_node.cpp src/3dm_gx4.cpp)
target_link_libraries(3dm_gx4_node hg_ros_serial
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
