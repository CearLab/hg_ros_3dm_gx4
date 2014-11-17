cmake_minimum_required(VERSION 2.8.3)

project(hg_3dm_gx4)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Boost REQUIRED COMPONENTS thread system)
find_package(hg_ros_serial REQUIRED)

catkin_package()

include_directories(
    include    
    ${Boost_INCLUDE_DIRS}
    ${hg_ros_serial_INCLUDE_DIRS}
)

add_executable(3dm_test src/test.cpp src/3dm_gx4.cpp)
target_link_libraries(3dm_test  
  ${catkin_LIBRARIES}
)



