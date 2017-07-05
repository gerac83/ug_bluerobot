cmake_minimum_required(VERSION 2.8.3)
project(clopema_force)
find_package(catkin REQUIRED COMPONENTS tf roscpp tf_conversions urdf)

catkin_package(
    INCLUDE_DIRS include 
    CATKIN_DEPENDS
)
    
include_directories(include 
    ${catkin_INCLUDE_DIRS}
)

add_executable(filter_force 
    src/filter_force.cpp
)
target_link_libraries(filter_force 
    ${catkin_LIBRARIES}
)

add_executable(filter_gripper 
    src/filter_gripper.cpp
)
target_link_libraries(filter_gripper 
    ${catkin_LIBRARIES}
)

