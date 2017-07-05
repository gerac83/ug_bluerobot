cmake_minimum_required(VERSION 2.8.3)
project(virtual_robot)

find_package(catkin REQUIRED
    industrial_msgs
    roscpp
    actionlib
    clopema_drivers_msgs
)

catkin_package(
    INCLUDE_DIRS 
        include
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_executable(virtual_robot
    src/virtual_robot.cpp
    src/VirtualRobot.cpp
)
target_link_libraries(virtual_robot
    ${catkin_LIBRARIES}
)
add_dependencies(virtual_robot
    ${catkin_EXPORTED_TARGETS}
)

add_executable(virtual_gripper
    src/virtual_gripper.cpp
    src/VirtualGripper.cpp
)
target_link_libraries(virtual_gripper
    ${catkin_LIBRARIES}
)
add_dependencies(virtual_gripper
    ${catkin_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(tests)
endif()

