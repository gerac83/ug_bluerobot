cmake_minimum_required(VERSION 2.8.3)
project(clopema_controller)

add_definitions(-DROS=1)           #build using ROS libraries
add_definitions(-DLINUXSOCKETS=1)  #build using LINUX SOCKETS libraries

find_package(catkin REQUIRED COMPONENTS
    industrial_msgs
    industrial_robot_client
    industrial_utils
    motoman_driver
    simple_message
    clopema_drivers_msgs
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_executable(clopema_robot_state 
    src/clopema_robot_state.cpp
    src/ClopemaRobotStateInterface.cpp
    src/ClopemaJointRelayHandler.cpp
)
target_link_libraries(clopema_robot_state
    industrial_robot_client
    simple_message
    ${catkin_LIBRARIES}
)
add_dependencies(clopema_robot_state
    ${catkin_EXPORTED_TARGETS}
)

add_executable(clopema_joint_streamer
    src/clopema_joint_streamer.cpp
    src/ClopemaJointTrajectoryStreamer.cpp
    src/MotomanMotionCtrl.cpp
)
target_link_libraries(clopema_joint_streamer
    motoman_simple_message
    motoman_industrial_robot_client
    industrial_robot_client
    simple_message
    ${catkin_LIBRARIES}
)
add_dependencies(clopema_joint_streamer
    ${catkin_EXPORTED_TARGETS}
)

# if(CATKIN_ENABLE_TESTING)
#     find_package(rostest REQUIRED)
#     add_subdirectory(tests)
# endif()
