cmake_minimum_required(VERSION 2.8.3)
project(clopema_drivers_msgs)

find_package(catkin REQUIRED COMPONENTS 
    sensor_msgs
    actionlib_msgs
    message_generation 
)

add_service_files(
    DIRECTORY
        srv
    FILES 
        SetSynchronization.srv
)

add_action_files(
    DIRECTORY 
        action
    FILES 
        GetCameraImage.action
        GetStereoPair.action 
)

generate_messages(
    DEPENDENCIES 
        actionlib_msgs 
        sensor_msgs 
)

catkin_package(
    CATKIN_DEPENDS    
        actionlib_msgs 
        message_runtime
        sensor_msgs 
    INCLUDE_DIRS 
        include    #Forwarding include files from this package to INCLUDE_DIRS
)

