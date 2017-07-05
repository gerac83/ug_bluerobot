cmake_minimum_required(VERSION 2.8.3)
project(clopema_kinematics)

add_compile_options(-std=c++11)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
set(MOVEIT_LIB_NAME clopema_kinematics_plugin)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    moveit_core
    pluginlib
    roscpp
    tf
    tf_conversions
    cmake_modules
)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(LAPACK REQUIRED)

add_service_files(
    FILES
        ClopemaGetAllIkt.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
    LIBRARIES 
        ${MOVEIT_LIB_NAME}
    CATKIN_DEPENDS 
        message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    include
)

add_library(${MOVEIT_LIB_NAME} 
    src/clopema_arm_kinematics_plugin.cpp
)
target_link_libraries(${MOVEIT_LIB_NAME} 
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
)
add_dependencies(${MOVEIT_LIB_NAME}
    ${PROJECT_NAME}_gencpp
)

install(
    TARGETS 
        ${MOVEIT_LIB_NAME}
    LIBRARY DESTINATION
        ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
    DIRECTORY
        include/
    DESTINATION
        include
)
install(
    FILES
        clopema_kinematics_plugin_description.xml
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

