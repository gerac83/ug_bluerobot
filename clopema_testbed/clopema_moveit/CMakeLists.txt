cmake_minimum_required(VERSION 2.8.3)
project(clopema_moveit)

add_definitions(-std=c++11)
add_compile_options(-std=c++11)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
find_package(catkin REQUIRED COMPONENTS moveit_msgs moveit_ros_move_group moveit_ros_planning moveit_ros_planning_interface moveit_core roscpp rospy tf tf_conversions message_generation eigen_conversions)
find_package(orocos_kdl REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES
   ClopemaJointInterpolation.srv
   ClopemaLinearInterpolation.srv
   ClopemaLinearInterpolationDual.srv
   ClopemaGraspIt.srv
   ClopemaGraspFromTable.srv
   ClopemaGraspFromTableDual.srv
   GetCartesianPathDual.srv
   GFold.srv
   TrajCollision.srv
)

generate_messages(DEPENDENCIES moveit_msgs)

catkin_package(
   INCLUDE_DIRS
   LIBRARIES clopema_moveit_capabilities
   CATKIN_DEPENDS
        moveit_msgs 
        moveit_ros_move_group 
        moveit_core 
        moveit_ros_planning 
        moveit_ros_planning_interface 
        roscpp rospy
        tf tf_conversions
        message_runtime
   DEPENDS
        orocos_kdl
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_executable(interpolation src/interpolation.cpp)
target_link_libraries(interpolation ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(interpolation ${PROJECT_NAME}_gencpp)

add_executable(grasping src/grasping.cpp)
target_link_libraries(grasping ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(grasping ${PROJECT_NAME}_gencpp)

add_library(clopema_moveit_capabilities
  src/cartesian_path_service_capability.cpp
  src/gfold_service_capability.cpp
  src/traj_collision_service_capability.cpp
)
target_link_libraries(clopema_moveit_capabilities ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(clopema_moveit_capabilities ${PROJECT_NAME}_gencpp)

install(TARGETS clopema_moveit_capabilities
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
install(FILES
  clopema_moveit_capabilities.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
