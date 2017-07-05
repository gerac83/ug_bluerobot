cmake_minimum_required(VERSION 2.8.3)
project(clopema_robot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED clopema_moveit actionlib actionlib_msgs message_generation)

add_service_files(
  DIRECTORY srv
  FILES
    GraspAndFold.srv
    TimeParametrization.srv
    SetSpeed.srv
)

catkin_python_setup()
generate_messages(DEPENDENCIES clopema_moveit)

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})
catkin_package(
    LIBRARIES ${PROJECT_NAME} ${KDL_LIBRARY}
    INCLUDE_DIRS include
    CATKIN_DEPENDS clopema_moveit actionlib actionlib_msgs message_runtime
)

include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src_cpp/clopema_robot/robot_commander.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(move_home src_cpp/clopema_robot/move_home.cpp)
target_link_libraries(move_home ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(move_home ${catkin_EXPORTED_TARGETS})

add_executable(grippers_demo src_cpp/clopema_robot/grippers_demo.cpp)
target_link_libraries(grippers_demo ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(grippers_demo ${catkin_EXPORTED_TARGETS})

add_executable(grasp_and_fold src_cpp/clopema_robot/grasp_and_fold.cpp)
target_link_libraries(grasp_and_fold ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(grasp_and_fold ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(grasping_services src_cpp/clopema_robot/grasping_services.cpp)
target_link_libraries(grasping_services ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(time_param_srv
    src_cpp/clopema_robot/time_param_srv.cpp
)
target_link_libraries(time_param_srv
    ${catkin_LIBRARIES}
)
add_dependencies(time_param_srv
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencpp
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_subdirectory(tests)
    add_rostest(tests/test_limits.test)
    add_rostest(tests/test_async_execute.test)
endif()
