cmake_minimum_required(VERSION 2.8.3)
project(clopema_calibration)

find_package(catkin REQUIRED COMPONENTS 
  image_transport 
  cv_bridge
  message_generation 
  sensor_msgs 
  dynamic_reconfigure  
  clopema_robot
  clopema_moveit
  image_geometry
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_python_setup()

add_message_files(FILES 
  Dotmatrix.msg 
  DotmatrixDot.msg
)
add_service_files(FILES 
  DetectDotmatrix.srv
)
generate_messages(DEPENDENCIES sensor_msgs)

generate_dynamic_reconfigure_options(cfg/Dotmatrix.cfg)
include_directories(
  include 
  dotmatrix 
  ${catkin_INCLUDE_DIRS}
)
link_directories(dotmatrix)

catkin_package(
  INCLUDE_DIRS 
    dotmatrix 
    include
  LIBRARIES
  CATKIN_DEPENDS 
    roscpp 
    image_transport 
    cv_bridge
    clopema_robot
    clopema_moveit
    image_geometry
    message_runtime
  DEPENDS 
    opencv 
    Eigen3
)  

add_executable(dotmatrix_node 
  src_cpp/dotmatrix_node.cpp
)
target_link_libraries(dotmatrix_node 
  dotmatrix 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(dotmatrix_srv 
  src_cpp/dotmatrix_srv.cpp
)
target_link_libraries(dotmatrix_srv 
  dotmatrix 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(dotmatrix_srv 
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)

