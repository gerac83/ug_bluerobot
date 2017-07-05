cmake_minimum_required(VERSION 2.8.3)
project(clopema_stereo)

set(CMAKE_MODULE_PATH 
    ${CMAKE_MODULE_PATH} 
    "${PROJECT_SOURCE_DIR}/cmake_modules/"
)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    clopema_drivers_msgs
)

find_package(OpenCV REQUIRED)
find_package(GPHOTO2 REQUIRED)

catkin_package()

include_directories(
    include 
    ${catkin_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS} 
    ${GPHOTO2_INCLUDE_DIRS}
)

add_executable(get_camera_image_server 
    src/get_camera_image_server.cpp 
    src/GPhotoRos.cpp
)
target_link_libraries(get_camera_image_server 
    ${catkin_LIBRARIES} 
    ${GPHOTO2_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    jpeg
)
add_dependencies(get_camera_image_server 
    ${catkin_EXPORTED_TARGETS}
)
message(WARNING "Replace jpeg dependency with find_pkg")

add_executable(get_stereo_pair_server 
    src/get_stereo_pair_server.cpp
)
target_link_libraries(get_stereo_pair_server 
    ${catkin_LIBRARIES}
)
add_dependencies(get_stereo_pair_server 
    ${catkin_EXPORTED_TARGETS}
)

add_executable(get_stereo_pair_client 
    src/get_stereo_pair_client.cpp
)
target_link_libraries(get_stereo_pair_client 
    ${catkin_LIBRARIES}
)
add_dependencies(get_stereo_pair_client 
    ${catkin_EXPORTED_TARGETS}
)

