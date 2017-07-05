cmake_minimum_required(VERSION 2.8.3)
project(clopema_libs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
    roscpp sensor_msgs 
    cv_bridge 
    image_geometry 
    clopema_robot
    cmake_modules
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

catkin_python_setup()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
        clopema_libs 
        clopema_segmentation_libs 
        clopema_force_libs
    CATKIN_DEPENDS 
        roscpp 
        sensor_msgs 
        image_geometry 
        clopema_robot
    DEPENDS 
        Boost 
        OpenCV
)

add_executable(topic_latch_test 
    src_cpp/clopema_libs/topic_latch_test.cpp
)
target_link_libraries(topic_latch_test 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${OpenCV_LIBRARIES}
)

add_library(clopema_libs 
    src_cpp/clopema_libs/tf_utils.cpp 
    src_cpp/clopema_libs/ui_utils.cpp 
    src_cpp/clopema_libs/RobotMovingUtils.cpp
    src_cpp/clopema_libs/BasicClasificators.cpp
    src_cpp/clopema_libs/clopema_pose_generator.cpp
)
target_link_libraries(clopema_libs 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${Eigen_LIBRARIES}
)

add_library(clopema_segmentation_libs 
    src_cpp/clopema_libs/Segmentation.cpp
)
target_link_libraries(clopema_segmentation_libs 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${Eigen_LIBRARIES}
)

add_library(clopema_force_libs 
    src_cpp/clopema_libs/ForceGrabber.cpp
)
target_link_libraries(clopema_force_libs 
    ${catkin_LIBRARIES} 
    ${Eigen_LIBRARIES}
)

