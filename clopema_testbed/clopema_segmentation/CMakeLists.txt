cmake_minimum_required(VERSION 2.8.3)
project(clopema_segmentation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    tf_conversions
    cv_bridge
    image_geometry
    message_generation
    std_msgs
    sensor_msgs
    segmentation
    clopema_libs
)

add_service_files(FILES
    Segmentation.srv
    SegmentationGMM.srv
)

generate_messages(DEPENDENCIES 
    std_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(segmentation_gc_gmm_learn
    src/segmentation_learn.cpp
)
add_dependencies(segmentation_gc_gmm_learn
    ${PROJECT_NAME}_gencpp
)
target_link_libraries(segmentation_gc_gmm_learn
    ${catkin_LIBRARIES}
)

add_executable(segmentation_gc_gmm_service
    src/seg_service.cpp
)
add_dependencies(segmentation_gc_gmm_service 
    ${PROJECT_NAME}_gencpp
)
target_link_libraries(segmentation_gc_gmm_service
    ${catkin_LIBRARIES}
)

add_executable(segmentation_gc_gmm_example
    src/seg_example.cpp
)
add_dependencies(segmentation_gc_gmm_example 
    ${PROJECT_NAME}_gencpp
)
target_link_libraries(segmentation_gc_gmm_example
    ${catkin_LIBRARIES}
)
