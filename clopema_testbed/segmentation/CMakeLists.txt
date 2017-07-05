cmake_minimum_required(VERSION 2.8.3)
project(segmentation)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    cmake_modules
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS
    serialization
    iostreams
)

catkin_package(
    INCLUDE_DIRS 
        include
    LIBRARIES
        libSegmentation
    CATKIN_DEPENDS
        cmake_modules
        roscpp
    DEPENDS
        OpenCV
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(libSegmentation
    src/SegmentationGMMGrabCut.cpp
    src/Cluster.cpp
)
target_link_libraries(libSegmentation
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
)

set(CMAKE_BUILD_TYPE Release)
