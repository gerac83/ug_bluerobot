cmake_minimum_required(VERSION 2.8.3)
project(clopema_description)

find_package(catkin REQUIRED COMPONENTS urdf roscpp roslib)
find_package(orocos_kdl REQUIRED) 
find_package(Boost REQUIRED COMPONENTS filesystem)
catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

#add_executable(mechanical_calibration src/mechanical_calibration.cpp)
#target_link_libraries(mechanical_calibration ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
#set_target_properties(mechanical_calibration PROPERTIES LINK_FLAGS -L${orocos_kdl_LIBRARY_DIRS})

##########################
## Generate description ##
##########################
set(clopema_partner $ENV{CLOPEMA_PARTNER})
if("${clopema_partner}" STREQUAL "")
	message(FATAL_ERROR "Environment variable CLOPEMA_PARTNER must be set.")
endif()

message(STATUS "Compiling model (robot_description) for ${clopema_partner}")

set(calib_arg "${PROJECT_SOURCE_DIR}/scripts/calibrate.sh")
set(calib_cmd "bash")
execute_process(COMMAND ${calib_cmd} ${calib_arg} ${CATKIN_DEVEL_PREFIX}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  RESULT_VARIABLE calib_result
  OUTPUT_VARIABLE calib_ver)
