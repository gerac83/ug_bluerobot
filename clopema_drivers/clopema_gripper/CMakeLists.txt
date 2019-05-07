cmake_minimum_required(VERSION 2.8.3)
project(clopema_gripper)

# Options
option(BUILD_GRIPPER_SERVER "Build the gripper server." OFF)

set(ROS_BUILD_STATIC_LIBS true)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp actionlib actionlib_msgs control_msgs sensor_msgs message_runtime)
find_package(Boost REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(view/tools view/aux/Ngin view/aux/Ngin3d view/aux/shFont view/aux/shImage)

add_service_files(DIRECTORY srv)
add_message_files(DIRECTORY msg)

generate_messages(DEPENDENCIES std_msgs control_msgs sensor_msgs actionlib_msgs)
catkin_package(CATKIN_DEPENDS std_msgs roscpp actionlib actionlib_msgs control_msgs sensor_msgs message_runtime)

set(VIEW_PATH "${CMAKE_CURRENT_SOURCE_DIR}/view")

# Externally built libraries (for visualizer)
#add_custom_target(tools_lib COMMAND make WORKING_DIRECTORY "${VIEW_PATH}/tools")
#add_custom_target(aux_lib COMMAND make WORKING_DIRECTORY "${VIEW_PATH}/aux")

# Server-side executables
## URT output
if (BUILD_GRIPPER_SERVER)
    execute_process(COMMAND urt-config --no-new-line --user-cflags OUTPUT_VARIABLE URT_CFLAGS)
    execute_process(COMMAND urt-config --no-new-line --user-ldflags-shared OUTPUT_VARIABLE URT_LDFLAGS)

    set(SERVER_CFLAGS "${URT_CFLAGS} -Wall -O0 -g -Wno-unused-result")
    set(SERVER_LDFLAGS "${URT_LDFLAGS}")

    add_executable(clopema_gripper_driver src/driver.c)
    add_library(clopema_gripper_skin src/skin.c src/skin_iterators.cpp)
    add_library(clopema_gripper_motor src/motor.cpp)
    add_library(clopema_gripper_micro src/micro.cpp)
#    add_executable(clopema_gripper_view src/view.cpp src/shconio.c)

    add_executable(clopema_gripper_rosskin_server src/skin_ros_server.cpp)
    add_executable(clopema_gripper_motor_server src/motor_ros_server.cpp)
    add_executable(clopema_gripper_micro_server src/micro_ros_server.cpp)

    set_target_properties(clopema_gripper_driver PROPERTIES COMPILE_FLAGS "${SERVER_CFLAGS}")
    target_link_libraries(clopema_gripper_driver "-lpcan ${SERVER_LDFLAGS}")
    set_target_properties(clopema_gripper_skin PROPERTIES COMPILE_FLAGS "${SERVER_CFLAGS}")
    target_link_libraries(clopema_gripper_skin "${SERVER_LDFLAGS}")
    set_target_properties(clopema_gripper_motor PROPERTIES COMPILE_FLAGS "${SERVER_CFLAGS}")
    target_link_libraries(clopema_gripper_motor "${SERVER_LDFLAGS}")
    set_target_properties(clopema_gripper_micro PROPERTIES COMPILE_FLAGS "${SERVER_CFLAGS}")
    target_link_libraries(clopema_gripper_micro "${SERVER_LDFLAGS}")
#    set_target_properties(clopema_gripper_view PROPERTIES COMPILE_FLAGS
#        "${SERVER_CFLAGS} -I${VIEW_PATH}/tools -I${VIEW_PATH}/aux/shImage -I${VIEW_PATH}/aux/shFont -I${VIEW_PATH}/aux/Ngin -I${VIEW_PATH}/aux/Ngin3d")
#    target_link_libraries(clopema_gripper_view clopema_gripper_skin clopema_gripper_motor "-lviewtools -lNgin3d -lNgin -lSDLmain -lSDL -lGL -lGLU -lshFont -lshImage ${SERVER_LDFLAGS}")
#    add_dependencies(clopema_gripper_view tools_lib aux_lib)
    set_target_properties(clopema_gripper_rosskin_server PROPERTIES COMPILE_FLAGS "${SERVER_CFLAGS}")
    target_link_libraries(clopema_gripper_rosskin_server clopema_gripper_skin "${SERVER_LDFLAGS}" ${catkin_LIBRARIES})
    set_target_properties(clopema_gripper_motor_server PROPERTIES COMPILE_FLAGS "${SERVER_CFLAGS}")
    target_link_libraries(clopema_gripper_motor_server clopema_gripper_motor "${SERVER_LDFLAGS}" ${catkin_LIBRARIES})
    set_target_properties(clopema_gripper_micro_server PROPERTIES COMPILE_FLAGS "${SERVER_CFLAGS}")
    target_link_libraries(clopema_gripper_micro_server clopema_gripper_micro "${SERVER_LDFLAGS}" ${catkin_LIBRARIES})

    add_dependencies(clopema_gripper_rosskin_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
    add_dependencies(clopema_gripper_motor_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
    add_dependencies(clopema_gripper_micro_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif (BUILD_GRIPPER_SERVER)

# Client-side executables
add_library(clopema_gripper_skin_ds src/skin.c)
add_library(clopema_gripper_rosskin_client src/skin_ros_client.cpp)
#add_executable(clopema_gripper_ros_view src/view.cpp src/shconio.c)
#add_dependencies(clopema_gripper_ros_view tools_lib aux_lib clopema_gripper_skin_ds clopema_gripper_rosskin_client)

set(CLIENT_CFLAGS "-Wall -O0 -g -Wno-unused-result -DSKIN_DS_ONLY")

set_target_properties(clopema_gripper_skin_ds PROPERTIES COMPILE_FLAGS "${CLIENT_CFLAGS}")
set_target_properties(clopema_gripper_rosskin_client PROPERTIES COMPILE_FLAGS "${CLIENT_CFLAGS}")
target_link_libraries(clopema_gripper_rosskin_client clopema_gripper_skin_ds ${catkin_LIBRARIES})
#set_target_properties(clopema_gripper_ros_view PROPERTIES COMPILE_FLAGS
#	"${CLIENT_CFLAGS} -DTHROUGH_ROS -I${VIEW_PATH}/tools -I${VIEW_PATH}/aux/shImage -I${VIEW_PATH}/aux/shFont -I${VIEW_PATH}/aux/Ngin -I${VIEW_PATH}/aux/Ngin3d")
#target_link_libraries(clopema_gripper_ros_view clopema_gripper_skin_ds clopema_gripper_rosskin_client "-lviewtools -lNgin3d -lNgin -lrt -lSDLmain -lSDL -lGL -lGLU -lshFont -lshImage" ${catkin_LIBRARIES})

add_dependencies(clopema_gripper_skin_ds ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(clopema_gripper_rosskin_client ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(clopema_gripper_ros_view ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Visualizer data installation
file(GLOB data_files "data/*")
install(FILES ${data_files} DESTINATION ~/.clopema_view/data/)
file(GLOB setting_files "view/settings/*")
install(FILES ${setting_files} DESTINATION ~/.clopema_view/settings/)
file(GLOB gui_files "view/gui/*")
install(FILES ${gui_files} DESTINATION ~/.clopema_view/gui/)
