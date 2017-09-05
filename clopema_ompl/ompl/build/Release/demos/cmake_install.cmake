# Install script for directory: /home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE FILE FILES
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/HypercubeBenchmark.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/RigidBodyPlanningWithIntegrationAndControls.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/CForestCircleGridBenchmark.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/OptimalPlanning.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/RigidBodyPlanningWithIK.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/LTLWithTriangulation.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/GeometricCarPlanning.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/KinematicChainBenchmark.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/OpenDERigidBodyPlanning.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/ThunderLightning.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/RigidBodyPlanningWithControls.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/PlannerData.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/RigidBodyPlanning.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/HybridSystemPlanning.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/TriangulationDemo.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/RigidBodyPlanningWithODESolverAndControls.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/StateSampling.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/Diagonal.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/Point2DPlanning.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/PlannerProgressProperties.cpp"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/RigidBodyPlanningWithODESolverAndControls.py"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/RandomWalkPlanner.py"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/PlannerData.py"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/OptimalPlanning.py"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/RigidBodyPlanningWithControls.py"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/RigidBodyPlanning.py"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/Point2DPlanning.py"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/KinematicChainPathPlot.py"
    "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/StateSampling.py"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/Koules")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/demos/VFRRT")
endif()

