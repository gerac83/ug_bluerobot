# Install script for directory: /home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE RENAME "ompl.pc" FILES "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/CMakeModules/ompl.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl" TYPE FILE RENAME "ompl-config.cmake" FILES "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/doc/markdown/FindOMPL.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl" TYPE FILE FILES "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/ompl.conf")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/py-bindings/cmake_install.cmake")
  include("/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/src/cmake_install.cmake")
  include("/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/tests/cmake_install.cmake")
  include("/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/demos/cmake_install.cmake")
  include("/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/scripts/cmake_install.cmake")
  include("/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/doc/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/gerardo/Documents/blue_ws/src/ug_bluerobot/clopema_ompl/ompl/build/Release/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
