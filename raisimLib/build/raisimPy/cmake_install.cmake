# Install script for directory: /home/dohyeon/raisim_ws/raisimLib/raisimPy

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/dohyeon/raisim_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/dohyeon/raisim_ws/install/lib/python3/dist-packages/raisimpy.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/dohyeon/raisim_ws/install/lib/python3/dist-packages/raisimpy.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/dohyeon/raisim_ws/install/lib/python3/dist-packages/raisimpy.cpython-38-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/dohyeon/raisim_ws/install/lib/python3/dist-packages/raisimpy.cpython-38-x86_64-linux-gnu.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/dohyeon/raisim_ws/install/lib/python3/dist-packages" TYPE MODULE FILES "/home/dohyeon/raisim_ws/raisimLib/build/raisimPy/raisimpy.cpython-38-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}/home/dohyeon/raisim_ws/install/lib/python3/dist-packages/raisimpy.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/dohyeon/raisim_ws/install/lib/python3/dist-packages/raisimpy.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/dohyeon/raisim_ws/install/lib/python3/dist-packages/raisimpy.cpython-38-x86_64-linux-gnu.so"
         OLD_RPATH "/home/dohyeon/raisim_ws/raisimLib/raisim/linux/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/dohyeon/raisim_ws/install/lib/python3/dist-packages/raisimpy.cpython-38-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/dohyeon/raisim_ws/raisimLib/raisimPy/../raisim/linux/lib/raisimpy.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/dohyeon/raisim_ws/raisimLib/raisimPy/../raisim/linux/lib/raisimpy.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/dohyeon/raisim_ws/raisimLib/raisimPy/../raisim/linux/lib/raisimpy.cpython-38-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/dohyeon/raisim_ws/raisimLib/raisimPy/../raisim/linux/lib/raisimpy.cpython-38-x86_64-linux-gnu.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/dohyeon/raisim_ws/raisimLib/raisimPy/../raisim/linux/lib" TYPE MODULE FILES "/home/dohyeon/raisim_ws/raisimLib/build/raisimPy/raisimpy.cpython-38-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}/home/dohyeon/raisim_ws/raisimLib/raisimPy/../raisim/linux/lib/raisimpy.cpython-38-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/dohyeon/raisim_ws/raisimLib/raisimPy/../raisim/linux/lib/raisimpy.cpython-38-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/dohyeon/raisim_ws/raisimLib/raisimPy/../raisim/linux/lib/raisimpy.cpython-38-x86_64-linux-gnu.so"
         OLD_RPATH "/home/dohyeon/raisim_ws/raisimLib/raisim/linux/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/dohyeon/raisim_ws/raisimLib/raisimPy/../raisim/linux/lib/raisimpy.cpython-38-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

