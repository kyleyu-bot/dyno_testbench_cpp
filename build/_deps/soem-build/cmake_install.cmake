# Install script for directory: /home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src

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
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-build/libsoem.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/soem" TYPE FILE FILES
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/soem/ethercat.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/soem/ethercatbase.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/soem/ethercatcoe.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/soem/ethercatconfig.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/soem/ethercatconfiglist.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/soem/ethercatdc.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/soem/ethercateoe.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/soem/ethercatfoe.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/soem/ethercatmain.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/soem/ethercatprint.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/soem/ethercatsoe.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/soem/ethercattype.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/osal/linux/osal_defs.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/osal/osal.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/oshw/linux/nicdrv.h"
    "/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-src/oshw/linux/oshw.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-build/test/linux/slaveinfo/cmake_install.cmake")
  include("/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-build/test/linux/eepromtool/cmake_install.cmake")
  include("/home/dyno-nuc2/gitRepos/dyno_testbench_cpp/build/_deps/soem-build/test/linux/simple_test/cmake_install.cmake")

endif()

