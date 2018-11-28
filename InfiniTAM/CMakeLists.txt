cmake_minimum_required(VERSION 2.8)
project(InfiniTAM)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

macro(CANONIFY_BOOL var)
  if (${var})
    set(${var} TRUE)
  else()
    set(${var} FALSE)
  endif()
endmacro()

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

find_package(CUDA QUIET)
CANONIFY_BOOL(CUDA_FOUND)
message(STATUS "CUDA found: " ${CUDA_FOUND})

find_package(OpenNI QUIET)
CANONIFY_BOOL(OPENNI_FOUND)
message(STATUS "OpenNI found: " ${OPENNI_FOUND})

find_package(libuvc QUIET)
CANONIFY_BOOL(libuvc_FOUND)
message(STATUS "libuvc found: ${libuvc_FOUND}")

find_package(PNG QUIET)
CANONIFY_BOOL(PNG_FOUND)
message(STATUS "libpng found: ${PNG_FOUND}")

find_package(OpenVR QUIET)
CANONIFY_BOOL(OpenVR_FOUND)
message(STATUS "OpenVR found: ${OPENVR_FOUND}")

# Eigen & PCL.
# NOTE: We also make sure to leave CMAKE_MODULE_PATH and CMAKE_PREFIX_PATH's
#       value unchanged (as cmake complains about CUDA otherwise).
set(CMAKE_MODULE_PATH_OLD ${CMAKE_MODULE_PATH})
set(CMAKE_PREFIX_PATH_OLD ${CMAKE_PREFIX_PATH})

set(CMAKE_MODULE_PATH ../../../Dependencies/Eigen/v_f3137d73/share/eigen3/cmake)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ../../../Dependencies/Eigen/v_f3137d73/)
find_package(Eigen3 REQUIRED)


set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ../../../Dependencies/pcl/v1.8.1)
find_package(PCL 1.7 REQUIRED COMPONENTS common io filters)

# The VTK library used by the PCL automatically add vtk* definitions
# that are not compatible with nvcc. This can be solved with the code below.
# Code is taken from https://github.com/PointCloudLibrary/pcl/issues/776
get_directory_property(dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
set(vtk_flags)
foreach(it ${dir_defs})
    if(it MATCHES "vtk*")
    list(APPEND vtk_flags ${it})
    endif()
endforeach()

foreach(d ${vtk_flags})
    remove_definitions(-D${d})
endforeach()


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH_OLD})
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH_OLD})
# End Eigen & PCL.


OPTION(WITH_CUDA "Build with CUDA support?" ${CUDA_FOUND})
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)

IF(MSVC_IDE)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-DUSING_CMAKE=1)
ELSE(MSVC_IDE)
  set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  set(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #set(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF(MSVC_IDE)

if(APPLE)
  set(CUDA_HOST_COMPILER /usr/bin/clang)
endif(APPLE)

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
ENDIF()

# Use C++11.
# NOTE: NVCC's option for C++11 needs to be specified separately (hence the 2nd
#       line below).
# NOTE: With the newer version of CMake used in Ubuntu 16.04, the CXX flags are
#       passed to NVCC. This causes the compilation to faile because the c++
#       flag is recorded twice. So the 2nd line isn't necessary anymore,
#       it even causes the compilation to fail.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11" )

################################################################################
# Dependencies.
# NOTE: this compiles Core and System specifically for the ROS package (and is
#       redundant with the compilation of Core and System for the toolkit).
add_subdirectory(../../Core ../build/Core)
add_subdirectory(../../System ../build/System)

add_subdirectory(ORUtils)
add_subdirectory(ITMLib)
add_subdirectory(Utils)
add_subdirectory(Engine)

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

# Include eigen and extensions (from Core).
include_directories(../../) # This path is used to find headers from [Core] and [System].
include_directories(${EIGEN_INCLUDE_DIRS})

add_executable(InfiniTAM_cli InfiniTAM_cli.cpp)
target_link_libraries(InfiniTAM_cli Engine)
target_link_libraries(InfiniTAM_cli Utils)
add_executable(InfiniTAM InfiniTAM.cpp)
target_link_libraries(InfiniTAM Engine)
target_link_libraries(InfiniTAM Utils)

