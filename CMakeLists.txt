# =====================================================================
# CMakeLists.txt
# ---------------------------------------------------------------------
# For an overview, check out:
#  http://www.cmake.org/cmake/help/v2.8.8/cmake.html
# =====================================================================

cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

# The top level project name. Sets the variable CMAKE_PROJECT_NAME.
project(PCT-CNN)

# Options are: None, Debug, Release, RelWithDebInfo and MinSizeRel
set(CMAKE_BUILD_TYPE Debug)

# Enable C++ 11 support.
add_definitions(-std=c++11)

message(STATUS "CMAKE_VERSION: ${CMAKE_VERSION}")
message(STATUS "CMAKE_SYSTEM: ${CMAKE_SYSTEM}")
message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "CMAKE_GENERATOR: ${CMAKE_GENERATOR}")
message(STATUS "CMAKE_PROJECT_NAME: ${CMAKE_PROJECT_NAME}")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "")

# ---------------------------------------------------------------------
# SECTION 1: CMake General Configuration
#
# The settings in this section apply for all subprojects.
# They can be overwritten by the CMakeLists.txt for each subproject.
# ---------------------------------------------------------------------

# Disabling in-source builds at this point results in an error with
#  the find_package(PCL 1.7 ...) command.
set(CMAKE_DISABLE_IN_SOURCE_BUILD OFF)
set(CMAKE_DISABLE_SOURCE_CHANGES ON)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE ON)

# ---------------------------------------------------------------------
# SECTION 2: Top Level Project Layout
#
# This section contains the organization of the top level directory.
# Configuration for every subproject needs to be included in the
#  corresponding CMakeLists.txt.
# ---------------------------------------------------------------------

set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR})
set(PROJECT_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

# Output directories for executables and libraries respectively.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_INCLUDE_DIR})

message(STATUS "PROJECT_NAME: ${PROJECT_NAME}")
message(STATUS "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
message(STATUS "PROJECT_INCLUDE_DIR: ${PROJECT_INCLUDE_DIR}")
message(STATUS "PROJECT_BINARY_DIR: ${PROJECT_BINARY_DIR}")
message(STATUS "")

# ---------------------------------------------------------------------
# SECTION 3: PCL Stuff
#
# Assuming that every subproject will use the PCL, it's reasonable to
#  put these settings in the top-level CMakeLists.txt here.
# ---------------------------------------------------------------------

# Available components are:
# --
#  common kdtree octree search sample_consensus filters features io
#  geometry visualization segmentation surface registration keypoints
#  tracking apps app_modeler app_cloud_composer app_in_hand_scanner
#  app_point_cloud_editor outofcore people
# --
set(PCL_COMPONENTS
    common
    kdtree
    octree
    search
    filters
    features
    io
    geometry
    visualization
    )

find_package(PCL 1.7 REQUIRED ${PCL_COMPONENTS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIR})
add_definitions(${PCL_DEFINITIONS})

# ---------------------------------------------------------------------
# SECTION 4: OpenCV Settings
#
# Include and linking flags for OpenCV.
# ---------------------------------------------------------------------

find_package(OpenCV REQUIRED)

message(STATUS)
message(STATUS "OpenCV version: " ${OpenCV_VERSION})

message(STATUS "Found the following OpenCV libraries:")
foreach(component ${OpenCV_LIB_COMPONENTS})
    message(STATUS "  " ${component})
endforeach()
message(STATUS)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

# ---------------------------------------------------------------------
# SECTION 5: Boost Settings
#
# Include and linking flags for Boost.
# ---------------------------------------------------------------------

# You have to add the required components manually.
# Some components are already in use by the pcl. Those compents are:
# --
#  system filesystem thread date_time iostream mpi serialization
# --
set(Boost_COMPONENTS
    program_options
    )

find_package(Boost COMPONENTS ${Boost_COMPONENTS} REQUIRED)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIR})
    link_directories(${Boost_LIBRARY_DIR})
endif()

# ---------------------------------------------------------------------
# SECTION 6: Subprojects
#
# Add your subproject directories here.
# ---------------------------------------------------------------------

add_subdirectory(demos/pcl_visualizer)
add_subdirectory(demos/icp_registration)
add_subdirectory(cnn_registration)
add_subdirectory(demos/pct_cnn_demo)
