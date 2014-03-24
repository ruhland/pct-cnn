# ======================================================================
# PCLVisualizer - Tutorial/Demo found on:
#  http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
# ======================================================================

# This subproject's name. Sets the variable PROJECT_NAME.
project(pcl_visualizer_demo)

# This project's directory structure.
set(PROJECT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}")
set(PROJECT_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bin")

message(STATUS)
message(STATUS "Current subproject: ${PROJECT_NAME}")
message(STATUS "  Source directory: ${PROJECT_SOURCE_DIR}")
message(STATUS "  Include directory: ${PROJECT_INCLUDE_DIR}")
message(STATUS "  Binary directory: ${PROJECT_BINARY_DIR}")

# Finds the source files in the specified directory and
#  lists the in the given variable (2nd argument).
aux_source_directory(${PROJECT_SOURCE_DIR} PROJECT_SOURCES)

message(STATUS "Found sources: ${PROJECT_SOURCES}")

add_executable(pcl_visualizer_demo ${PROJECT_SOURCES})
target_link_libraries(pcl_visualizer_demo ${PCL_LIBRARIES})
