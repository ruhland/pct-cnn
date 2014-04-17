# ======================================================================
# Coherent Nearest Neighbour Registration Demo
# ======================================================================

# This subproject's name. Sets the variable PROJECT_NAME.
project(pct_cnn_demo)

# This project's directory structure.
set(PROJECT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}")
set(PROJECT_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bin")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

message(STATUS)
message(STATUS "Current subproject: ${PROJECT_NAME}")
message(STATUS "  Source directory: ${PROJECT_SOURCE_DIR}")
message(STATUS "  Include directory: ${PROJECT_INCLUDE_DIR}")
message(STATUS "  Binary directory: ${PROJECT_BINARY_DIR}")

# Finds the source files in the specified directory and
#  lists them in the given variable (2nd argument).
aux_source_directory(${PROJECT_SOURCE_DIR} PROJECT_SOURCES)

message(STATUS "Found sources: ${PROJECT_SOURCES}")

set(EXECUTABLE_NAME "pct_cnn_demo")

add_executable(${EXECUTABLE_NAME} ${PROJECT_SOURCES})

message(STATUS "Adding executable ${EXECUTABLE_NAME} in ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

target_link_libraries(${EXECUTABLE_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
