cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(exercise_03)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs CACHE PATH "Path to lib folder")
set(Eigen3_INCLUDE_DIR ${LIBRARY_DIR}/Eigen/ CACHE PATH "Path to Eigen source folder")

# Set C++ flags
set(CMAKE_CXX_STANDARD 14)


# Define header and source files
set(HEADERS
    Eigen.h
    ProcrustesAligner.h
    SimpleMesh.h
)

set(SOURCES
    main.cpp
)

add_executable(exercise_03 ${HEADERS} ${SOURCES})
target_include_directories(exercise_03 PUBLIC ${Eigen3_INCLUDE_DIR} ${FreeImage_INCLUDE_DIR})

# Visual Studio properties
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT exercise_03)
set_property(TARGET exercise_03 PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
