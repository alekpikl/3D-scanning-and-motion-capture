cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(exercise_04)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs CACHE PATH "Path to lib folder")
set(Eigen3_INCLUDE_DIR ${LIBRARY_DIR}/Eigen/ CACHE PATH "Path to Eigen source folder")
# set(Ceres_INCLUDE_DIR ${LIBRARY_DIR}/ceres-solver/include CACHE PATH "Path to ceres source folder")
# set(glog_DIR_HINT ${LIBRARY_DIR}/glog-lib/ CACHE PATH "Path to glog source folder")
set(glog_INCLUDE_DIR ${LIBRARY_DIR}/glog-lib/include CACHE PATH "Path to glog source folder")
set(Ceres_DIR ${LIBRARY_DIR}/Ceres/CMake CACHE PATH "Path to Ceres build folder")
set(Ceres_INCLUDE_DIR ${Ceres_DIR}/../include)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${Ceres_DIR})

# Set C++ flags
set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS_RELEASE "/MDd")
# set(CMAKE_CXX_FLAGS_DEBUG "/MDd")

find_package(Ceres REQUIRED)
find_package(glog REQUIRED ${glog_DIR_HINT})

add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")


# Define header and source files
set(UTILS
    utils/io.h
    utils/points.h
)

message(STATUS "ceres library name: ${Ceres_LIBRARIES}")
message(STATUS "path to ceres cmake information: ${Ceres_DIR}")
message(STATUS "path to ceres include directory: ${Ceres_INCLUDE_DIR}")
message(STATUS "glog library name: ${glog_LIBRARIES}")
message(STATUS "path to glog include directory = ${glog_INCLUDE_DIR}")

add_executable(gaussian ${UTILS} gaussian.cpp)
add_executable(dragon ${UTILS} dragon.cpp)
add_executable(surface ${UTILS} surface.cpp)

target_link_libraries(gaussian ceres glog::glog)
target_link_libraries(dragon ceres glog::glog)
target_link_libraries(surface ceres glog::glog)


target_include_directories(gaussian PUBLIC ${Eigen3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}  ${Ceres_INCLUDE_DIR} ${glog_INCLUDE_DIR})
target_include_directories(dragon PUBLIC ${Eigen3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${Ceres_INCLUDE_DIR} ${glog_INCLUDE_DIR})
target_include_directories(surface PUBLIC ${Eigen3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${Ceres_INCLUDE_DIR} ${glog_INCLUDE_DIR})

# # Visual Studio properties
# set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT exercise_03)
# set_property(TARGET exercise_03 PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)


