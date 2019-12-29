cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(exercise_01)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../libs CACHE PATH "Path to lib folder")
set(Eigen3_INCLUDE_DIR ${LIBRARY_DIR}/Eigen/ CACHE PATH "Path to Eigen source folder")

# Use pre-compiled libraries on Windows
if(WIN32)
    set(FreeImage_INCLUDE_DIR ${LIBRARY_DIR}/FreeImage/x64/ CACHE PATH "Path to FreeImage header file")
    set(FreeImage_LIBRARY_DIR ${LIBRARY_DIR}/FreeImage/x64/ CACHE PATH "Path to FreeImage .lib/.dll folder")
endif(WIN32)

# Set C++ flags
set(CMAKE_CXX_STANDARD 14)


set(HEADERS 
    Eigen.h
    FreeImageHelper.h
    VirtualSensor.h
)

set(SOURCES
    main.cpp
    FreeImageHelper.cpp
)

link_directories(${FreeImage_LIBRARY_DIR})
add_executable(exercise_01 ${HEADERS} ${SOURCES})
target_include_directories(exercise_01 PUBLIC ${Eigen3_INCLUDE_DIR} ${FreeImage_INCLUDE_DIR})
target_link_libraries(exercise_01 general freeimage)

# Visual Studio properties
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT exercise_01)
set_property(TARGET exercise_01 PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)

# On Windows copy dll to executable folder
if(WIN32)
    add_custom_command(TARGET exercise_01 POST_BUILD        # Adds a post-build event to MyTest
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${LIBRARY_DIR}/FreeImage/x64/FreeImage.dll"      # <--this is in-file
            $<TARGET_FILE_DIR:exercise_01>)                 # <--this is out-file path
endif(WIN32)
