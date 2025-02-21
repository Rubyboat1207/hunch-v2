cmake_minimum_required(VERSION 3.12)
project(hunch VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(hunch main.cpp packet.cpp state_machine.cpp utils.cpp lua_commands.cpp)

set(MOTORHAT_DIR ${CMAKE_SOURCE_DIR}/adafruit-motor-hat-cpp-library)
set(OPENCV_INCLUDE_DIR /usr/local/include/opencv4)

# Sockpp
find_package(sockpp REQUIRED)

# MotorHat Project
file(GLOB MOTORHAT_SOURCES ${MOTORHAT_DIR}/source/*.cpp)
add_library(motorhat STATIC ${MOTORHAT_SOURCES})
target_include_directories(hunch PRIVATE ${MOTORHAT_DIR}/source)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Lua
find_package(Lua REQUIRED)
target_include_directories(hunch PRIVATE ${LUA_INCLUDE_DIR})

target_link_libraries(hunch PUBLIC Sockpp::sockpp)
target_link_libraries(hunch PRIVATE motorhat)
target_link_libraries(hunch PUBLIC ${OpenCV_LIBS})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${LUA_INCLUDE_DIR})
target_link_libraries(hunch PUBLIC ${LUA_LIBRARIES})


if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    add_definitions(-DDBG)  # Define WIN32 for your compiler
    add_definitions(-DHOST=\"127.0.0.1\")
endif()

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(CMAKE_C_COMPILER_LAUNCHER ccache)
    set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
    
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Debug)  # Default to Debug if not set
    endif()

    # Apply compilation flags based on build type
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        message(STATUS "Compiling in Debug mode (-Og)")
        set(CMAKE_CXX_FLAGS_DEBUG "-Og -g")
    elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
        message(STATUS "Compiling in Release mode (-O2 -s)")
        set(CMAKE_CXX_FLAGS_RELEASE "-O2 -s")
    endif()
endif()



