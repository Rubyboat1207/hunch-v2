cmake_minimum_required(VERSION 3.12)
project(hunch VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(hunch main.cpp packet.cpp)

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

target_link_libraries(hunch PUBLIC Sockpp::sockpp)
target_link_libraries(hunch PRIVATE motorhat)
target_link_libraries(hunch PUBLIC ${OpenCV_LIBS})
include_directories(${CMAKE_SOURCE_DIR})





