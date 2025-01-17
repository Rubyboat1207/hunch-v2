cmake_minimum_required(VERSION 3.12)
project(hunch VERSION 1.0.0)

add_executable(hunch main.cpp)

set(MOTORHAT_DIR ${CMAKE_SOURCE_DIR}/adafruit-motor-hat-cpp-library)

# Sockpp
find_package(sockpp REQUIRED)

# MotorHat Project
file(GLOB MOTORHAT_SOURCES ${MOTORHAT_DIR}/source/*.cpp)
add_library(motorhat STATIC ${MOTORHAT_SOURCES})
target_include_directories(hunch PRIVATE ${MOTORHAT_DIR}/source)


target_link_libraries(hunch PUBLIC Sockpp::sockpp)
target_link_libraries(hunch PRIVATE motorhat)







