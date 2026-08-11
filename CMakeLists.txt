cmake_minimum_required(VERSION 3.16)

project(lidar_rover LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Utils library (no dependencies)
add_library(utils STATIC src/utils/I2c.cpp src/utils/scan_utils.cpp)
target_include_directories(utils PRIVATE include/)
 
# Lidar library (depends on utils)
add_library(lidar STATIC src/lidar/lidar.cpp)
target_include_directories(lidar PRIVATE include/)
target_link_libraries(lidar PRIVATE utils)

# Nav library (depends on lidar and motor)
add_library(nav STATIC src/nav/navigator.cpp)
target_include_directories(nav PRIVATE include/)
target_link_libraries(nav PRIVATE lidar motor)

# Motor library (no nav dependency)
add_library(motor STATIC src/motor/motor.cpp)
target_include_directories(motor PRIVATE include/)

# Main executable
add_executable(lidar_rover src/main.cpp)
target_include_directories(lidar_rover PRIVATE include/)
target_link_libraries(lidar_rover PRIVATE utils lidar nav motor)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
