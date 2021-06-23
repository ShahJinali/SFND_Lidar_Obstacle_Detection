cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(LidarObstacleDetection)

find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (environment src/environment.cpp
                            src/render/render.h src/render/render.cpp
                            src/render/box.h
                            src/sensors/lidar.h
                            src/processPointClouds.h src/processPointClouds.cpp)

target_link_libraries (environment ${PCL_LIBRARIES})

