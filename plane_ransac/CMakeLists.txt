cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(LidarVisualPlane)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3")
# list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


include_directories("/usr/local/include/eigen3")

find_package(PCL REQUIRED)
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message("PCL_INCLUDE_DIRS = (${PCL_INCLUDE_DIRS}")
add_executable(plane_ransac
  src/plane_ransac.cpp)

target_link_libraries(plane_ransac
  ${PCL_LIBRARIES})

