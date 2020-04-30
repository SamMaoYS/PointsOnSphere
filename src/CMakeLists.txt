cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set(PCL_DIR "/home/sam/Development/PCL_LIB/PCL/share/pcl-1.9/")

project(point-on-sphere)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable (main main.cpp)
target_link_libraries (main ${PCL_LIBRARIES} ${OpenCV_LIBS} stdc++fs)
