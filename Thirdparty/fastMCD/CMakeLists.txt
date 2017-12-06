cmake_minimum_required(VERSION 2.8)
project(fastMCD)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -o3")

#添加sse指令支持
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")

# Lib output
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Opencv
#set(OpenCV_DIR "/home/buyi/Software/Opencv/opencv-3.2.0/build")
find_package(OpenCV REQUIRED)
include_directories(${Opencv_INCLUDE_DIRS})
message(STATUS "    version: ${OpenCV_VERSION}")

# Eigen
include_directories("/usr/include/eigen3")


include_directories(
        ${PROJECT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} SHARED
        src/MCDWrapper.cpp
        src/KLTWrapper.cpp)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS})

