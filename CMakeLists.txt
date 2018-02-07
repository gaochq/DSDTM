cmake_minimum_required(VERSION 2.8)
project(DSDTM)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -o3")

#添加sse指令支持
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY  ${PROJECT_SOURCE_DIR}/lib)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


# Opencv
#set(OpenCV_DIR "/home/buyi/Software/Opencv/opencv-3.2.0/build")

find_package(OpenCV REQUIRED)
include_directories(${Opencv_INCLUDE_DIRS})
message(STATUS "    version: ${OpenCV_VERSION}")

# Ceres
find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
set(Sophus_LIBRARIES libSophus.so)
include_directories(${Sophus_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")

#glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

#Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/Thirdparty/fast/include
	${PROJECT_SOURCE_DIR}/Thirdparty/fastMCD/include)

add_library(${PROJECT_NAME} SHARED
    src/Frame.cpp
    src/Feature_detection.cpp
	src/Config.cpp
	src/Camera.cpp
	src/Tracking.cpp
	src/Initializer.cpp
	src/Rarsac_base.cpp
	src/MapPoint.cpp
	src/Map.cpp
	src/Keyframe.cpp
	src/Optimizer.cpp
	src/LocalMapping.cpp
	src/Moving_Detection.cpp
	src/Feature_alignment.cpp
	src/Sprase_ImageAlign.cpp
	src/Viewer.cpp
	src/System.cpp
	src/Utils.cpp)

target_link_libraries(${PROJECT_NAME}
		${OpenCV_LIBS}
		${CERES_LIBRARIES}
		${Sophus_LIBRARIES}
		${GLOG_LIBARIES}
		${Pangolin_LIBRARIES}
		${PROJECT_SOURCE_DIR}/Thirdparty/fast/lib/libfast.so
		${PROJECT_SOURCE_DIR}/Thirdparty/fastMCD/lib/libfastMCD.so)

# Test the feature detection
add_executable(test_Feature_detection Test/test_Feature_detection.cpp)
target_link_libraries(test_Feature_detection ${PROJECT_NAME})

# Test the tracking process
add_executable(test_Tracking Test/test_Tracking.cpp)
target_link_libraries(test_Tracking ${PROJECT_NAME})

# Test the Euroc Datasets
#add_executable(Test_Euroc Test/test_Euroc.cpp)
#target_link_libraries(Test_Euroc ${PROJECT_NAME})

# Test the tracking process
add_executable(test_Combination Test/test_Combination.cpp)
target_link_libraries(test_Combination ${PROJECT_NAME})

# Test the tracking process
add_executable(test_Jotting Test/test_Jotting.cpp)
target_link_libraries(test_Jotting ${PROJECT_NAME})

# Test the initial pose solver
add_executable(test_Optimizer Test/test_Optimizer.cpp)
target_link_libraries(test_Optimizer ${PROJECT_NAME})

#Test Motion Dection
add_executable(test_Motion_dection Test/test_Motion_detection.cpp)
target_link_libraries(test_Motion_dection ${PROJECT_NAME})

#Test Feature Alignment
add_executable(test_Feature_alignemnt Test/test_Feature_alignment.cpp)
target_link_libraries(test_Feature_alignemnt ${PROJECT_NAME})

#Test Sprase iamge Alignment
add_executable(test_SpraseImg_alignment Test/test_SpraseImg_alignment.cpp)
target_link_libraries(test_SpraseImg_alignment ${PROJECT_NAME})

#Test Tracking with SVO datasets
add_executable(test_uzhsin Test/test_uzhsin.cpp)
target_link_libraries(test_uzhsin ${PROJECT_NAME})



#Test Keyframe creating
#add_executable(KFrame_Creating Test/test_KFrame_Creating.cpp)
#target_link_libraries(KFrame_Creating ${PROJECT_NAME})