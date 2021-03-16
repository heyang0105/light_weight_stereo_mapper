cmake_minimum_required(VERSION 2.8.3)
project(zed_wrapper_test)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
    image_transport)

find_package(OpenCV REQUIRED)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

# SET THE STEREO MAPPER PACKAGE
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../stereo_mapping/source/system
  ${CMAKE_CURRENT_SOURCE_DIR}/../stereo_mapping/source)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../stereo_mapping/build/lib)


add_executable(zed_test zed_wrapper_test.cpp)
target_link_libraries(zed_test 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  camera_model  
  stereo_estimator
  struct
  util)

