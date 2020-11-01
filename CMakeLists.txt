cmake_minimum_required(VERSION 2.8.3)
project(stereo_state_estimator)

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O0 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(Eigen3 REQUIRED)
#find_package(Ceres REQUIRED)
#find_package(Pangolin REQUIRED)
set(OpenCV_DIR /home/yang/third_party/opencv-3.2.0/install/share/OpenCV)
find_package(OpenCV REQUIRED)
#find_package(Boost REQUIRED COMPONENTS filesystem program_options system)


message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

include_directories(
  ${PROJECT_SOURCE_DIR}/source
  ${EIGEN3_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIR}
  #${Pangolin_INCLUDE_DIRS}
)

set(CAM_MOD_FILES 
  source/camodocal/camera_models/Camera.cc
  source/camodocal/camera_models/CameraFactory.cc
  source/camodocal/camera_models/PinholeCamera.cc
  source/camodocal/camera_models/CataCamera.cc
  source/camodocal/camera_models/EquidistantCamera.cc
  source/camodocal/camera_models/ScaramuzzaCamera.cc
  source/camodocal/gpl/gpl.cc)


add_library(camera_model SHARED
    ${CAM_MOD_FILES})

target_link_libraries(camera_model  ${OpenCV_LIBS})



