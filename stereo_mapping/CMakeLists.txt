cmake_minimum_required(VERSION 3.1)
project(STEREO_MAPPING LANGUAGES CXX C)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/source)

# these two are essential for the system
# find_package( Glog REQUIRED )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-3.7)

# Define the lib names
set(camera_model_lib "camera_model")
set(project_util_lib "util")
set(struct_lib "struct")

# integrated estimator system
set(stereo_estimator_system_lib "stereo_estimator")

#
add_subdirectory(source)
add_subdirectory(third_party)
