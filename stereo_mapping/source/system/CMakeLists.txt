#  OPENCV, ONLY MAT ARE USED
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    message(STATUS "Don't get Opencv 3.0, looking for OpenCV 2.4.")
   find_package(OpenCV 2.4 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

#include_directories(${${OpenCV_INCLUDE_DIRS}})

#
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR})

file(
  GLOB_RECURSE
  syetem_h
  *.h)

file(
  GLOB_RECURSE
  system_cc
  *.cc)

add_library(${stereo_estimator_system_lib} SHARED
    ${syetem_h}
    ${system_cc})

target_link_libraries(${stereo_estimator_system_lib} 
  ${struct_lib}
  ${project_util_lib}
  ${camera_model_lib}
  ${OpenCV_LIBS})
