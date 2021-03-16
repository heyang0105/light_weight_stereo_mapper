find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    message(STATUS "Don't get Opencv 3.0, looking for OpenCV 2.4.")
   find_package(OpenCV 2.4 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR})

file(
  GLOB_RECURSE
  cam_model_h
  *.h)

file(
  GLOB_RECURSE
  cam_model_cc
  *.cc)

add_library(${camera_model_lib} SHARED
    ${cam_model_h}
    ${cam_model_cc})

target_link_libraries( ${camera_model_lib} ${OpenCV_LIBS})
