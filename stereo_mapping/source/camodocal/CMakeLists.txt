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
