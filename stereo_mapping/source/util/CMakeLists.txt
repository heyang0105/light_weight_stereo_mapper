include_directories(
	${CMAKE_CURRENT_SOURCE_DIR})

file(
  GLOB_RECURSE
  util_h
  *.h)

file(
  GLOB_RECURSE
  util_cc
  *.cc)

add_library(${project_util_lib} SHARED
    ${util_h}
    ${util_cc})
