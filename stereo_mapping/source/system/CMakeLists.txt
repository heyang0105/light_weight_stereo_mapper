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

add_library(${stereo_estimator} SHARED
    ${util_h}
    ${util_cc})
