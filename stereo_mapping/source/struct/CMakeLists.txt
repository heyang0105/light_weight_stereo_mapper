include_directories(
	${CMAKE_CURRENT_SOURCE_DIR})

file(
  GLOB_RECURSE
  struct_h
  *.h)

file(
  GLOB_RECURSE
  struct_cc
  *.cc)

add_library(${struct_lib} SHARED
    ${struct_h}
    ${struct_cc})
