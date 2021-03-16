add_executable(euroc_test 
    euroc_stereo_main.cpp
    euroc_reader.h
    euroc_reader.cpp )

target_link_libraries( euroc_test 
    ${camera_model_lib}
    ${project_util_lib}
    ${struct_lib}
    ${stereo_estimator_system_lib}
    ${GLOG_LIBRARY})
