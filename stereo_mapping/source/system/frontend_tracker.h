//2021-03-01
// TODO: a finite state machine inside this thread

#ifndef STEREO_FRONT_END_H
#define STEREO_FRONT_END_H

//c++
#include<string>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include"util/threading.h"
#include"sophus/se3.hpp"

using Sophus::SE3d;
using namespace std;
using namespace UTIL;

namespace stereo{

namespace inner{
    struct ImageData;
    struct TrackData;
}

class LKOpticFlowTracker : public Thread{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

struct Options{
    string cam_left_yaml_path;
    string cam_right_yaml_path;
};

    LKOpticFlowTracker() = delete;
    LKOpticFlowTracker(const Options& opt,
        JobQueue<ImageData>* input_queue_ptr,
        JobQueue<TrackData>* output_queue_ptr);

    // SE3d InsertStereo(
    //             const cv::Mat &im_rect_left, 
    //             const cv::Mat &im_rect_right,  
    //             const double &timestamp);

private:

    const Options option_;

    JobQueue<ImageData>* input_queue_ptr_;
    JobQueue<TrackData>* output_queue_ptr_;

    camodocal::CameraPtr 
        cam_left_ = nullptr, cam_right = nullptr;

};

#endif