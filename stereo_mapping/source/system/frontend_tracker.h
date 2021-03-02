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

namespace stereo{

class LKOpticFlowTracker : public Thread{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

struct Options{
    string cam_left_yaml_path;
    string cam_right_yaml_path;
};

    LKOpticFlowTracker(const Options& opt);

    SE3d InsertStereo(
                const cv::Mat &im_rect_left, 
                const cv::Mat &im_rect_right,  // 左右两图
                const double &timestamp);

private:

    const Options option_;

    camodocal::CameraPtr 
        cam_left_ = nullptr, cam_right = nullptr;

};

#endif