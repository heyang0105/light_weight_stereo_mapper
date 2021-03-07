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
    /* CTRL: num to calc the local map optim */
    int min_track_local_map_inlier_size;
    
    /*INIT: path to set the camera model */
    string cam_left_yaml_path;
    string cam_right_yaml_path;
};

    LKOpticFlowTracker() = delete;
    LKOpticFlowTracker(const Options& opt,
        JobQueue<inner::ImageData>* input_queue_ptr,
        JobQueue<inner::TrackData>* output_queue_ptr);

private:
    /* CORE FUNC */
    void StereoInit();

    void InsertKeyFrame(); // set the output jo queue

    void TrackByFeaturePoints(); //PnP + 2 frames BA

    void TrackByMapPoints();// local BA

    void ManageFeaturePoints();

    bool NeedKeyFrame();

    void CreateStereoMapPoints();

    void PredictCurPose();

    /* DATA */
    const Options option_;

    /* pipline data ptr */
    JobQueue<inner::ImageData>* input_queue_ptr_;
    JobQueue<inner::TrackData>* output_queue_ptr_;

    /* cameta model */
    camodocal::CameraPtr 
        cam_left_ = nullptr, cam_right = nullptr;


};

#endif
