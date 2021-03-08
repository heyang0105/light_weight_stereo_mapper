//2021-3-3
#ifndef STRUCT_FRAME_H
#define STRUCT_FRAME_H

// C++
#include <vector>
#include <memory>

#include <opencv2/core/core.hpp>

#include <Eigen/Core>

#include"sophus/se3.hpp"
#include"sophus/so3.hpp"

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "struct/feature.h"

#include "util/types.h"

using cv::Mat;
using namespace std;

namespace stereo_mapper{

class Frame{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    Frame(            
        const cv::Mat& img_left, const cv::Mat& img_right,
        const double &timestamp,
        camodocal::CameraPtr cam_left, camodocal::CameraPtr cam_right);
    
    Frame(const Frame &frame);



private:
    // ---Data
    double timestamp_ = 0.0;

    // orig image
    Mat img_left_, img_right_;
    
    // left and right pyramid
    vector<Mat> pyramid_left;
    vector<Mat> pyramid_right;

    // cam model
    camodocal::CameraPtr 
        cam_left_ = nullptr, cam_right = nullptr;

    // features TODO
    std::list<shared_ptr<Feature>> feature_left_;  
    std::list<shared_ptr<Feature>> feature_right_;
    
    // index
    size_t cur_frame_id_ = 0;
    size_t keyframe_id_ = 0;

    // pose
    Eigen::Vector4d quat_w_c_;
    Eigen::Vector3d t_w_c_;

    // info
    bool is_key_frame_ = false;

};


}// namespace
#endif
