
//2021-03-03
#ifndef STRUCT_FEATURE_H
#define STRUCT_FEATURE_H

#include<memory>
#include"Eigen/Core"

namespace stereo{

class MapPoint;

class Feature{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Feature(const Eigen::Vector2f& p) : 
        pixel_(p){}

private:
    // --- DATA
    Eigen::Vector2f pixel_ = Eigen::Vector2f(0., 0.); // Position in the image
    float inv_depth = -1.0; 
    std::shared_ptr<MapPoint> map_point_ptr_ = nullptr;
    bool is_outlier_ = false;
};

}//namespace
#endif