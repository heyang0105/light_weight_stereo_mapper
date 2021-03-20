
//2021-03-03
#ifndef STRUCT_FEATURE_POINT_H
#define STRUCT_FEATURE_POINT_H

#include<memory>
#include"Eigen/Core"

namespace stereo_mapper{

class MapPoint;

class FeaturePoint{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    FeaturePoint(const Eigen::Vector2f& p) : 
        pixel_(p){}

    //----GET

private:
    // --- DATA
    Eigen::Vector2d pixel_ = Eigen::Vector2d(0., 0.); // Position in the image
    float inv_depth = -1.0; 
    /* Corresponding to a MapPoint */
    std::shared_ptr<MapPoint> map_point_ptr_ = nullptr;
    bool is_outlier_ = false;
};

}//namespace
#endif