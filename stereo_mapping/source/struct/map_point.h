//2021-03-03
#ifndef STRUCT_MAP_POINT_H
#define STRUCT_MAP_POINT_H

#include<memory>
#include"Eigen/Core"

namespace stereo{

class Feature;

class Frame;

class MapPoint{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    enum State{
        GOOD = 0,    // 正常追踪
        IMMATURE,    // 不成熟的地图点（仅单次2D观测）
        BAD      // 追踪质量差，删除
    };



private:
    // ---DATA
    size_t idx_;
    State state_; 

    // pos
    Eigen::Vector3d p_w_ = Eigen::Vector3d(0., 0., 0.); 

    // first key frame
    std::shared_ptr<Frame> first_key_frame_ptr_;
};

} // stereo
#endif