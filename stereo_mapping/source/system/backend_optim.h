// 2021-03-04
#ifndef STEREO_BACK_END_OPTIM_H
#define STEREO_BACK_END_OPTIM_H

#include<deque>
#include<set>
#include<memory>

#include"util/threading.h"

#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include"sophus/se3.hpp"

#include"data_que.h"

using Sophus::SE3d;
using namespace UTIL;

namespace stereo_mapper{

class Frame;
class MapPoint;

class SlidingWinOptim : public Thread{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Options{

    };

    SlidingWinOptim() = delete;
    SlidingWinOptim(const Options& opt, 
        JobQueue<inner::TrackData>* input_queue_ptr,
        JobQueue<inner::PoseData>* output_queue_ptr);


private:
    const Options options_;

    /* new coming frame from the frontend */
    JobQueue<inner::TrackData>* input_queue_ptr_;

    /* TODO to be defeined outcoming */
    JobQueue<inner::PoseData>* output_queue_ptr_;

    /* Slidng Window Frames*/
    std::deque<std::shared_ptr<Frame>> keyframes_deq_;
    std::set<std::shared_ptr<MapPoint>> map_points_set_;

    void Run();
};

} //namespace

#endif
