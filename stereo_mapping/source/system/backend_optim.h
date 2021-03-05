// 2021-03-04
#ifndef STEREO_BACK_END_OPTIM_H
#define STEREO_BACK_END_OPTIM_H

#include"util/threading.h"

#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include"sophus/se3.hpp"

using Sophus::SE3d;

namespace stereo{

namespace inner{
    struct TrackData;
    struct PoseData;
}

class SlidingWinOptim : public Thread{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Options{

    };

    SlidingWinOptim() = delete;
    SlidingWinOptim(const Options& opt, 
        JobQueue<TrackData>* input_queue_ptr,
        JobQueue<PoseData>* output_queue_ptr);


private:
    const Options options_;

    JobQueue<TrackData>* input_queue_ptr_;
    JobQueue<PoseData>* output_queue_ptr_;
    
    void Run();
};

} //namespace

#endif