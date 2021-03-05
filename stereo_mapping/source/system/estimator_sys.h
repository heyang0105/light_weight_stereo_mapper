
//2021-03-01
#ifndef STEREO_ESTIMATOR_H
#define STEREO_ESTIMATOR_H

#include<memory>
#include"util/logging.h"
#include"util/threading.h"

#include"frontend_tracker.h"
#include"backend_optim.h"

#include <opencv2/opencv.hpp>

using namespace std;
using cv::Mat;

namespace stereo{

namespace inner{
    
    // PIPLINE DATA FLOW DEFINITION
    struct ImageData{
        double timestamp;
        Mat left;
        Mat right;
        ImageData(const double _t, const Mat& _l, const Mat& _r) :
            timestamp(_t), left(_l), right(_r){}
    };

    struct TrackData{
        // TODO
    };

    struct PoseData{
        //TODO
    };
}

class StereoEstimator : public Thread{
public:
    struct Options{
        size_t imgs_queue_size = 3;
        size_t track_queue_size = 3;
        LKOpticFlowTracker::Options frontend_tracker_opt;
        SlidingWinOptim::Options backend_optim_opt;
    };

    StereoEstimator(const Options& opt);

private:
    const Options options_;
    void Run();
    void InsertData(const double& timestamp,
         const Mat& left, cons Mat& right);

    // sub module
    unique_ptr<Thread> front_end_tracker_ptr_;
    unique_ptr<Thread> back_end_optim_ptr_;

    // data struct
    std::unique_ptr<JobQueue<inner::ImageData>> imgs_queue_ptr_;
    std::unqiue_ptr<JobQueue<inner::TrackData>> track_queue_ptr_;
};

}

#endif