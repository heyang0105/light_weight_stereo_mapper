
//2021-03-01
#ifndef STEREO_ESTIMATOR_H
#define STEREO_ESTIMATOR_H

#include<memory>

//
#include"util/logging.h"
#include"util/threading.h"

// MODULES
#include"img_preprocessor.h"
#include"frontend_tracker.h"
#include"backend_optim.h"

#include <opencv2/opencv.hpp>

using namespace std;
using cv::Mat;

namespace stereo{

class Frame;

namespace inner{
    
    // PIPLINE DATA FLOW DEFINITIONS
    /* store the raw img data */
    struct ImageData;

    /* store the tracked fame */
    struct TrackData;

    /* store the calced pose */
    struct PoseData;
}

class StereoEstimator : public Thread{
public:
    struct Options{
        /* JobQueue */
        size_t in_img_que_size = 3;
        size_t pro_img_que_size = 3;
        size_t track_que_size = 3;
        size_t pose_que_size = 5;

        /* Thread */
        ImgPreprocessor::Options img_preprocessor_opt;
        LKOpticFlowTracker::Options frontend_tracker_opt;
        SlidingWinOptim::Options backend_optim_opt;
    };

    StereoEstimator(const Options& opt);

private:
    const Options options_;
    void Run();
    void InsertData(const double& timestamp,
         const Mat& left, const Mat& right);

    // sub module
    /* process images: .i.e. photomatrically reimaging, or rectified, or contrast enhanced */
    /* # CUDA accelaration */
    unique_ptr<Thread> img_processor_ptr_ = nullptr;

    /* track each frame */
    unique_ptr<Thread> front_end_tracker_ptr_ = nullptr;

    /* optim keyframe window */
    unique_ptr<Thread> back_end_optim_ptr_ = nullptr;

    // data struct
    /* input raw data*/
    std::unique_ptr<JobQueue<inner::ImageData>> input_img_que_ptr_;
 
    /* processed images: .i.e. photomatrically reimaging, or rectified, or contrast enhanced */
    std::unique_ptr<JobQueue<inner::ImageData>> pro_img_que_ptr_;
 
    /* the output of the frontend track */
    std::unqiue_ptr<JobQueue<inner::TrackData>> track_que_ptr_;
 
    /* the output of the backend optim */
    std::unqiue_ptr<JobQueue<inner::PoseData>> pose_que_ptr_;
};

/*****************************IMPLEMENTATION****************************/
namespace inner{
    
    // PIPLINE DATA FLOW DEFINITIONS
    struct ImageData{
        double timestamp;
        Mat left;
        Mat right;
        ImageData(const double _t, const Mat& _l, const Mat& _r) :
            timestamp(_t), left(_l), right(_r){}
    };

    struct TrackData{
        // TODO
        /* design: some other infos may be added here to support the Image data */
        shared_ptr<Frame> frame_ptr;
    };

    struct PoseData{
        //TODO
    };
}

}//namespace

#endif