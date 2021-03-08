
//2021-03-01
#ifndef STEREO_ESTIMATOR_H
#define STEREO_ESTIMATOR_H

#include<memory>

//
#include"util/logging.h"
#include"util/threading.h"

// MODULES
#include"data_que.h"
#include"img_preprocessor.h"
#include"frontend_tracker.h"
#include"backend_optim.h"


using namespace UTIL;
namespace stereo_mapper{

class Frame;


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
          cv::Mat& left,  cv::Mat& right);

    // sub module
    /* process images: .i.e. photomatrically reimaging, or rectified, or contrast enhanced */
    /* # CUDA accelaration */
    std::unique_ptr<Thread> img_processor_ptr_ = nullptr;

    /* track each frame */
    std::unique_ptr<Thread> front_end_tracker_ptr_ = nullptr;

    /* optim keyframe window */
    std::unique_ptr<Thread> back_end_optim_ptr_ = nullptr;

    // data struct
    /* input raw data*/
    std::unique_ptr<JobQueue<inner::ImageData>> input_img_que_ptr_;
 
    /* processed images: .i.e. photomatrically reimaging, or rectified, or contrast enhanced */
    std::unique_ptr<JobQueue<inner::ImageData>> pro_img_que_ptr_;
 
    /* the output of the frontend track */
    std::unique_ptr<JobQueue<inner::TrackData>> track_que_ptr_;
 
    /* the output of the backend optim */
    std::unique_ptr<JobQueue<inner::PoseData>> pose_que_ptr_;
};


}//namespace

#endif