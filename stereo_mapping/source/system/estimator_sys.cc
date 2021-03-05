// 2021-03-04

#include"estimator_sys.h"
#include"frontend_tracker.h"
#include"backend_optim.h"

namespace stereo{

StereoEstimator::StereoEstimator(const Options& opt) :
    options_(opt){

    // set thread
    /* TO CHANGE HERE IF THE MODULE IS REPLACED */
    front_end_tracker_ptr_.reset(
        new LKOpticFlowTracker(options_.frontend_tracker_opt));
    
    back_end_optim_ptr_.reset(
        new SlidingWinOptim(options_.backend_optim_opt));

    // set queue
    imgs_queue_ptr_.reset(
        new JobQueue<inner::ImageData>(options_.imgs_queue_size));

    track_queue_ptr_.rest(
        new JobQueue<inner::TrackData>(options_.track_queue_size));
}

void StereoEstimator::InsertData(const double& timestamp,
        const Mat& left, cons Mat& right){

    inner::ImageData in_data(timestamp, left, right);
    CHECK(imgs_queue_ptr->Push(in_data));
}

void StereoEstimator::Run(){
    //--- 1. init the threads
    front_end_tracker_->Start();

    back_end_optim_->Start();
    
    //--- 2. main loop    
    while(1){
        if(IsStopped()){
            imgs_queue_ptr_->Stop();
            imgs_queue_ptr_->Clear();
            track_queue_ptr_->Stop();
            track_queue_ptr_->Clear();
            break;
        }
    }

    //---3. bef end process
}

}// namespace