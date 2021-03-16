// 2021-03-04
#include"estimator_sys.h"

namespace stereo_mapper{

StereoEstimator::StereoEstimator(const Options& opt) :
    options_(opt){

    // 1. set queue
    input_img_que_ptr_.reset(
        new JobQueue<inner::ImageData>(options_.in_img_que_size));
    
    pro_img_que_ptr_.reset(
        new JobQueue<inner::ImageData>(options_.pro_img_que_size));

    track_que_ptr_.reset(
        new JobQueue<inner::TrackData>(options_.track_que_size));

    pose_que_ptr_.reset(
        new JobQueue<inner::PoseData>(options_.pose_que_size));

    // 2. set thread + data pipline
    /* TO CHANGE HERE IF THE MODULE IS REPLACED */
    img_processor_ptr_.reset(
        new ImgPreprocessor(options_.img_preprocessor_opt,
           input_img_que_ptr_.get(),
            pro_img_que_ptr_.get()));

    front_end_tracker_ptr_.reset(
        new LKOpticFlowTracker(options_.frontend_tracker_opt,
         pro_img_que_ptr_.get(),
         track_que_ptr_.get()));
    
    back_end_optim_ptr_.reset(
        new SlidingWinOptim(options_.backend_optim_opt,
            track_que_ptr_.get(), 
            pose_que_ptr_.get()));

}

void StereoEstimator::InsertData(const double& timestamp,
         cv::Mat& left,  cv::Mat& right){

    // inner::ImageData in_data(timestamp, left, right);
    inner::ImageData in_data;
    in_data .timestamp = timestamp;
    left.copyTo(in_data.left);
    right.copyTo(in_data.right);
    CHECK(input_img_que_ptr_->Push(in_data));
}

void StereoEstimator::Run(){
    //--- 1. init the threads
    img_processor_ptr_->Start();
    if (!img_processor_ptr_->CheckValidSetup()) return;

    front_end_tracker_ptr_->Start();
    if (!front_end_tracker_ptr_->CheckValidSetup()) return;

    back_end_optim_ptr_->Start();
    if (!back_end_optim_ptr_->CheckValidSetup()) return;
    
    //--- 2. main loop    
    while(true){
        if(IsStopped()){
            input_img_que_ptr_->Stop();
            input_img_que_ptr_->Clear();
            pro_img_que_ptr_->Stop();
            pro_img_que_ptr_->Clear();
            track_que_ptr_->Stop();
            track_que_ptr_->Clear();
            pose_que_ptr_->Stop();
            pose_que_ptr_->Clear();
            break;
        }

        /* should be blocked here, if no image insert */
        const auto input_job = input_img_que_ptr_->Pop();
        if (input_job.IsValid()) {
            
            //---1. set out
            

            //---2. prepare in 
            auto input_data = input_job.Data();
            cv::Mat left = input_data.left;
            cv::Mat right = input_data.right;
            double time = input_data.timestamp;

            /* TODO: add my func here start */

            /* TODO: add my func here end */

            //---3. propagate the data
        }
        else{
            break; // end the whole process in a cascading way
        }
    }

    //---3. bef end process
    //  ---3.1 log TODO

    //  ---3.2 end the system
    input_img_que_ptr_->Wait();
    input_img_que_ptr_->Stop();

    img_processor_ptr_->Wait();

    pro_img_que_ptr_->Wait();
    pro_img_que_ptr_->Stop();

    front_end_tracker_ptr_->Wait();

    track_que_ptr_->Wait();
    track_que_ptr_->Stop();

    back_end_optim_ptr_->Wait();

    pose_que_ptr_->Wait();
    pose_que_ptr_->Stop();

}

}// namespace
