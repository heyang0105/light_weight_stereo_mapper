#include"frontend_tracker.h"

namespace stereo_mapper{

LKOpticFlowTracker::LKOpticFlowTracker(
    const Options& opt,
    JobQueue<inner::ImageData>* input_queue_ptr,
    JobQueue<inner::TrackData>* output_queue_ptr):
    option_(opt), 
    input_queue_ptr_(input_queue_ptr),
    output_queue_ptr_(output_queue_ptr){
        

}

LKOpticFlowTracker::Run(){
    SignalValidSetup();
    while(true){
        if (IsStopped()) break; 
        const auto input_job = input_queue_ptr_->Pop();

        if (input_job.IsValid()) {
            
            //---1. set out
            

            //---2. prepare in 
            auto input_data = input_job.Date();
            cv::Mat left = input_data.left;
            cv::Mat right = input_data.right;
            double time = input_data.timestamp;
            
            /* TODO: add my func here start */

            /* TODO: add my func here end */

            //---3. propagate the data
            inner::TrackData out_data; // TODO
            output_queue_ptr->Push(out_data);
        }
        else{
            break; // end the whole process in a cascading way
        }
    }
}


}// namespace