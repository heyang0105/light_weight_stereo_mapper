//2021-03-04
#include"img_preprocessor.h"
// ### TODO
// add functinality here

//

namespace stereo_mapper{

ImgPreprocessor::ImgPreprocessor(
    const Options& opt,
    JobQueue<inner::ImageData>* input_queue_ptr,
    JobQueue<inner::ImageData>* output_queue_ptr) :
        
    options_(opt), 
    input_queue_ptr_(input_queue_ptr),
    output_queue_ptr_(output_queue_ptr){

}

void ImgPreprocessor::Run(){
    SignalValidSetup();
    while(true){
        if (IsStopped()) break; 
        const auto input_job = input_queue_ptr_->Pop();

        if (input_job.IsValid()) {
            
            //---1. set out
            Mat out_left, out_right;

            //---2. prepare in 
            auto input_data = input_job.Data();
            cv::Mat left = input_data.left;
            cv::Mat right = input_data.right;
            double time = input_data.timestamp;
            
            /* TODO: add my func here start */

            /* TODO: add my func here end */

            //---3. propagate the data
            inner::ImageData out_data;
            output_queue_ptr->Push(out_data);
        }
        else{
            break; // end the whole process in a cascading way
        }
    }
}

}// namespace
