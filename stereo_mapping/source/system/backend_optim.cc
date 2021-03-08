#include"backend_optim.h"

namespace stereo_mapper{

SlidingWinOptim::SlidingWinOptim(
    const Options& opt,
    JobQueue<inner::TrackData>* input_queue_ptr,
    JobQueue<inner::PoseData>* output_queue_ptr):
        options_(opt),
        input_queue_ptr_(input_queue_ptr),
        output_queue_ptr_(output_queue_ptr){

}

void SlidingWinOptim::Run(){
    SignalValidSetup();
    while(true){
        if (IsStopped()) break; 
        const auto input_job = input_queue_ptr_->Pop();

        if (input_job.IsValid()) {
            
            //---1. set out
            

            //---2. prepare in 
            auto input_data = input_job.Data();
            
            /* TODO: add my func here start */

            /* TODO: add my func here end */

            //---3. propagate the data
            inner::PoseData out_data; // TODO
            output_queue_ptr_->Push(out_data);
        }
        else{
            break; // end the whole process in a cascading way
        }
    }
}

} // namespace
