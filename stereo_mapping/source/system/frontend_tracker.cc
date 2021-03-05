#include"frontend_tracker.h"

namespace stereo{

LKOpticFlowTracker::LKOpticFlowTracker(
    const Options& opt,
    JobQueue<ImageData>* input_queue_ptr,
    JobQueue<TrackData>* output_queue_ptr):
    option_(opt), 
    input_queue_ptr_(input_queue_ptr),
    output_queue_ptr_(output_queue_ptr){
        

}

LKOpticFlowTracker::Run(){

}


}// namespace