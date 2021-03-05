#include"backend_optim.h"

namespace stereo{

SlidingWinOptim::SlidingWinOptim(
    const Options& opt,
    JobQueue<TrackData>* input_queue_ptr,
    JobQueue<PoseData>* output_queue_ptr):
        options_(opt),
        input_queue_ptr_(input_queue_ptr),
        output_queue_ptr_(output_queue_ptr){

}

SlidingWinOptim::Run(){

}

} // namespace
