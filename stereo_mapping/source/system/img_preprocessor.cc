//2021-03-04
#include"img_preprocessor.h"

namespace stereo{

ImgPreprocessor::ImgPreprocessor(
        const Options& opt,
        JobQueue<ImageData>* input_queue_ptr,
        JobQueue<ImageData>* output_queue_ptr) :
        
        options_(opt), 
        input_queue_ptr_(input_queue_ptr),
        output_queue_ptr_(output_queue_ptr){

}

void Run(){
    
}

}// namespace