
//2021-03-01
#ifndef STEREO_ESTIMATOR_H
#define STEREO_ESTIMATOR_H

#include<memory>
#include"util/threading.h"

#include"frontend_tracker.h"

#include <opencv2/opencv.hpp>

using namespace std;

namespace stereo{

namespace inner{
    struct ImageData;
}

class StereoEstimator : public Thread{
public:
    struct Options{

    };



private:
    const Options options_;
    void Run();

    // sub module
    unique_ptr<Thread> front_end_tracker_;
    unique_ptr<Thread> back_end_optim_;

    // data struct
    std::unique_ptr<JobQueue<internal::ImageData>> imgs_queue_;
};

}

#endif