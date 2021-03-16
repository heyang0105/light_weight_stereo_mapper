//2021-03-08
#ifndef SYSTEM_DATA_QUE_H
#define SYSTEM_DATA_QUE_H

#include <opencv2/opencv.hpp>

namespace stereo_mapper{

class Frame;

namespace inner{
    
    // PIPLINE DATA FLOW DEFINITIONS
    /* store the raw img data */
    struct ImageData{
        double timestamp;
        cv::Mat left;
        cv::Mat right;
        // ImageData(const double _t,  cv::Mat& _l,  cv::Mat& _r) :
        //     timestamp(_t), left(_l), right(_r){}
    };

     /* store the tracked fame */
    struct TrackData{
        // TODO
        /* design: some other infos may be added here to support the Image data */
        std::shared_ptr<Frame> frame_ptr;
    };

    /* store the calced pose */
    struct PoseData{
        //TODO
    };
} // inner
}// stereo_mapper
#endif