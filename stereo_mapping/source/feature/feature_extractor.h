// 2021-03-13
/* 
    Design:
    we set the unified interface that the frame's pointer will be put
    into the func. and we set the extracted content directly in the 
    frame. So these classes are actually a wrapper on the basic cv
    algorithms
*/
#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include<memory>
#include<vector>

#include <opencv2/core/core.hpp>

using namespace std;

namespace stereo_mapper{

struct Feature;

/* ---------------------------BASE CLASS-------------------------------*/
class FeatureExtractor{
public:
    struct Options{
        size_t feature_num = 500;
        size_t frame_grid_rows = 30;
        size_t frame_grid_cols = 30;
    };

    FeatureExtractor(const Options& opt) :
        options_(opt){}

    virtual void Detect(
                shared_ptr<Frame> frame_ptr, // set the extracted features directly in the frame
                bool left = true,     // extract which one
                bool compute_rot_desc = true   // if we need to compute the rotation and the descriptor?
        );
private:
    const Options options_;
    shared_ptr<Frame> frame_ptr_ = nullptr;
};

/*----------------------------CHILD CLASS--------------------------------*/
class FASTSingleLevelExtractor : public FeatureExtractor{
public:
    void Detect(
                shared_ptr<Frame> frame_ptr, // set the extracted features directly in the frame
                bool left = true,     // extract which one
                bool compute_rot_desc = true   // if we need to compute the rotation and the descriptor?
        );
private:
    void ComputeKeyPointsFastSingleLevel(
        std::vector<shared_ptr<Feature >> & key_points, 
        const cv::Mat &image);
};

class FASTMultiLevelExtractor : public FeatureExtractor{
public:
    void Detect(
                shared_ptr<Frame> frame_ptr, // set the extracted features directly in the frame
                bool left = true,     // extract which one
                bool compute_rot_desc = true   // if we need to compute the rotation and the descriptor?
        );
};

}//namespace
#endif