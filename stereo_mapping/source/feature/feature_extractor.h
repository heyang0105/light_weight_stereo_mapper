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
struct Frame;

/* ---------------------------BASE CLASS-------------------------------*/
class FeatureExtractor{
public:
    struct Options{
        size_t feature_num = 500;
        int init_FAST_thr = 20;
        int min_FAST_thr = 5;
        int feature_num = 200;
        int half_patch_size = 15;
    };

    FeatureExtractor(const Options& opt) :
        options_(opt){}

    virtual void Detect(
                shared_ptr<Frame> frame_ptr, // set the extracted features directly in the frame
                bool left = true,     // extract which one
                bool compute_rot_desc = true   // if we need to compute the rotation and the descriptor?
        );
protected:
    const Options options_;
    
    bool compute_rot_desc_ = false;

    shared_ptr<Frame> frame_ptr_ = nullptr;
    

    float IC_Angle(const cv::Mat &image, 
        const Eigen::Vector2f &pt, const vector<int> &u_max);

        // Shi-Tomasi 分数，这个分数越高则特征越优先
    inline float ShiTomasiScore(const cv::Mat &img, const int &u, const int &v) const {
        float dXX = 0.0;
        float dYY = 0.0;
        float dXY = 0.0;
        const int halfbox_size = 4;
        const int box_size = 2 * halfbox_size;
        const int box_area = box_size * box_size;
        const int x_min = u - halfbox_size;
        const int x_max = u + halfbox_size;
        const int y_min = v - halfbox_size;
        const int y_max = v + halfbox_size;

        if (x_min < 1 || x_max >= img.cols - 1 || y_min < 1 || y_max >= img.rows - 1)
            return 0.0; // patch is too close to the boundary

        const int stride = img.step.p[0];
        for (int y = y_min; y < y_max; ++y) {
            const uint8_t *ptr_left = img.data + stride * y + x_min - 1;
            const uint8_t *ptr_right = img.data + stride * y + x_min + 1;
            const uint8_t *ptr_top = img.data + stride * (y - 1) + x_min;
            const uint8_t *ptr_bottom = img.data + stride * (y + 1) + x_min;
            for (int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom) {
                float dx = *ptr_right - *ptr_left;
                float dy = *ptr_bottom - *ptr_top;
                dXX += dx * dx;
                dYY += dy * dy;
                dXY += dx * dy;
            }
        }

        // Find and return smaller eigenvalue:
        dXX = dXX / (2.0 * box_area);
        dYY = dYY / (2.0 * box_area);
        dXY = dXY / (2.0 * box_area);
        return 0.5 * (dXX + dYY - sqrt((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
    }
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


// TODO, IMPLEMENT IT LATER
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