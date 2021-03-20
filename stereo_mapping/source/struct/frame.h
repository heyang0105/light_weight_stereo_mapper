//2021-3-3
#ifndef STRUCT_FRAME_H
#define STRUCT_FRAME_H

// C++
#include <vector>
#include <memory>

#include <opencv2/core/core.hpp>

#include <Eigen/Core>

#include"sophus/se3.hpp"
#include"sophus/so3.hpp"

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "struct/feature.h"

#include "util/types.h"

using cv::Mat;
using namespace std;

namespace stereo_mapper{

//--------------------GRID CONFIG----------------------

class FrameGrid{
public:
    FrameGrid(const FrameGrid&) = delete;
    FrameGrid& operator=(const FrameGrid&) = delete;
    static FrameGrid& get_instance(){
        static FrameGrid instance;
        return instance;
    }

    //
    int frame_gird_size_;

    int boarder_;

    int img_height_;
    int img_width_;

    int frame_grid_rows_;
    int frame_grid_cols_; 

    double grid_element_width_inv;
    double grid_element_height_inv;

    //   TODO; call it at the system level   
    void SetFrameGrid(const int& frame_grid_size,
        const int& boarder,
        const int& img_height, 
        const int& img_width){
            
        boarder_ = boarder;

        frame_gird_size_ = frame_grid_size;

        img_height_  = img_height;
        img_width_ = img_width;

        frame_grid_rows_ = ceil(img_height / frame_grid_size);
        frame_grid_cols_ = ceil(img_width_ / frame_grid_size);

        grid_element_width_inv = 1./double(frame_grid_size);
        grid_element_height_inv = 1./double(frame_grid_size);
    }
private:
    FrameGrid(){}
};


//--------------------Frame----------------------
class Frame{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    struct Opitons{
        double pyramid_scale = 2.0;
        size_t pyramid_num = 3;
        size_t edge_thr = 19;
    };

    Frame(    
        const Options& opt,        
        const cv::Mat& img_left, const cv::Mat& img_right,
        const double &timestamp,
        camodocal::CameraPtr cam_left, camodocal::CameraPtr cam_right);
    
    Frame(const Frame &frame);//TODO is this necessary???

    /* ### FUNC ###*/
    // GET
    std::vector<cv::Mat> GetLeftPyramid();
    std::vector<cv::Mat> GetRightPyramid();

    /* set the index of each feature in the grid */
    void AssignFeaturesToGrid();

    void ComputeImagePyramid();

private:


    const Options options_;

    // ---Data
    double timestamp_ = 0.0;

    // orig image
    cv::Mat img_left_, img_right_;
    
    // left and right pyramid
    vector<cv::Mat> pyramid_left_;
    vector<cv::Mat> pyramid_right_;

    // cam model
    camodocal::CameraPtr 
        cam_left_ = nullptr, cam_right = nullptr;

    // features 
    std::mutex feature_mux_;
    std::vector<shared_ptr<Feature>> feature_left_;  
    std::vector<shared_ptr<Feature>> feature_right_;
    
    // index
    size_t cur_frame_id_ = 0;
    size_t keyframe_id_ = 0;

    // pose
    Eigen::Vector4d quat_c_w_;
    Eigen::Vector3d t_c_w_;

    Eigen::Vector4d quat_w_c_;
    Eigen::Vector3d t_w_c_;

    // info
    bool is_key_frame_ = false;

    // grid to get the index of features in it 
    std::vector<std::vector<std::size_t>> grid_;

};


}// namespace
#endif
