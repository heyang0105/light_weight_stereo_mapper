#include"frame.h"

namespace steres_mapper{

Frame::Frame(    
    const Options& opt,        
    const cv::Mat& img_left, const cv::Mat& img_right,
    const double &timestamp,
    camodocal::CameraPtr cam_left, camodocal::CameraPtr cam_right){

    //
    grid_.resize( FrameGrid::get_instance().frame_grid_cols_ * 
                FrameGrid::get_instance().frame_grid_rows_);
}

Frame::Frame(const Frame &frame){

    grid_.resize( FrameGrid::get_instance().frame_grid_cols_ * 
            FrameGrid::get_instance().frame_grid_rows_);
}

void Frame::AssignFeaturesToGrid(){
    if (feature_left_.empty()) return;

    for (auto g: grid_)
        g.clear();

    unique_lock<mutex> lock(feature_mutex_);
    for (size_t i = 0; i < feature_left_.size(); i++) {
        shared_ptr<Feature> f = feature_left_[i];
        if (f == nullptr)
            continue;
        int pos_x, pos_y;
        if (PosInGrid(f, pos_x, pos_y)) {
            grid_[pos_x + pos_y * FrameGrid::get_instance().frame_grid_cols_].push_back(i);
        }
    }
}

bool Frame::PosInGrid(
    const shared_ptr<Feature> feature, int &pos_x, int &pos_y){
        
    pos_x = int(feature->pixel_[0] * 
        FrameGrid::get_instance().grid_element_width_inv);
    pos_y = int(feature->pixel_[1] * 
        FrameGrid::get_instance().grid_element_height_inv);
    if (pos_x < 0 || pos_x >= FrameGrid::get_instance().frame_grid_cols_
        || pos_y < 0 || pos_y >= FrameGrid::get_instance().frame_grid_rows_)
            return false;
    return true;
}

void Frame::ComputeImagePyramid(){
        
    pyramid_left_.resize( options_.pyramid_num);
    pyramid_right_.resize(options_.pyramid_num);

    // ---1. calc the level
    vector<double> inv_scale_factors, scale_factors;
    
    scale_factors.assign(options_.pyramid_num, 1.0);
    for (size_t i = 1; i < options_.pyramid_num; i++) {
        scale_factors[i] = scale_factors[i - 1] * options_.pyramid_scale;
    }
            
    inv_scale_factor.assign(options_.pyramid_num, 1.0);
    for (size_t i = 1; i < options_.pyramid_num; i++) {
        inv_scale_factor[i] = 1.0 / scale_factors[i];
    }

    // ---2. get images
    for (size_t level = 0; level < options_.pyramid_num; ++level) {
        
        float inv_scale = inv_scale_factor[level];
        cv::Size sz(cvRound((double) img_left_.cols * inv_scale), 
                    cvRound((double) img_right_.rows * inv_scale));
        cv::Size whole_size(sz.width + options_.edge_thr * 2, 
                            sz.height + options_.edge_thr * 2);

        cv::Mat temp_left(whole_size, img_left_.type());
        cv::Mat temp_right(whole_size, img_right_.type());

        pyramid_left_[level] = 
            temp_left(cv::Rect(options_.edge_thr, options_.edge_thr, sz.width, sz.height));

        pyramid_right_[level] = 
            temp_right(cv::Rect(options_.edge_thr, options_.edge_thr, sz.width, sz.height));

        // Compute the resized image
        if (level != 0) {
            cv::resize(pyramid_left_[level - 1], pyramid_left_[level], 
                                            sz, 0, 0, cv::INTER_LINEAR);

            cv::resize(pyramid_right_[level - 1], pyramid_right_[level], sz, 0, 0, INTER_LINEAR);

            copyMakeBorder(pyramid_left_[level], temp_left, 
                            options_.edge_thr, options_.edge_thr,
                            options_.edge_thr, options_.edge_thr,
                            cv::BORDER_REFLECT_101 + cv::BORDER_ISOLATED);

            copyMakeBorder(pyramid_right_[level], temp_right, 
                            options_.edge_thr, options_.edge_thr,
                            options_.edge_thr, options_.edge_thr,
                            cv::BORDER_REFLECT_101 + cv::BORDER_ISOLATED);
        } 
        else {
            copyMakeBorder(img_left_, temp_left, 
                            options_.edge_thr, options_.edge_thr,
                            options_.edge_thr, options_.edge_thr,
                            cv::BORDER_REFLECT_101);
            copyMakeBorder(img_right_, temp_right, 
                            options_.edge_thr, options_.edge_thr,
                            options_.edge_thr, options_.edge_thr, 
                            cv::BORDER_REFLECT_101);
        }
    }//for
}  


}//namespace