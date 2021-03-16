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

}//namespace