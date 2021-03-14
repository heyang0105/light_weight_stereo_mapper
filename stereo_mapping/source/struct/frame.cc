#include"frame.h"

namespace steres_mapper{

Frame::Frame(    
    const Options& opt,        
    const cv::Mat& img_left, const cv::Mat& img_right,
    const double &timestamp,
    camodocal::CameraPtr cam_left, camodocal::CameraPtr cam_right){

    //
    grid.resize(options_.frame_grid_rows * options_.frame_grid_cols_);
}

Frame::Frame(const Frame &frame){

    grid.resize(options_.frame_grid_rows * options_.frame_grid_cols_);
}

void Frame::AssignFeaturesToGrid(){
    if (feature_left_.empty()) return;

    for (auto g: grid_)
        g.clear();

    unique_lock<mutex> lock(mMutexFeature);
        for (size_t i = 0; i < mFeaturesLeft.size(); i++) {
            shared_ptr<Feature> f = mFeaturesLeft[i];
            if (f == nullptr)
                continue;
            int nGridPosX, nGridPosY;
            if (PosInGrid(f, nGridPosX, nGridPosY)) {
                mGrid[nGridPosX + nGridPosY * setting::FRAME_GRID_COLS].push_back(i);
            }
        }
}

bool Frame::PosInGrid(
    const shared_ptr<Feature> feature, int &posX, int &posY){
        
 }

}//namespace