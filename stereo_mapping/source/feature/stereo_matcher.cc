#include"stereo_matcher.h"

namespace stereo_mapper{

void ComputeStereoMatchesOptiFlow(
    shared_ptr<Frame> f_ptr, bool only_2D_points) {
    
    assert(!f_ptr->feature_left_.empty());
    
    if (f_ptr->pyramid_left_.empty() || f_ptr->pyramid_right_.empty())
        f_ptr->ComputeImagePyramid();

    // 对于那些未关联地图点的特征，或关联了未成熟地图点的特征，尝试通过双目估计其深度
    for (int i = 0; i < f_ptr->feature_left_.size(); i++) {
        auto &feat = f_ptr->feature_left_[i];
        
        if (feat == nullptr){
            continue;
        }
           
        if (only_2D_points && feat->mpPoint &&
            feat->map_point_ptr_->GetState() == MapPoint::GOOD){
            continue;
        }    // already have a good point
            
        Eigen::Vector2d pl = feat->pixel_;
        Eigen::Vector2d pr = feat->pixel_;
        bool ret = LKFlowSinglePoint(
                f_ptr->mPyramidLeft, f->mPyramidRight, feat->mPixel, pr);

        if (ret) {
            // check the right one
            if (pl[0] < pr[0] || (fabs(pl[1] - pr[1]) > setting::stereoMatchingTolerance)) {
                continue;
            } else {
                float disparity = pl[0] - pr[0];
                if (disparity > 1)    // avoid zero disparity
                    feat->mfInvDepth = disparity / f->mpCam->bf;
            }
        }
    }
}

}// namespace