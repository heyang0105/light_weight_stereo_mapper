#include"feature/feature_extractor.h"
#include"struct/frame.h"

namespace stereo_mapper{

/* ---------------------------BASE CLASS-------------------------------*/
float FeatureExtractor::IC_Angle(const cv::Mat &image, 
        const Eigen::Vector2f &pt, const vector<int> &u_max){
    
    int m_01 = 0, m_10 = 0;
        
    const uchar *center = &image.at<uchar>(cvRound(pt[1]), cvRound(pt[0]));

    // Treat the center line differently, v=0
    for (int u = -options_.half_patch_size; u <= options_.half_patch_size; ++u)
        m_10 += u * center[u];

    // Go line by line in the circuI853lar patch
    int step = (int) image.step1();
    for (int v = 1; v <= options_.half_patch_size; ++v) {
        // Proceed over the two lines
        int v_sum = 0;
        int d = u_max[v];
        for (int u = -d; u <= d; ++u) {
            int val_plus = center[u + v * step], val_minus = center[u - v * step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }

    return fastAtan2((float) m_01, (float) m_10); //opencv func
}


/*-------------------FASTSingleLevelExtractor-----------------------*/
void FASTSingleLevelExtractor::Detect(
            shared_ptr<Frame> frame_ptr, 
            bool left = true,     
            bool compute_rot_desc = true ){

    frame_ptr_ = frame_ptr;

    compute_rot_desc_ = compute_rot_desc;

    vector<shared_ptr<Feature> > single_level_features;

    if (left) {
        ComputeKeyPointsFastSingleLevel(single_level_features, frame_ptr_->img_left_);
    } else {
        ComputeKeyPointsFastSingleLevel(single_level_features, frame_ptr_->img_right_);
    }
}

void FASTSingleLevelExtractor::ComputeKeyPointsFastSingleLevel(
    std::vector<shared_ptr<Feature >> & key_points, 
    const cv::Mat &image){

    frame_ptr_->AssignFeaturesToGrid();
    key_points.reserve(options_.feature_num * 2);
    int cnt = 0;   // 特征数
    // 试图在每个没有特征的网格中寻找一个特征
    for (size_t i = 1; i < FrameGrid::get_instance().frame_grid_rows_- 1; i++) {
        for (int j = 1; j < FrameGrid::get_instance().frame_grid_cols_ - 1; j++) {
            if (frame_ptr_->grid_[i * FrameGrid::get_instance().frame_grid_cols_ + j].empty()) {

                // 尝试在此网格中提取一个特征
                const uchar *data = 
                    image.ptr<uchar>(i * FrameGrid::get_instance().frame_gird_size_) + 
                                    j * FrameGrid::get_instance().frame_gird_size_;
                vector<fast::fast_xy> fast_corners;
#ifdef __SSE2__
                fast::fast_corner_detect_10_sse2(data, 
                    FrameGrid::get_instance().frame_gird_size_,
                    FrameGrid::get_instance().frame_gird_size_,
                    FrameGrid::get_instance().img_width_, 
                    options_.init_FAST_thr,
                    fast_corners);
#else
                fast::fast_corner_detect_10(data,
                    FrameGrid::get_instance().frame_gird_size_,
                    FrameGrid::get_instance().frame_gird_size_,
                    FrameGrid::get_instance().img_width_, 
                    options_.init_FAST_thr,
                    fast_corners);
#endif
                if (fast_corners.empty()) {
                // try lower threshold
#ifdef __SSE2__
                    fast::fast_corner_detect_10_sse2(data, 
                        FrameGrid::get_instance().frame_gird_size_,
                        FrameGrid::get_instance().frame_gird_size_,
                        FrameGrid::get_instance().img_width_, 
                        options_.min_FAST_thr,
                        fast_corners);
#else
                    fast::fast_corner_detect_10(data, 
                        FrameGrid::get_instance().frame_gird_size_,
                        FrameGrid::get_instance().frame_gird_size_,
                        FrameGrid::get_instance().img_width_, 
                        options_.min_FAST_thr,
                        fast_corners);
#endif
                }

                if (fast_corners.empty())
                    continue;

                // find the best one and insert as a feature
                int x_start = j * FrameGrid::get_instance().frame_gird_size_;
                int y_start = i * FrameGrid::get_instance().frame_gird_size_;

                // sort the corners according to shi-tomasi score
                vector<pair<fast::fast_xy, float> > corner_score;
                int idxBest = 0;
                float scoreBest = -1;
                for (int k = 0; k < fast_corners.size(); k++) {
                    fast::fast_xy &xy = fast_corners[k];
                    xy.x += x_start;
                    xy.y += y_start;

                    if (xy.x < FrameGrid::get_instance().boarder_ || 
                            xy.y < FrameGrid::get_instance().boarder_r ||
                            xy.x >= FrameGrid::get_instance().img_width_ - FrameGrid::get_instance().boarder_ ||
                            xy.y >= FrameGrid::get_instance().img_height_ - FrameGrid::get_instance().boarder_) {
                            // 太边缘不便于计划描述子
                            continue;
                    }

                    float score = ShiTomasiScore(image, xy.x, xy.y);
                    if (/* score > setting::minShiTomasiScore && */ score > scoreBest) {
                            scoreBest = score;
                            idxBest = k;
                        }
                    }

                    if (scoreBest < 0)
                        continue;

                    // 按score降序
                    fast::fast_xy &best = fast_corners[idxBest];
                    shared_ptr<Feature> feature(new Feature());
                    feature->pixel_ = Eigen::Vector2d(best.x, best.y);
                    
                    if (compute_rot_desc_)
                        feature->mAngle = IC_Angle(image, feature->mPixel, umax);
                    allKeypoints.push_back(feature);
                }
            }
        }

}
}//namespace