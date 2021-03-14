#include"feature_extractor.h"

namespace stereo_mapper{

/*-------------------FASTSingleLevelExtractor-----------------------*/
void FASTSingleLevelExtractor::Detect(
            shared_ptr<Frame> frame_ptr, 
            bool left = true,     
            bool compute_rot_desc = true ){

    frame_ptr_ = frame_ptr;
    vector<shared_ptr<Feature> > single_level_features;

    if (left) {
        ComputeKeyPointsFastSingleLevel(single_level_features, frame->mImLeft);
    } else {
        ComputeKeyPointsFastSingleLevel(single_level_features, frame->mImRight);
    }
}

void FASTSingleLevelExtractor::ComputeKeyPointsFastSingleLevel(
    std::vector<shared_ptr<Feature >> & key_points, 
    const cv::Mat &image){

    frame_ptr->AssignFeaturesToGrid();
    key_points.reserve(options_.feature_num * 2);
    int cnt = 0;   // 特征数
    // 试图在每个没有特征的网格中寻找一个特征
    for (size_t i = 1; i < options_.frame_grid_rows- 1; i++) {
        for (int j = 1; j < options_.frame_grid_cols - 1; j++) {
            if (mpFrame->mGrid[i * options_.frame_grid_cols + j].empty()) {

                    // 尝试在此网格中提取一个特征
                    const uchar *data = image.ptr<uchar>(i * setting::FRAME_GRID_SIZE) + j * setting::FRAME_GRID_SIZE;
                    vector<fast::fast_xy> fast_corners;
#ifdef __SSE2__
                    fast::fast_corner_detect_10_sse2(data, setting::FRAME_GRID_SIZE, setting::FRAME_GRID_SIZE,
                                                     setting::imageWidth, setting::initTHFAST, fast_corners);
#else
                    fast::fast_corner_detect_10(data, setting::FRAME_GRID_SIZE, setting::FRAME_GRID_SIZE, setting::imageWidth, setting::initTHFAST, fast_corners);
#endif
                    if (fast_corners.empty()) {
                        // try lower threshold
#ifdef __SSE2__
                        fast::fast_corner_detect_10_sse2(data, setting::FRAME_GRID_SIZE, setting::FRAME_GRID_SIZE,
                                                         setting::imageWidth, setting::minTHFAST, fast_corners);
#else
                        fast::fast_corner_detect_10(data, setting::FRAME_GRID_SIZE, setting::FRAME_GRID_SIZE, setting::imageWidth, setting::minTHFAST, fast_corners);
#endif
                    }

                    if (fast_corners.empty())
                        continue;

                    // find the best one and insert as a feature
                    int x_start = j * setting::FRAME_GRID_SIZE;
                    int y_start = i * setting::FRAME_GRID_SIZE;

                    // sort the corners according to shi-tomasi score
                    vector<pair<fast::fast_xy, float> > corner_score;
                    int idxBest = 0;
                    float scoreBest = -1;
                    for (int k = 0; k < fast_corners.size(); k++) {
                        fast::fast_xy &xy = fast_corners[k];
                        xy.x += x_start;
                        xy.y += y_start;

                        if (xy.x < setting::boarder || xy.y < setting::boarder ||
                            xy.x >= setting::imageWidth - setting::boarder ||
                            xy.y >= setting::imageHeight - setting::boarder) {
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
                    feature->mPixel = Vector2f(best.x, best.y);
                    if (mbComputeRotAndDesc)
                        feature->mAngle = IC_Angle(image, feature->mPixel, umax);
                    allKeypoints.push_back(feature);
                }
            }
        }

}
}//namespace