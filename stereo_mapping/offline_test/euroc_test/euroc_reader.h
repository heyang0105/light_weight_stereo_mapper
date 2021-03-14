#ifndef  EUROC_READER_H
#define  EUROC_READER_H

#include<fstream>
#include<sstream>
#include<string>
#include<map>
#include<vector>
#include"glog/logging.h"

#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

using namespace std;

namespace euroc {


    // Load the stereo image data
    // 输入：左眼图像目录，右眼图像目录，时间戳文件
    // 输出：排序后左眼图像文件路径、右眼图像文件路径、时间戳
    bool LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                    vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

    /**
     * Load the ground truth trajectory
     * @param [in] trajPath the path to trajectory, in euroc will be xxx/state_groundtruth_estimate0/data.csv
     * @param [out] the loaded trajectory
     * @return true if succeed
     */
    typedef map<double, Sophus::SE3d, std::less<double>, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

    bool LoadGroundTruthTraj(const string &trajPath,
                             TrajectoryType &trajectory);
} // namespace

#endif
