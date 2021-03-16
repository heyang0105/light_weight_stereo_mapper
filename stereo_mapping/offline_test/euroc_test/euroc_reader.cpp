#include "euroc_reader.h"

#include <iomanip>

namespace euroc {

    bool LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                    vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps) {
        ifstream fTimes;
        fTimes.open(strPathTimes.c_str());
        if (!fTimes) {
            LOG(ERROR) << "cannot find timestamp file: " << strPathTimes << endl;
            return false;
        }
        vTimeStamps.reserve(5000);
        vstrImageLeft.reserve(5000);
        vstrImageRight.reserve(5000);

        while (!fTimes.eof()) {
            string s;
            getline(fTimes, s);
            if (!s.empty()) {
                stringstream ss;
                ss << s;
                vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
                vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
                double t;
                ss >> t;
                vTimeStamps.push_back(t / 1e9);
            }
        }
        fTimes.close();

        if (strPathLeft.empty()) {
            LOG(ERROR) << "No images in left folder!" << endl;
            return false;
        }

        if (strPathRight.empty()) {
            LOG(ERROR) << "No images in right folder!" << endl;
            return false;
        }
        return true;
    }

    bool LoadGroundTruthTraj(const string &trajPath,
                             TrajectoryType &trajectory) {

        ifstream fTraj(trajPath);
        if (!fTraj) {
            LOG(ERROR) << "cannot find trajectory file!" << endl;
            return false;
        }

        while (!fTraj.eof()) {
            string s;
            getline(fTraj, s);
            if (!s.empty()) {
                if (s[0] < '0' || s[0] > '9') // not a number
                    continue;

                stringstream ss;
                ss << s;
                double timestamp = 0;
                ss >> timestamp;
                ss.ignore();

                timestamp *= 1e-9;

                double data[7];
                for (double &d:data) {
                    ss >> d;
                    if (ss.peek() == ',' || ss.peek() == ' ')
                        ss.ignore();
                }

                // x,y,z,qw,qx,qy,qz
                Sophus::SE3d pose(Sophus::SO3d(Eigen::Quaterniond(data[3], data[4], data[5], data[6])),
                          Eigen::Vector3d(data[0], data[1], data[2]));
                trajectory[timestamp] = pose;
            }
        }

        fTraj.close();

        return true;
    }
}
