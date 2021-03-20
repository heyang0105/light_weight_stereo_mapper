// 2021-03-15
/*
    This class only take charge of the match of left and right images
*/
#ifndef STEREO_MATCHER_H
#define STEREO_MATCHER_H

#include<memory>
#include"struct/frame.h"
using namespace std;

namespace stereo_mapper{

void ComputeStereoMatchesOptiFlow(
    shared_ptr<Frame> f_ptr, bool only_2D_points = false);

}// namespace
#endif