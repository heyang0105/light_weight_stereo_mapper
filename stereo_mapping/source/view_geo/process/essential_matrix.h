// 2021-03-27
#ifndef VIEW_GEO_ESSENTIAL_MATRIX_H_
#define VIEW_GEO_ESSENTIAL_MATRIX_H_

#include <vector>

#include <Eigen/Core>

#include "util/alignment.h"
#include "util/types.h"

namespace ViewGeo{

//
void DecomposeEssentialMatrix();

//
void PoseFromEssentialMatrix();

//
void EssentialMatrixFromPose();

//
void EssentialMatrixFromAbsolutePoses();

//
void FindOptimalImageObservations();

//
void EpipoleFromEssentialMatrix();

//
void EssentialMatrix();

}// namespace
#endif