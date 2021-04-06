// 2021-03-27
#ifndef VIEW_GEO_HOMOGRAPHY_MATRIX_H_
#define VIEW_GEO_HOMOGRAPHY_MATRIX_H_

#include <vector>

#include <Eigen/Core>

#include"view_geo.h"

namespace ViewGeo {

void DecomposeHomographyMatrix(const Eigen::Matrix3d& H,
                               const Eigen::Matrix3d& K1,
                               const Eigen::Matrix3d& K2,
                               std::vector<Eigen::Matrix3d>* R,
                               std::vector<Eigen::Vector3d>* t,
                               std::vector<Eigen::Vector3d>* n);

void PoseFromHomographyMatrix(const Eigen::Matrix3d& H,
                              const Eigen::Matrix3d& K1,
                              const Eigen::Matrix3d& K2,
                              const std::vector<Eigen::Vector2d>& points1,
                              const std::vector<Eigen::Vector2d>& points2,
                              Eigen::Matrix3d* R, Eigen::Vector3d* t,
                              Eigen::Vector3d* n,
                              std::vector<Eigen::Vector3d>* points3D);


Eigen::Matrix3d HomographyMatrixFromPose(const Eigen::Matrix3d& K1,
                                         const Eigen::Matrix3d& K2,
                                         const Eigen::Matrix3d& R,
                                         const Eigen::Vector3d& t,
                                         const Eigen::Vector3d& n,
                                         const double d);
}//namespace