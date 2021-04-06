// 2021-03-27

#ifndef COMMON_VIEW_GEO_H_
#define COMMON_VIEW_GEO_H_

#include <vector>

#include <Eigen/Core>

#include "view_geo.h"

namespace ViewGeo{

// --- BASIC MAT PROCESS
Eigen::Matrix3d CrossProductMatrix(const Eigen::Vector3d& vector);

// ---  QUAT RELATED
/* We warp the quaternion, only using the Vector4d */
Eigen::Vector4d ComposeIdentityQuaternion();

/* */
Eigen::Vector4d NormalizeQuaternion(const Eigen::Vector4d& qvec);

/* */
Eigen::Vector4d InvertQuaternion(const Eigen::Vector4d& qvec);

/* */
Eigen::Vector4d ConcatenateQuaternions(const Eigen::Vector4d& qvec1,
                                       const Eigen::Vector4d& qvec2);

/* */
Eigen::Vector3d QuaternionRotatePoint(const Eigen::Vector4d& qvec,
                                      const Eigen::Vector3d& point);

/* */
Eigen::Vector4d AverageQuaternions(const std::vector<Eigen::Vector4d>& qvecs,
                                   const std::vector<double>& weights);

// --- EXTRACT 
Eigen::Matrix3d RotationFromUnitVectors(const Eigen::Vector3d& vec1,
                                        const Eigen::Vector3d& vec2);

/* */
Eigen::Vector3d ProjectionCenterFromMatrix(
    const Eigen::Matrix3x4d& proj_matrix);    

/* */
Eigen::Vector3d ProjectionCenterFromPose(const Eigen::Vector4d& qvec,
                                         const Eigen::Vector3d& tvec);

/* */
void InterpolatePose(const Eigen::Vector4d& qvec1, const Eigen::Vector3d& tvec1,
                     const Eigen::Vector4d& qvec2, const Eigen::Vector3d& tvec2,
                     const double t, Eigen::Vector4d* qveci,
                     Eigen::Vector3d* tveci);

/* */
Eigen::Vector3d CalculateBaseline(const Eigen::Vector4d& qvec1,
                                  const Eigen::Vector3d& tvec1,
                                  const Eigen::Vector4d& qvec2,
                                  const Eigen::Vector3d& tvec2);

/* */
bool CheckCheirality(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                     const std::vector<Eigen::Vector2d>& points1,
                     const std::vector<Eigen::Vector2d>& points2,
                     std::vector<Eigen::Vector3d>* points3D);

/* */
Eigen::Matrix3d ComputeClosestRotationMatrix(const Eigen::Matrix3d& matrix);

/* */
bool DecomposeProjectionMatrix(const Eigen::Matrix3x4d& proj_matrix,
                               Eigen::Matrix3d* K, Eigen::Matrix3d* R,
                               Eigen::Vector3d* T);

// --- TRANSFORM: EULER <-> MAT
// `R = Rx * Ry * Rz` 
void RotationMatrixToEulerAngles(const Eigen::Matrix3d& R, double* rx,
                                 double* ry, double* rz);

Eigen::Matrix3d EulerAnglesToRotationMatrix(const double rx, const double ry,
                                            const double rz);

// ---TRANSFORM:  QUAT <-> MAT
Eigen::Vector4d RotationMatrixToQuaternion(const Eigen::Matrix3d& rot_mat);

Eigen::Matrix3d QuaternionToRotationMatrix(const Eigen::Vector4d& qvec);

}//namespace

#endif