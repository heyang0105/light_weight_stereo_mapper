

#include"common.h"

namespace ViewGeo{

// --- BASIC MAT PROCESS
Eigen::Matrix3d CrossProductMatrix(const Eigen::Vector3d& vector){
    Eigen::Matrix3d matrix;
    matrix << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1),
        vector(0), 0;
    return matrix;
}

// ---  QUAT RELATED
Eigen::Vector4d ComposeIdentityQuaternion(){
    return Eigen::Vector4d(1, 0, 0, 0);
}

Eigen::Vector4d NormalizeQuaternion(const Eigen::Vector4d& qvec){
    const double norm = qvec.norm();
    if (norm == 0) {
    // We do not just use (1, 0, 0, 0) because that is a constant and when used
    // for automatic differentiation that would lead to a zero derivative.
        return Eigen::Vector4d(1.0, qvec(1), qvec(2), qvec(3));
    } else {
        return qvec / norm;
    }
}

Eigen::Vector4d InvertQuaternion(const Eigen::Vector4d& qvec){
    return Eigen::Vector4d(qvec(0), -qvec(1), -qvec(2), -qvec(3));
}

Eigen::Vector4d ConcatenateQuaternions(const Eigen::Vector4d& qvec1,
                                       const Eigen::Vector4d& qvec2){
    const Eigen::Vector4d normalized_qvec1 = NormalizeQuaternion(qvec1);
    const Eigen::Vector4d normalized_qvec2 = NormalizeQuaternion(qvec2);
    const Eigen::Quaterniond quat1(normalized_qvec1(0), normalized_qvec1(1),
                                 normalized_qvec1(2), normalized_qvec1(3));
    const Eigen::Quaterniond quat2(normalized_qvec2(0), normalized_qvec2(1),
                                 normalized_qvec2(2), normalized_qvec2(3));
    const Eigen::Quaterniond cat_quat = quat1 * quat2;
    return NormalizeQuaternion(
        Eigen::Vector4d(cat_quat.w(), cat_quat.x(), cat_quat.y(), cat_quat.z()));
}

Eigen::Vector3d QuaternionRotatePoint(const Eigen::Vector4d& qvec,
                                      const Eigen::Vector3d& point){

}

Eigen::Vector4d AverageQuaternions(const std::vector<Eigen::Vector4d>& qvecs,
                                   const std::vector<double>& weights);

// --- EXTRACT 
Eigen::Matrix3d RotationFromUnitVectors(const Eigen::Vector3d& vec1,
                                        const Eigen::Vector3d& vec2);

Eigen::Vector3d ProjectionCenterFromMatrix(
    const Eigen::Matrix3x4d& proj_matrix);    

Eigen::Vector3d ProjectionCenterFromPose(const Eigen::Vector4d& qvec,
                                         const Eigen::Vector3d& tvec);

void InterpolatePose(const Eigen::Vector4d& qvec1, const Eigen::Vector3d& tvec1,
                     const Eigen::Vector4d& qvec2, const Eigen::Vector3d& tvec2,
                     const double t, Eigen::Vector4d* qveci,
                     Eigen::Vector3d* tveci);

Eigen::Vector3d CalculateBaseline(const Eigen::Vector4d& qvec1,
                                  const Eigen::Vector3d& tvec1,
                                  const Eigen::Vector4d& qvec2,
                                  const Eigen::Vector3d& tvec2);

bool CheckCheirality(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                     const std::vector<Eigen::Vector2d>& points1,
                     const std::vector<Eigen::Vector2d>& points2,
                     std::vector<Eigen::Vector3d>* points3D);

Eigen::Matrix3d ComputeClosestRotationMatrix(const Eigen::Matrix3d& matrix);

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
}// namespace