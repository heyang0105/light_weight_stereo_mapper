#ifndef SRC_UTIL_GEO_3D_H
#define SRC_UTIL_GEO_3D_H

#include<vector>
#include <Eigen/Core>

#include "util/logging.h"

namespace UTIL{

template <typename DataType>
bool IsPointsInALine(const std::vector< Eigen::Matrix<DataType, 3, 1>>& points);

template <typename DataType>
Eigen::Matrix<DataType, 3, 1> CalcMeanPoint3D(const std::vector< Eigen::Matrix<DataType, 3, 1>>& points);

template <typename DataType>
DataType CalcVariencePoint3D(const std::vector< Eigen::Matrix<DataType, 3, 1>>& points);

template <typename DataType>
Eigen::Matrix<DataType, 3, 3> CalcCovariencePoint3D(
    const std::vector< Eigen::Matrix<DataType, 3, 1>>& points1,
    const std::vector< Eigen::Matrix<DataType, 3, 1>>& points2);

/*
in:
    points1 = estimate
    points2 = groundtruth
out:
    scale: pt2 = pt1 * scale
*/
template <typename DataType>
DataType CalcScaleBetweenEstimateAndGroundtruth(
    const std::vector< Eigen::Matrix<DataType, 3, 1>>& points1,
    const std::vector< Eigen::Matrix<DataType, 3, 1>>& points2);

/*
in:
    points1 = estimate
    points2 = groundtruth
    quat = R_GT_EST
    scale: pt2 = pt1 * scale
out:
    trans = mean_GT - s*R*mean_EST
*/
template <typename DataType>
Eigen::Matrix<DataType, 3, 1> CalcTranslationBetweenEstimateAndGroundtruth(
    const Eigen::Matrix<DataType, 3, 3> rot_mat,
    const DataType& scale,
    const std::vector< Eigen::Matrix<DataType, 3, 1>>& points1,
    const std::vector< Eigen::Matrix<DataType, 3, 1>>& points2);

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

//To test if input 3D points are in a line
template <typename DataType>
bool IsPointsInALine(const std::vector< Eigen::Matrix<DataType, 3, 1>>& points){
    if(points.size() < 3) return false;
    Eigen::Matrix<DataType, Eigen::Dynamic, 3> Q(points.size(), 3);
    for(int i = 0; i < points.size(); i++){
        Q(i, 0) = points[i][0];
        Q(i, 1) = points[i][1];
        Q(i, 2) = points[i][2];
    }

    const Eigen::JacobiSVD<Eigen::Matrix<DataType, Eigen::Dynamic, 3>> svd(
        Q, Eigen::ComputeFullV);

    Eigen::Matrix<DataType, 3, 1> A = svd.singularValues();
    if(A[0] > 50 * A[1]){
        return true;
    }
    return false;
}

// 
template <typename DataType>
Eigen::Matrix<DataType, 3, 1> CalcMeanPoint3D(const std::vector< Eigen::Matrix<DataType, 3, 1>>& points){
    Eigen::Matrix<DataType, 3, 1> res_point(0, 0, 0);
    //size_t count = 0;
    for(size_t i = 0; i < points.size(); i++){
        res_point = res_point + points.at(i);
        //count +=1;
    }

    //res_point = res_point / static_cast<DataType>(count);
    res_point = res_point / static_cast<DataType>(points.size());
    return res_point;
}

template <typename DataType>
DataType CalcVariencePoint3D(const std::vector< Eigen::Matrix<DataType, 3, 1>>& points){
    
    //---1. mean
    Eigen::Matrix<DataType, 3, 1> mean_vec = CalcMeanPoint3D(points);

    //---2 var
    DataType var = 0;
    //size_t count = 0;
    for(size_t i = 0; i < points.size(); i++){
        DataType cur_var = (points.at(i) - mean_vec).norm();
        cur_var = cur_var * cur_var;
        var = var + cur_var;
        //count += 1;
    }

    var = var / static_cast<DataType>(points.size());
    return var;
}


template <typename DataType>
Eigen::Matrix<DataType, 3, 3> CalcCovariencePoint3D(
    const std::vector< Eigen::Matrix<DataType, 3, 1>>& points1,
    const std::vector< Eigen::Matrix<DataType, 3, 1>>& points2){

    CHECK_EQ(points1.size(), points2.size());
    Eigen::Matrix<DataType, 3, 3> cov = Eigen::Matrix<DataType, 3, 3>::Zero();

    //---1. mean
    Eigen::Matrix<DataType, 3, 1> mean1 = CalcMeanPoint3D(points1);
    Eigen::Matrix<DataType, 3, 1> mean2 = CalcMeanPoint3D(points2);

    //---2.cov
    for(size_t i = 0; i < points1.size(); i++){
        Eigen::Matrix<DataType, 3, 3> cur_cov = 
            (points1.at(i) - mean1) *  (points2.at(i) - mean2).transpose();
        cov = cov + cur_cov;
    }
    cov = cov / static_cast<DataType>(points1.size());
    return cov;
}

/*
in:
    points1 = estimate
    points2 = groundtruth
out:
    scale: pt2 = pt1 * scale
*/
template <typename DataType>
DataType CalcScaleBetweenEstimateAndGroundtruth(
    const std::vector< Eigen::Matrix<DataType, 3, 1>>& points1,
    const std::vector< Eigen::Matrix<DataType, 3, 1>>& points2){

    CHECK_EQ(points1.size(), points2.size());

    //---1. calc cov
    Eigen::Matrix<DataType, 3, 3> cov = CalcCovariencePoint3D(points1, points2);
    
    //---2. calc svd -> W
    const Eigen::JacobiSVD<Eigen::Matrix<DataType, 3, 3>> svd(
        cov, Eigen::ComputeFullV | Eigen::ComputeFullU );

    Eigen::Matrix<DataType, 3, 3> W = Eigen::Matrix<DataType, 3, 3>::Identity(3, 3);
    if(svd.matrixV().determinant() * svd.matrixU().determinant() < 0){
        W(2, 2) = -1;
    }

    Eigen::Matrix<DataType, 3, 1> D_vec = svd.singularValues();
    Eigen::Matrix<DataType, 3, 3> D_diag = Eigen::Matrix<DataType, 3, 3>::Identity(3, 3);
    D_diag(0, 0) = D_vec[0];
    D_diag(1, 1) = D_vec[1];
    D_diag(2, 2) = D_vec[2];

    //---3. cal var
    DataType var = CalcVariencePoint3D(points1);
    
    //---4. cal scale
    DataType scale = 1.0 / var * (D_diag * W).trace();
    
    return scale;
}

template <typename DataType>
Eigen::Matrix<DataType, 3, 1> CalcTranslationBetweenEstimateAndGroundtruth(
    const Eigen::Matrix<DataType, 3, 3> rot_mat,
    const DataType& scale,
    const std::vector< Eigen::Matrix<DataType, 3, 1>>& points1,
    const std::vector< Eigen::Matrix<DataType, 3, 1>>& points2){

    CHECK_EQ(points1.size(), points2.size());
    CHECK_GT(scale, 0.0);

    //---1. calc mean
    Eigen::Matrix<DataType, 3, 1> mean1 = CalcMeanPoint3D(points1);
    Eigen::Matrix<DataType, 3, 1> mean2 = CalcMeanPoint3D(points2);

    //---2. calc trans
    Eigen::Matrix<DataType, 3, 1> trans = 
        mean2 - scale * rot_mat * mean1;
    
    return trans;
}


} // namespace
#endif
