//
// Created by cheng on 11/5/21.
//

#ifndef RIGIDTRANSREG_TRANSFORMATIONESTIMATION_H
#define RIGIDTRANSREG_TRANSFORMATIONESTIMATION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <random>

#include "PointSet.h"

class TransformationEstimation {
public:
    TransformationEstimation() = default;
    ~TransformationEstimation() = default;

    /// compute RMSE between points in source and target, correspondence is as order. Which is distance is first source
    /// point to first target point, second to second, etc.
    /// \param src source points
    /// \param tgt target points
    /// \return RMSE between all point correspondence
    static double ComputeRMSE(const PointSet& src, const PointSet& tgt);
//    std::vector<std::vector<float>> ComputeTransformation(const PointSet& src, const PointSet& tgt);

    /// compute rigid body transformation that mapping points in source frame to target frame, using Umeyama algorithm
    /// \param src source points
    /// \param tgt target points
    /// \return rigid body transformation that mapping points in source frame to target frame, shape is 4x4
    Eigen::Matrix4d ComputeTransformation(const PointSet& src, const PointSet& tgt);
    /// compute rigid body transformation that mapping points in source frame to target frame, ignore outlier, using
    /// RAMSAC algorithm
    /// \param src source points
    /// \param tgt target points
    /// \param max_correspondences_distance distance to filter outliers, points exceed this distance will be outlier
    /// \return rigid body transformation that mapping points in source frame to target frame, shape is 4x4
    Eigen::Matrix4d ComputeTransformationRANSAC(const PointSet& src, const PointSet& tgt, double max_correspondences_distance=0.01);


public:
    bool with_scaling_ = false;
};


#endif //RIGIDTRANSREG_TRANSFORMATIONESTIMATION_H
