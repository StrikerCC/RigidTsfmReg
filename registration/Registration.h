//
// Created by cheng on 11/8/21.
//

#ifndef RIGIDTRANSREG_REGISTRATION_H
#define RIGIDTRANSREG_REGISTRATION_H
#include <iostream>
#include <eigen3/Eigen/Core>
#include <vector>

class Registration {
public:
    Registration() = default;
    ~Registration() = default;

    /// estimate a optimal rigid body transformation that mapping points in source frame to target frame
    /// \param src source points
    /// \param tgt target points
    void RegisterWithSequenceAsCorrespondence(const std::vector<std::vector<double>>& src, const std::vector<std::vector<double>>& tgt);

public:
    /// last transformation estimated by function Registration::RegisterWithSequenceAsCorrespondence
    Eigen::Matrix4d transformation_;
};


#endif //RIGIDTRANSREG_REGISTRATION_H
