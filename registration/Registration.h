//
// Created by cheng on 11/8/21.
//

#ifndef RIGIDTRANSREG_REGISTRATION_H
#define RIGIDTRANSREG_REGISTRATION_H
#include <iostream>
#include <Eigen/Core>
#include <vector>

enum class status {
    reg_incomplete = 0,
    reg_success = 1,
    reg_fail = -1
};

class Registration {
public:
    Registration() = default;
    ~Registration() = default;

    /// estimate a optimal rigid body transformation that mapping points in source frame to target frame
    /// \param src source points
    /// \param tgt target points
    status RegisterWithSequenceAsCorrespondence(const std::vector<std::vector<double>>& src, const std::vector<std::vector<double>>& tgt);

public:
    /// last transformation estimated by function Registration::RegisterWithSequenceAsCorrespondence
    Eigen::Matrix4d transformation_;
};


#endif //RIGIDTRANSREG_REGISTRATION_H
