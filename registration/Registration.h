//
// Created by cheng on 11/8/21.
//

#ifndef RIGIDTRANSREG_REGISTRATION_H
#define RIGIDTRANSREG_REGISTRATION_H
#include <iostream>
//#include <Eigen/Core>
#include <eigen3/Eigen/Core>
#include <vector>
#include <map>
#include <unordered_map>
#include <nlohmann/json.hpp>

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

    /// estimate a optimal rigid body transformation that mapping points in source frame to target frame
    /// \param src source points with names
    /// \param tgt target points with names
    /// \return
    status RegisterWithSequenceAsCorrespondence(const std::map<std::string, std::vector<double>>& src, const std::map<std::string, std::vector<double>>& tgt);

    /// estimate a optimal rigid body transformation that mapping points in source frame to target frame
    /// \param src source points with names
    /// \param tgt target points with names
    /// \return
    status RegisterWithSequenceAsCorrespondence(const std::unordered_map<std::string, std::vector<double>>& src, const std::unordered_map<std::string, std::vector<double>>& tgt);

    void RecordLog(const std::string& log_path, const std::vector<std::vector<double>>& tracking_in_src, const std::vector<std::vector<double>>& tracking_in_tgt);

public:
    /// last transformation estimated by function Registration::RegisterWithSequenceAsCorrespondence
    std::vector<std::vector<double>> src_;
    std::vector<std::vector<double>> tgt_;

//    std::shared_ptr<std::unordered_map<std::string, std::vector<double>>> src_with_name_;
//    std::shared_ptr<std::unordered_map<std::string, std::vector<double>>> tgt_with_name_;

    std::unordered_map<std::string, std::vector<double>> src_with_name_;
    std::unordered_map<std::string, std::vector<double>> tgt_with_name_;

    Eigen::Matrix4d transformation_;
    Eigen::Matrix4d transformation_inv_;
    double error_ = INFINITY;
};


#endif //RIGIDTRANSREG_REGISTRATION_H
