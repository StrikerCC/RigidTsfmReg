//
// Created by cheng on 11/8/21.
//

#include "Registration.h"
#include "PointSet.h"
#include "TransformationEstimation.h"
#include <fstream>

status Registration::RegisterWithSequenceAsCorrespondence(const std::vector<std::vector<double>>& src, const std::vector<std::vector<double>>& tgt) {
    if (src.size() != tgt.size()) {
        std::cout << "Registration::RegisterWithSequenceAsCorrespondence: number of source points: " << src.size() << " doesn't match with number of target points: " << tgt.size() << std::endl;
        return status::reg_fail;
    }
    if (src.size() < 3) {
        std::cout << "Registration::RegisterWithSequenceAsCorrespondence: "<< src.size() <<" point pairs is not enough to compute a transformation" << std::endl;
        return status::reg_fail;
    }
    auto transf_est = TransformationEstimation();

    this->transformation_ = transf_est.ComputeTransformation(src, tgt);
    this->src_ = src;
    this->tgt_ = tgt;

    /// log error
//    PointSet src_p = PointSet(src), tgt_p = PointSet(tgt);
    this->error_ = TransformationEstimation::ComputeRMSE(src, tgt);

    return status::reg_success;
}

status Registration::RegisterWithSequenceAsCorrespondence(const std::map<std::string, std::vector<double>> &src,
                                                          const std::map<std::string, std::vector<double>> &tgt) {
    /// build vector from map
    std::vector<std::vector<double>> src_vec, tgt_vec;
    for (const auto & iter_src : src) {  // loop through src
        // check tgt
        auto iter_tgt = tgt.find(iter_src.first);
        if (iter_tgt == tgt.end()) {   // tgt has this key too
            std::cout << "Registration::RegisterWithSequenceAsCorrespondence: cannot find " << iter_src.first << " in tgt";
        } else {
            if (iter_src.second.size() < 3 or iter_tgt->second.size() < 3) {     // point coordinate should be equal or bigger than 3
                std::cout << "Registration::RegisterWithSequenceAsCorrespondence: " << iter_src.first << " has only " << iter_src.second.size() << " coord in src, but " << iter_tgt->second.size() << " in tgt";
            } else {
                src_vec.push_back(std::vector<double> {iter_src.second[0], iter_src.second[1], iter_src.second[2]});
                tgt_vec.push_back(std::vector<double> {iter_tgt->second[0], iter_tgt->second[1], iter_tgt->second[2]});
            }
        }
    }
    return this->RegisterWithSequenceAsCorrespondence(src_vec, tgt_vec);
}

void Registration::RecordLog(std::string log_path) {
    std::ofstream out(log_path);
    nlohmann::json log;
    log["error"] = this->error_;
    log["src"] = this->src_;
    log["tgt"] = this->tgt_;
    std::vector<std::vector<double>> tsfm_v;
    for (int i = 0; i < 4; i++) {
        std::vector<double> row;
        for (int j = 0; j < 4; j++) {
            row.push_back(this->transformation_(i, j));
        }
        tsfm_v.push_back(row);
    }
    log["tsfm"] = tsfm_v;
    out << log << std::endl;
}
