//
// Created by cheng on 11/8/21.
//

#include "Registration.h"
#include "PointSet.h"
#include "TransformationEstimation.h"

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
    Registration::transformation_ = transf_est.ComputeTransformation(src, tgt);
    return status::reg_success;
}
