//
// Created by cheng on 11/8/21.
//

#include "Registration.h"
#include "PointSet.h"
#include "TransformationEstimation.h"

void Registration::RegisterWithSequenceAsCorrespondence(const std::vector<std::vector<double>>& src, const std::vector<std::vector<double>>& tgt) {
    assert(src.size() == tgt.size());
    if (src.empty()) {
        std::cout << "Registration::RegisterWithSequenceAsCorrespondence: No points to compute transformation" << std::endl;
    }
    auto transf_est = TransformationEstimation();
    Registration::transformation_ = transf_est.ComputeTransformation(src, tgt);
}
