//
// Created by cheng on 11/5/21.
//

#include "TransformationEstimation.h"

double TransformationEstimation::ComputeRMSE(const PointSet &src, const PointSet &tgt) {
    assert (src.GetNumOfPoints() == tgt.GetNumOfPoints());
    if (src.empty()) {
        std::cout << "TransformationEstimation::ComputeRMSE: No points to compute rmse" << std::endl;
        return 0.0;
    }
    double err = 0.0;
    for (int i = 0; i < src.GetNumOfPoints(); i++) {
        err += (src.points_[i] - tgt.points_[i]).squaredNorm();
    }
    return std::sqrt(err / (double) src.GetNumOfPoints());
}

//std::vector<std::vector<float>>
Eigen::Matrix4d
TransformationEstimation::ComputeTransformation(const PointSet &src, const PointSet &tgt) {
//    std::vector<std::vector<float>> transformation = {{1.0, 0.0, 0.0, 0.0},
//                                                      {0.0, 1.0, 0.0, 0.0},
//                                                      {0.0, 0.0, 1.0, 0.0},
//                                                      {0.0, 0.0, 0.0, 1.0}};
    assert(src.GetNumOfPoints() == tgt.GetNumOfPoints());

    if (src.empty()) {
        std::cout << "TransformationEstimation::ComputeRMSE: No points to compute transformation" << std::endl;
        return Eigen::Matrix4d::Identity();
    }
    // build point vector for umeyama
    Eigen::MatrixXd source_mat(3, src.GetNumOfPoints());
    Eigen::MatrixXd target_mat(3, src.GetNumOfPoints());
    for (auto i = 0; i < src.GetNumOfPoints(); i++) {
        source_mat.block<3, 1>(0, i) = src.points_[i];
        target_mat.block<3, 1>(0, i) = tgt.points_[i];
    }
    auto transformation_eigen = Eigen::umeyama(source_mat, target_mat, TransformationEstimation::with_scaling_);
    return transformation_eigen;
//    return transformation_eigen.inverse();
    // fill out transformation in stf format
//    for (int i = 0; i < transformation.size(); i++) {
//        for (int j = 0; j < transformation[0].size(); j++) {
//            transformation[i][j] = (float) transformation_eigen(i, j);
//        }
//    }
//    return transformation;
}


int UniformRandInt(const int min, const int max) {
    static thread_local std::mt19937 generator(std::random_device{}());
    std::uniform_int_distribution<int> distribution(min, max);
    return distribution(generator);
}


Eigen::Matrix4d TransformationEstimation::ComputeTransformationRANSAC(const PointSet &src, const PointSet &tgt, double max_correspondences_distance) {
    assert(src.GetNumOfPoints() == tgt.GetNumOfPoints());
    if (src.empty()) {
        std::cout << "TransformationEstimation::ComputeTransformationRANSAC: No points to compute transformation"
                  << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    int max_num_iter = 100;

    int max_num_inlier = 0;
    std::vector<int> best_correspondence;
    Eigen::Matrix4d best_transformation;

#pragma omp parallel
    {
        int max_num_inlier_local = 0;
        std::vector<int> best_correspondence_local;
        Eigen::Matrix4d best_transformation_local;

#pragma omp for nowait
        /// ransac loop
        for (int i = 0; i < max_num_iter; i++) {
            /// pick up 3 correspondence and compute transformation
            int num_inlier = 0;
            std::vector<int> correspondence;
            Eigen::Matrix4d transformation;
            PointSet src_transf = PointSet(src);

            std::vector<Eigen::Vector3d> src_3, tgt_3;
            for (int j = 0; j < 3; j++) {
                int index = UniformRandInt(0, src.GetNumOfPoints());
                src_3.push_back(src.points_[index]);
                tgt_3.push_back(tgt.points_[index]);

                std::cout << index << " ";
            }
            std::cout << std::endl;

            transformation = TransformationEstimation::ComputeTransformation(PointSet(src_3), PointSet(tgt_3));

            /// evaluate this transformation: compute num of inliers
            src_transf.Transform(transformation);
            for (int k = 0; k < src.GetNumOfPoints(); k++) {
                if ((src_transf.points_[k] - tgt.points_[k]).squaredNorm() < max_correspondences_distance) {
                    num_inlier++;
                    correspondence.push_back(k);
                }
            }

            //// keep best one over loop
            if (num_inlier > max_num_inlier_local) {
                max_num_inlier_local = num_inlier;
                best_correspondence_local = correspondence;
                best_transformation_local = transformation;
            }
        } // ransac loop
#pragma omp critical
        {
            /// pick best result from threads, comparison one thread after another
            if (max_num_inlier_local > max_num_inlier) {
                max_num_inlier = max_num_inlier_local;
                best_correspondence = best_correspondence_local;
                best_transformation = best_transformation_local;
            }
        }
    }

        std::cout << "Best inlier ratio " << (float) max_num_inlier / (float) src.GetNumOfPoints() << std::endl;
//    return max_num_inlier < src.GetNumOfPoints() ? best_transformation : TransformationEstimation::ComputeTransformationRANSAC(src, tgt, max_correspondences_distance/2);
        return best_transformation;
}
