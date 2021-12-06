//
// Created by cheng on 11/5/21.
//

#include "PointSet.h"

PointSet::PointSet(const std::vector<std::vector<double>> &points) {
//    PointSet::points_ = std::vector<Eigen::Vector3d>();
    for (auto point : points) {
        if (point.size() >= 3) {
            PointSet::points_.emplace_back(point[0], point[1], point[2]);
        } else {    // ignore this point, and print it for debug
            std::cout << "PointSet::PointSet: (";
            for (auto coord : point) {
                std::cout << coord;
            }
            std::cout << ") is no enough coordinate to init a point" << std::endl;
        }
    }
}

int PointSet::GetNumOfPoints() const {
    return PointSet::points_.size();
}

std::vector<std::vector<double>> PointSet::GetPoints() const {
    std::vector<std::vector<double>> points;
    for (auto point_eigen : PointSet::points_) {
        std::vector<double> point;
        for (int i = 0; i < point_eigen.size(); i++) {
            point.push_back((double) point_eigen(i, 0));
        }
        points.push_back(point);
    }
    return points;
}

bool PointSet::empty() const {
    return PointSet::GetNumOfPoints() == 0;
}

void PointSet::Transform(Eigen::Matrix4d transformation) {
    if (this->GetNumOfPoints() == 0) {
        return;
    }
    if ((transformation * transformation.transpose()).isIdentity()) {
        std::cout << "PointSet::Transform: " << transformation << " not seems to be a rigid transformation matrix" << std::endl;
    }
#pragma omp parallel
    {
        for (auto &point: PointSet::points_) {
            Eigen::Vector4d point_ = transformation * Eigen::Vector4d(point(0), point(1), point(2), 1.0);
            point = point_.head<3>();
        }
    }
}

void PointSet::Transform(std::vector<std::vector<double>> transformation) {
    auto transformation_eigen = Eigen::Matrix4d();
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            transformation_eigen(i, j) = transformation[i][j];
        }
    }
    this->Transform(transformation_eigen);
}

double PointSet::NormalizedDotProduct(const std::vector<double>& point_1, const std::vector<double>& point_2) {
    Eigen::VectorXd point_1_eigen(std::max(point_1.size(), point_2.size())), point_2_eigen(std::max(point_1.size(), point_2.size()));
    /// copy to eigen vector
    for (int i = 0; i < std::max(point_1.size(), point_2.size()); i++) {
        point_1_eigen(i) = point_1[i];
        point_2_eigen(i) = point_2[i];
    }

    /// normalize
    point_1_eigen = point_1_eigen.normalized();
    point_2_eigen = point_2_eigen.normalized();

    /// compute dot product
    return point_1_eigen.dot(point_2_eigen);
}


std::vector<std::vector<double>> PointSet::Transform(const std::vector<std::vector<double>> &points, const Eigen::Matrix4d &transformation) {
    PointSet point_set = PointSet(points);
    point_set.Transform(transformation);
    return point_set.GetPoints();
}


std::vector<double> PointSet::Transform(const std::vector<double> &point, const Eigen::Matrix4d &transformation) {
    if (point.size() < 3) {
        return point;
    }
    return PointSet::Transform(std::vector<std::vector<double>> {point}, transformation)[0];
}


/*
 *
 * Hello, World!
  0.00451317     0.730193   0.00578636    -0.843667
    0.536868     0.999982   -0.0337547    -0.536856
   -0.843654  0.000686538   0.00368572     0.471792
2.37646e-321 1.63042e-322 1.63042e-322 1.63042e-322

 0.211193  0.919001  0.332918 -0.894655
 -0.42082  0.392907  -0.81764 -0.380708
-0.882218 0.0325817  0.469713  0.430603
        0         0         0         1
0.0021568
 */