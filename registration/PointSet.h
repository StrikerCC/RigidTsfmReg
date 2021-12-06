//
// Created by cheng on 11/5/21.
//

#ifndef RIGIDTRANSREG_POINTSET_H
#define RIGIDTRANSREG_POINTSET_H

//#include <Eigen/Core>
#include <eigen3/Eigen/Core>
#include <vector>
#include <iostream>

class PointSet {
public:
    /// default constructor
    PointSet() = default;
    /// parameterized constructor
    /// \param pointset another point set
    PointSet(const PointSet& pointset) : points_(pointset.points_) {};
    /// parameterized constructor
    /// \param points a list of 3d point in format of eigen vector3d
    PointSet(const std::vector<Eigen::Vector3d>& points) : points_(points) {};
    /// parameterized constructor
    /// \param points a list of list of coordinate in double, expect shape (N, 3), sequence x, y, z. Second vector
    /// length shorter than 3 will cause error, longer than 3, elements after z(third element) will be ignored.
    PointSet(const std::vector<std::vector<double>>& points);
    ~PointSet() = default;

    /// getter
    int GetNumOfPoints() const;
    /// return true if pointset is empty
    /// \return
    bool empty() const;
    /// return a copy of all points in pointset as a list of list of coordinate in double
    /// \return a list of list of coordinate in double, shape is (N, 3)
    std::vector<std::vector<double>> GetPoints() const;

    /// setter
    void Transform(Eigen::Matrix4d transformation);
    void Transform(std::vector<std::vector<double>> transformation);

    /// static func

    ///
    /// \param point_1
    /// \param point_2
    /// \return
    static double NormalizedDotProduct(const std::vector<double> &point_1, const std::vector<double> &point_2);

    /// Transform input points using transformation
    /// \param points points to be transform, expect shape (N, 3)
    /// \param transformation 4x4 transformation matrix
    /// \return transformed points, shape (N, 3)
    static std::vector<std::vector<double>> Transform(const std::vector<std::vector<double>> &points, const Eigen::Matrix4d &transformation);

    /// Transform input point using transformation
    /// \param point point to be transform, expect shape (3)
    /// \param transformation 4x4 transformation matrix
    /// \return transformed points, shape (3)
    static std::vector<double> Transform(const std::vector<double> &point, const Eigen::Matrix4d &transformation);

public:
    /// Points coordinates.
    std::vector<Eigen::Vector3d> points_;


};


#endif //RIGIDTRANSREG_POINTSET_H
