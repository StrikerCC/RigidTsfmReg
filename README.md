# RigidTsfmReg

This project contain source code to estimate rigid homogeneous transformation matrix between two point set.

## Prerequisite
### install Cmake
```bash
$ sudo apt-get install cmake cmake-curses-gui
```

### install Eigen3
####On Windows

####On Linux
```bash
## install Eigen3 library from apt
$ sudo apt-get install libeigen3-dev

## check Eigen3 installation 
$ dpkg -L libeigen3-dev
```
## Get Started 
make sure all library included well, to estimate transformation between two point set, in your main script
```C++
#include "registration/Registration.h"
#include "registration/TransformationEstimation.h"
#include "registration/PointSet.h"

int main() {
    
    /// init registration object
    Registration reg = Registration();
    
    /// source and target input points 
    std::vector<std::vector<double>> src_points = {{0.0, 0.0, 0.0,}, {2.0, 2.0, 2.0,}};
    std::vector<std::vector<double>> tgt_points = {{1.0, 1.0, 1.0,}, {3.0, 3.0, 3.0,}};
    
    /// estimate transformation
    reg.RegisterWithSequenceAsCorrespondence(src_points, tgt_points);

    /// do transformation
    auto src_points_2_tgt_frame = PointSet::Transform(src_points, reg.transformation_); // transform point set
    auto src_point_2_tgt_frame = PointSet::Transform(src_points[0], reg.transformation_ );// transform a point
    return 0;
}
```

Please main.cpp for more detail

