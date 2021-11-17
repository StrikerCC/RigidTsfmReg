#include <iostream>
#include <fstream>
#include "registration/Registration.h"
#include "registration/TransformationEstimation.h"
#include "registration/PointSet.h"
#include <chrono>

//
//std::vector<std::vector<double>> Txt2Vec2D(const std::string &file_path) {
//    std::vector<std::vector<double>> xyzs;
//    /// read txt file line by line
//    std::ifstream f;
//    f.open(file_path);
//    std::string line;
//    double x, y, z;
//
//    int i = 0;
//    while (!f.eof()) {
//        f >> x >> y >> z;
//        xyzs.push_back(std::vector<double>({x, y, z}));
//    }
//    ///
//    return xyzs;
//}
//
//int main_() {
//    std::cout << "Hello, World!" << std::endl;
//    std::vector<std::vector<double>> points = Txt2Vec2D("./data/3D_model.txt");
////    std::vector<std::vector<double>> pose = Txt2Vec2D("./data/pose_00000000.txt");
//    std::vector<Eigen::Matrix4d> transfs;
//    {
//        Eigen::Matrix4d transf = Eigen::Matrix4d::Identity();
//        transf <<
//               0.00451317, 0.536868, -0.843654, 0.730193,
//                0.999982, 0.000686538, 0.00578636, -0.0337547,
//                0.00368572, -0.843667, -0.536856, 0.471792,
//                0, 0, 0, 1;
//        transfs.emplace_back(Eigen::Matrix4d(transf));
//
//        transf <<
//               0.296139, 0.516011, -0.803766, 0.677304, 0.955138, -0.156268, 0.251589, -0.244826, 0.00421977, -0.84221, -0.539136, 0.473783, 0, 0, 0, 1;
//        transfs.emplace_back(Eigen::Matrix4d(transf));
//
//        transf <<
//               0.605034, 0.433097, -0.668107, 0.540048, 0.796197, -0.32561, 0.509956, -0.458985, 0.00331693, -0.840482, -0.541834, 0.476038, 0, 0, 0, 1;
//        transfs.emplace_back(Eigen::Matrix4d(transf));
//
//        transf <<
//               0.817534, 0.31885, -0.479567, 0.363181, 0.575841, -0.441911, 0.687846, -0.599921, 0.00739278, -0.838488, -0.54488, 0.478367, 0, 0, 0, 1;
//        transfs.emplace_back(Eigen::Matrix4d(transf));
//    }
//
//    for (const auto& transf: transfs) {
//        PointSet src = PointSet(points);
//        PointSet tgt = PointSet(points);
//        tgt.Transform(transf);
//
//        Registration reg = Registration();
//        reg.RegisterWithSequenceAsCorrespondence(src.GetPoints(), tgt.GetPoints());
////        auto tf_src_2_tgt = tf_est.ComputeTransformationRANSAC(src, tgt);
//
//
//
//        int i_who = 0;
//        std::cout << "{";
//        for (const auto& point : src.GetPoints()) {
//            if (i_who > 20) break;
//            std::cout << "{";
//            for (auto coord : point) {
//                std::cout << coord << ", ";
//            }
//            std::cout << "},";
//            std::cout << std::endl;
//            i_who++;
//        }
//        std::cout << "}" << std::endl;
//        i_who = 0;
//
//        std::cout << "{";
//        for (const auto& point : tgt.GetPoints()) {
//            if (i_who > 20) break;
//            std::cout << "{";
//            for (auto coord : point) {
//                std::cout << coord << ", ";
//            }
//            std::cout << "},";
//            std::cout << std::endl;
//            i_who++;
//        }
//        std::cout << "}" << std::endl;
//        std::cout << std::endl;
//
//
//
//
//        src.Transform(reg.transformation_);
//
//        /// compute error
//        std::cout << "Ground truth: \n" << transf << std::endl;
//        std::cout << std::endl;
//        std::cout << "computed transformation: \n" << reg.transformation_ << std::endl;
//        std::cout << std::endl;
//        std::cout << "RMSE: " << TransformationEstimation::ComputeRMSE(src, tgt);
//        std::cout << std::endl;
//
//    }
//    return 0;
//}

int main() {
    Registration reg = Registration();

    std::vector<double> query_point = {0.0595164, 0.0533297, 0.0857159, };
//
//    std::vector<std::vector<double>> src_points = {{0.0595164, 0.0533297, 0.0857159, },
//                                                   {0.0596195, 0.0500485, 0.0889023, },
//                                                   {0.0597568, 0.0515476, 0.0861541, },
//                                                   {0.0599648, 0.0502481, 0.0876689, },
//                                                   {0.0598265, 0.0499625, 0.0860588, },
//                                                   {0.0596821, 0.0456999, 0.092095, },
//                                                   {0.0595453, 0.0439616, 0.0932945, },
//                                                   {0.0601673, 0.0442375, 0.0924098, }};
//                                                   {0.0626142, 0.0467576, 0.0860099, },
//                                                   {0.0624336, 0.0435443, 0.0888359, },
//                                                   {0.0646273, 0.0439355, 0.0879918, },
//                                                   {0.0642869, 0.0439414, 0.085924, },
//                                                   {0.062475, 0.045278, 0.0873131, },
//                                                   {0.0628682, 0.0457298, 0.0863317, },
//                                                   {0.0627032, 0.0441376, 0.0876546, },
//                                                   {0.0634495, 0.0444015, 0.0864224, },
//                                                   {0.0596415, 0.0487915, 0.0904868, },
//                                                   {0.0602319, 0.0485623, 0.0892628, },
//                                                   {0.0594974, 0.0467962, 0.0906146, },
//                                                   {0.0601746, 0.0471376, 0.0892356, },
//                                                   {0.0611901, 0.0484408, 0.0861569, },};
//
//    std::vector<std::vector<double>> tgt_points = {{0.387735, -0.530257, 0.387386, },
//                                                   {0.385245, -0.526556, 0.388402, },
//                                                   {0.387153, -0.529029, 0.388643, },
//                                                   {0.386183, -0.527293, 0.388909, },
//                                                   {0.386751, -0.528354, 0.390025, },
//                                                   {0.382379, -0.522402, 0.390309, },
//                                                   {0.381137, -0.520887, 0.391112, },
//                                                   {0.382158, -0.52126, 0.391367, }};
//                                                   {0.388031, -0.525366, 0.392759, },
//                                                   {0.385504, -0.522106, 0.393912, },
//                                                   {0.387827, -0.521597, 0.39406, },
//                                                   {0.388542, -0.523218, 0.39518, },
//                                                   {0.386821, -0.523896, 0.393289, },
//                                                   {0.387757, -0.524544, 0.393448, },
//                                                   {0.38648, -0.523026, 0.39406, },
//                                                   {0.387765, -0.52356, 0.394516, },
//                                                   {0.384103, -0.524898, 0.388592, },
//                                                   {0.385099, -0.525298, 0.389456, },
//                                                   {0.383287, -0.524011, 0.390195, },
//                                                   {0.384611, -0.52472, 0.390665, },
//                                                   {0.387333, -0.526829, 0.391257, },};
    std::vector<std::vector<double>> src_points = {{-77.8473,-153.208,-65.3903},
                                                   {79.6214,-138.314,-74.5972},
                                                   {-63.5132,-153.964,-91.3907},
                                                   {53.3655,-147.75,-94.4482},
                                                   {-79.049,-153.746,-111.055},
                                                   {76.4789,-142.094,-119.898},
                                                   {7.89999,-250.869,-130.034},
                                                   {49.0412,-225.777,-97.4579},
                                                   {25.6939,-233.243,-99.9733},
                                                   {-38.4461,-236.155,-90.4999},
                                                   {-10.7764,-236.009,-97.2332},
                                                   {26.8799,-214.037,-158.93},
                                                   {-21.6415,-220.581,-155.64},
                                                   {21.2662,-227.761,-137.733},
                                                   {-12.018,-231.912,-134.536}};

    std::vector<std::vector<double>> tgt_points = {{220.45,120.824,160.298},
                                                   {55.1835,124.231,158.073},
                                                   {210.383,128.495,132.645},
                                                   {62.9985,129.299,132.17},
                                                   {212.443,132.041,107.708},
                                                   {55.8734,135.427,109.217},
                                                   {134.659,239.56,122.295},
                                                   {89.975,210.866,152.33},
                                                   {113.841,218.907,150.194},
                                                   {180.126,213.697,153.049},
                                                   {154.419,219.463,150.789},
                                                   {110.566,214.806,88.8482},
                                                   {155.522,219.27,90.1044},
                                                   {116.985,220.823,111.095},
                                                   {153.563,222.441,113.083}};

    /// estimate transformation
    auto start = std::chrono::system_clock::now();

    auto result = reg.RegisterWithSequenceAsCorrespondence(src_points, tgt_points);

    auto end = std::chrono::system_clock::now();
    auto cost = std::chrono::duration<double, std::milli>(end - start).count();

    std::cout << "Registration status " << (int) result << std::endl;
    std::cout << "take " << cost << " millisecond" << std::endl;

    /// do transformation
    auto src_points_2_tgt_frame = PointSet::Transform(src_points, reg.transformation_); // transform point set
    auto src_point_2_tgt_frame = PointSet::Transform(query_point, reg.transformation_ );// transform a point

    /// log
    reg.RecordLog("./log.json");

    /// error
//    std::cout << "Point Set Transformation: " << std::endl;
//    std::cout << "Transformation estimated: \n"  << reg.transformation_ << std::endl;
//    std::cout << "RMSE after transformation applied: " << TransformationEstimation::ComputeRMSE(src_points_2_tgt_frame, tgt_points) << std::endl;
//    std::cout << std::endl;
//
//    std::cout << "A Point Transformation: " << std::endl;
//    std::cout << "Transformation estimated: \n"  << reg.transformation_ << std::endl;
//    std::cout << "Point Coord before transformation: " << std::endl;
//    for (auto coord : src_points[0]) {
//        std::cout << coord << " ";
//    }
//    std::cout << std::endl;
//    std::cout << "after transformation" << std::endl;
//    for (auto coord : src_point_2_tgt_frame) {
//        std::cout << coord << " ";
//    }
//    std::cout << std::endl;
//    std::cout << "ground truth" << std::endl;
//    for (auto coord : tgt_points[0]) {
//        std::cout << coord << " ";
//    }
//    std::cout << std::endl;

    return 0;
}

