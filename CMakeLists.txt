cmake_minimum_required(VERSION 3.20)
project(RigidTransReg)

set(CMAKE_CXX_STANDARD 14)

####################################### eigen #######################################

find_package(Eigen3 REQUIRED)

include_directories("./registration")

add_executable(RigidTransReg main.cpp registration/TransformationEstimation.cpp registration/TransformationEstimation.h registration/PointSet.cpp registration/PointSet.h registration/Registration.cpp registration/Registration.h)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)