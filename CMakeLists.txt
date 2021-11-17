cmake_minimum_required(VERSION 3.20)
project(RigidTransReg)

set(CMAKE_CXX_STANDARD 14)
MESSAGE(STATUS "operation system is ${CMAKE_SYSTEM}")

####################################### eigen #######################################
IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
    MESSAGE(STATUS "other platform: ${CMAKE_SYSTEM_NAME}")
    find_package(Eigen3 REQUIRED)

ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Windows")
    MESSAGE(STATUS "other platform: ${CMAKE_SYSTEM_NAME}")
    set(Eigen3_DIR "./ThirdParty/Eigen/install_release/share/eigen3/cmake")
    find_package(Eigen3 REQUIRED)
    include_directories(${EIGEN3_INCLUDE_DIR})
ELSE ()
    MESSAGE(STATUS "other platform: ${CMAKE_SYSTEM_NAME}")
ENDIF()


####################################### open3d #######################################
set(Open3D_DIR /home/cheng/sources/open3d13/install_release_cxx11AbiOn/lib/cmake/Open3D)
find_package(Open3D)


include_directories("./registration")

add_executable(RigidTransReg main.cpp registration/TransformationEstimation.cpp registration/TransformationEstimation.h registration/PointSet.cpp registration/PointSet.h registration/Registration.cpp registration/Registration.h)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)