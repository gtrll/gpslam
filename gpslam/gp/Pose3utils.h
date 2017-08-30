/**
 *  @file  Pose3utils.h
 *  @brief Pose3 GP utils, mainly jacobians of expmap/logmap
 *  @author Xinyan Yan, Jing Dong
 **/

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

#include <boost/function.hpp>


namespace gpslam {

// fixed size matrix for Dim-6 operation
typedef Eigen::Matrix<double, 12, 1> Vector_12;
typedef Eigen::Matrix<double, 12, 12> Matrix_12;
typedef Eigen::Matrix<double, 6, 12> Matrix_6_12;
typedef Eigen::Matrix<double, 12, 6> Matrix_12_6;
typedef Eigen::Matrix<double, 3, 12> Matrix_3_12;
typedef Eigen::Matrix<double, 12, 3> Matrix_12_3;


/// get body-centric/body-frame velocity from two poses and delta_t
gtsam::Vector6 getBodyCentricVs(const gtsam::Pose3& pose1, const gtsam::Pose3& pose2, double delta_t);
gtsam::Vector6 getBodyCentricVb(const gtsam::Pose3& pose1, const gtsam::Pose3& pose2, double delta_t);

/// convert 6d body-frame velocity into 3+3 world-frame line and angular velocity
/// with optional jacobians
void convertVbtoVW(const gtsam::Vector6& v6, const gtsam::Pose3& pose,
    gtsam::Vector3& v, gtsam::Vector3& w,
    gtsam::OptionalJacobian<3, 6> Hv = boost::none, gtsam::OptionalJacobian<3, 6> Hw = boost::none);

gtsam::Vector6 convertVWtoVb(const gtsam::Vector3& v, const gtsam::Vector3& w,
    const gtsam::Pose3& pose, gtsam::OptionalJacobian<6, 3> Hv = boost::none,
    gtsam::OptionalJacobian<6, 3> Hw = boost::none,
    gtsam::OptionalJacobian<6, 6> Hpose = boost::none);


/// left Jacobian for Pose3 Expmap
gtsam::Matrix6 leftJacobianPose3(const gtsam::Vector6& xi);
gtsam::Matrix6 leftJacobianPose3inv(const gtsam::Vector6& xi);
gtsam::Matrix3 leftJacobianPose3Q(const gtsam::Vector6& xi);

/// right Jacobian for Pose3: jacobian of expmap/logmap
gtsam::Matrix6 rightJacobianPose3(const gtsam::Vector6& xi);
gtsam::Matrix6 rightJacobianPose3inv(const gtsam::Vector6& xi);
gtsam::Matrix3 rightJacobianPose3Q(const gtsam::Vector6& xi);

/// numerical diff of jacobian matrix methods
/// output d(A(xi)*x)/dxi jacobian matrix
gtsam::Matrix6 jacobianMethodNumercialDiff(boost::function<gtsam::Matrix6(
    const gtsam::Vector6&)> func, const gtsam::Vector6& xi,
    const gtsam::Vector6& x, double dxi = 1e-6);

/// left Jacobian for Rot3 Expmap
gtsam::Matrix3 leftJacobianRot3(const gtsam::Vector3& omega);
gtsam::Matrix3 leftJacobianRot3inv(const gtsam::Vector3& omega);

/// right Jacobian for Rot3: jacobian of expmap/logmap
gtsam::Matrix3 rightJacobianRot3(const gtsam::Vector3& omega);
gtsam::Matrix3 rightJacobianRot3inv(const gtsam::Vector3& omega);

} // namespace gtsam


