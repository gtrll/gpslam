// GP-SLAM wrapper

// delearation
class gtsam::Vector6;
class gtsam::Vector3;
class gtsam::Rot2;
class gtsam::Point2;
class gtsam::Pose2;
class gtsam::Rot3;
class gtsam::Point3;
class gtsam::Pose3;
class gtsam::Unit3;
class gtsam::Cal3_S2;

class gtsam::GaussianFactorGraph;
class gtsam::Values;

virtual class gtsam::noiseModel::Base;
virtual class gtsam::NonlinearFactor;
virtual class gtsam::NonlinearFactorGraph;
virtual class gtsam::NoiseModelFactor;


namespace gpslam {

////////////////////////////////////////////////////////////////////////////////
// Lie classes
////////////////////////////////////////////////////////////////////////////////

// prior factor
#include <gpslam/gp/GaussianProcessPriorPose3.h>
virtual class GaussianProcessPriorPose3 : gtsam::NoiseModelFactor {
  GaussianProcessPriorPose3(size_t key1, size_t key2, size_t key3, size_t key4,
      double delta, const gtsam::noiseModel::Base* Qc_model);
};

#include <gpslam/gp/GaussianProcessPriorPose3VW.h>
virtual class GaussianProcessPriorPose3VW : gtsam::NoiseModelFactor {
  GaussianProcessPriorPose3VW(size_t key1, size_t key2, size_t key3, size_t key4,
      size_t key5, size_t key6, double delta, const gtsam::noiseModel::Base* Qc_model);
};

#include <gpslam/gp/GaussianProcessPriorPose2.h>
virtual class GaussianProcessPriorPose2 : gtsam::NoiseModelFactor {
  GaussianProcessPriorPose2(size_t key1, size_t key2, size_t key3, size_t key4,
      double delta, const gtsam::noiseModel::Base* Qc_model);
};

#include <gpslam/gp/GaussianProcessPriorRot3.h>
virtual class GaussianProcessPriorRot3 : gtsam::NoiseModelFactor {
  GaussianProcessPriorRot3(size_t key1, size_t key2, size_t key3, size_t key4,
      double delta, const gtsam::noiseModel::Base* Qc_model);
};

// util class for all interpolated measurements
#include <gpslam/gp/GaussianProcessInterpolatorPose3.h>
class GaussianProcessInterpolatorPose3 {
  GaussianProcessInterpolatorPose3(const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
  gtsam::Pose3 interpolatePose(const gtsam::Pose3& pose1, Vector vel1,
        const gtsam::Pose3& pose2, Vector vel2) const;
};

#include <gpslam/gp/GaussianProcessInterpolatorPose3VW.h>
class GaussianProcessInterpolatorPose3VW {
  GaussianProcessInterpolatorPose3VW(const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
  gtsam::Pose3 interpolatePose(const gtsam::Pose3& pose1, Vector vel1, Vector w1,
        const gtsam::Pose3& pose2, Vector vel2, Vector w2) const;
};

#include <gpslam/gp/GaussianProcessInterpolatorPose2.h>
class GaussianProcessInterpolatorPose2 {
  GaussianProcessInterpolatorPose2(const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
  gtsam::Pose2 interpolatePose(const gtsam::Pose2& pose1, Vector vel1,
        const gtsam::Pose2& pose2, Vector vel2) const;
};

#include <gpslam/gp/GaussianProcessInterpolatorRot3.h>
class GaussianProcessInterpolatorRot3 {
  GaussianProcessInterpolatorRot3(const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
  gtsam::Rot3 interpolatePose(const gtsam::Rot3& pose1, Vector vel1,
        const gtsam::Rot3& pose2, Vector vel2) const;
};



// projection factor
#include <gpslam/slam/GPInterpolatedProjectionFactorPose3.h>
template <CALIBRATION>
virtual class GPInterpolatedProjectionFactorPose3 : gtsam::NoiseModelFactor {
  GPInterpolatedProjectionFactorPose3(const gtsam::Point2& measured,
      const gtsam::noiseModel::Base* cam_model, const gtsam::noiseModel::Base* Qc_model,
      size_t pose1Key, size_t vel1Key, size_t pose2Key, size_t vel2Key,
      size_t pointKey, double delta_t, double tau, const CALIBRATION* k);
  GPInterpolatedProjectionFactorPose3(const gtsam::Point2& measured,
      const gtsam::noiseModel::Base* cam_model, const gtsam::noiseModel::Base* Qc_model,
      size_t pose1Key, size_t vel1Key, size_t pose2Key, size_t vel2Key,
      size_t pointKey, double delta_t, double tau, const CALIBRATION* k,
      const gtsam::Pose3& body_P_sensor);
  GPInterpolatedProjectionFactorPose3(const gtsam::Point2& measured,
      const gtsam::noiseModel::Base* cam_model, const gtsam::noiseModel::Base* Qc_model,
      size_t pose1Key, size_t vel1Key, size_t pose2Key, size_t vel2Key,
      size_t pointKey, double delta_t, double tau, const CALIBRATION* k,
      const gtsam::Pose3& body_P_sensor, bool throwCheirality, bool verboseCheirality);
};

typedef gpslam::GPInterpolatedProjectionFactorPose3<gtsam::Cal3_S2> GPInterpolatedProjectionFactorPose3Cal3_S2;



// range factor
#include <gpslam/slam/RangeFactorPose2.h>
virtual class RangeFactorPose2 : gtsam::NoiseModelFactor {
  RangeFactorPose2(size_t posekey, size_t landkey,
      double measured, const gtsam::noiseModel::Base* model);
};

#include <gpslam/slam/GPInterpolatedRangeFactorPose2.h>
virtual class GPInterpolatedRangeFactorPose2 : gtsam::NoiseModelFactor {
  GPInterpolatedRangeFactorPose2(double measured,
      const gtsam::noiseModel::Base* meas_model, const gtsam::noiseModel::Base* Qc_model,
      size_t pose1Key, size_t vel1Key, size_t pose2Key, size_t vel2Key,
      size_t pointKey, double delta_t, double tau);
  GPInterpolatedRangeFactorPose2(double measured,
      const gtsam::noiseModel::Base* meas_model, const gtsam::noiseModel::Base* Qc_model,
      size_t pose1Key, size_t vel1Key, size_t pose2Key, size_t vel2Key,
      size_t pointKey, double delta_t, double tau, const gtsam::Pose2& body_P_sensor);
};


#include <gpslam/slam/GPInterpolatedRangeFactorPose3.h>
virtual class GPInterpolatedRangeFactorPose3 : gtsam::NoiseModelFactor {
  GPInterpolatedRangeFactorPose3(double measured,
      const gtsam::noiseModel::Base* meas_model, const gtsam::noiseModel::Base* Qc_model,
      size_t pose1Key, size_t vel1Key, size_t pose2Key, size_t vel2Key,
      size_t pointKey, double delta_t, double tau);
  GPInterpolatedRangeFactorPose3(double measured,
      const gtsam::noiseModel::Base* meas_model, const gtsam::noiseModel::Base* Qc_model,
      size_t pose1Key, size_t vel1Key, size_t pose2Key, size_t vel2Key,
      size_t pointKey, double delta_t, double tau, const gtsam::Pose3& body_P_sensor);
};

// attitude factor
#include <gpslam/slam/GPInterpolatedAttitudeFactorRot3.h>
virtual class GPInterpolatedAttitudeFactorRot3 : gtsam::NoiseModelFactor {
  GPInterpolatedAttitudeFactorRot3(
      size_t poseKey1, size_t velKey1, size_t poseKey2, size_t velKey2,
      double delta_t, double tau, const gtsam::noiseModel::Base* Qc_model,
      const gtsam::noiseModel::Base* meas_model, const gtsam::Unit3& nZ, const gtsam::Unit3& bRef);
  GPInterpolatedAttitudeFactorRot3(
      size_t poseKey1, size_t velKey1, size_t poseKey2, size_t velKey2,
      double delta_t, double tau, const gtsam::noiseModel::Base* Qc_model,
      const gtsam::noiseModel::Base* meas_model, const gtsam::Unit3& nZ);
};


/// get body-centric velocity from two poses and delta_t
Vector getBodyCentricVb(const gtsam::Pose3& pose1, const gtsam::Pose3& pose2,
    double delta_t);
Vector getBodyCentricVs(const gtsam::Pose3& pose1, const gtsam::Pose3& pose2,
    double delta_t);



////////////////////////////////////////////////////////////////////////////////
// Linear classes
////////////////////////////////////////////////////////////////////////////////


// prior factor
#include <gpslam/gp/GaussianProcessPriorLinear.h>

// template DOF list
template <DOF = {2,3}>
virtual class GaussianProcessPriorLinear : gtsam::NoiseModelFactor {
  GaussianProcessPriorLinear(size_t key1, size_t key2, size_t key3, size_t key4,
      double delta, const gtsam::noiseModel::Base* Qc_model);
};


// util class for all interpolated measurements
#include <gpslam/gp/GaussianProcessInterpolatorLinear.h>

// template DOF list
template <DOF = {2,3}>
class GaussianProcessInterpolatorLinear {
  GaussianProcessInterpolatorLinear(const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
  Vector interpolatePose(Vector pose1, Vector vel1, Vector pose2, Vector vel2) const;
  Vector interpolateVelocity(Vector pose1, Vector vel1, Vector pose2, Vector vel2) const;
};


// odometry factor
#include <gpslam/slam/OdometryFactor2DLinear.h>
virtual class OdometryFactor2DLinear : gtsam::NoiseModelFactor {
  OdometryFactor2DLinear(size_t pose1Key, size_t pose2Key, Vector odomMeasured,
      const gtsam::noiseModel::Base* model);
};


// range factor
#include <gpslam/slam/RangeFactor2DLinear.h>
virtual class RangeFactor2DLinear : gtsam::NoiseModelFactor {
  RangeFactor2DLinear(size_t posekey, size_t landkey,
      double measured, const gtsam::noiseModel::Base* model);
};

#include <gpslam/slam/GPInterpolatedRangeFactor2DLinear.h>
virtual class GPInterpolatedRangeFactor2DLinear : gtsam::NoiseModelFactor {
  GPInterpolatedRangeFactor2DLinear(double measured,
      size_t pose1Key, size_t vel1Key, size_t pose2Key, size_t vel2Key, size_t pointKey,
      const gtsam::noiseModel::Base* meas_model, const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
};


// bearing range factor
#include <gpslam/slam/RangeBearingFactor2DLinear.h>
virtual class RangeBearingFactor2DLinear : gtsam::NoiseModelFactor {
  RangeBearingFactor2DLinear(size_t posekey, size_t landkey, double measured,
      const gtsam::Rot2& bearing, const gtsam::noiseModel::Base* model);
};

}

