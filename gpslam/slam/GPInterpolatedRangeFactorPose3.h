/**
 *  @file  GPInterpolatedRangeFactorPose3.H
 *  @brief range factor for SE(3) pose, interpolated
 *  @author Frank Dellaert, Jing Dong
 **/

#pragma once

#include <gpslam/gp/GaussianProcessInterpolatorPose3.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/lexical_cast.hpp>


namespace gpslam {

/**
 * Binary factor for range measurement, GP interpolated, SE(3) pose version
 */
class GPInterpolatedRangeFactorPose3: public gtsam::NoiseModelFactor5<gtsam::Pose3,
    gtsam::Vector6, gtsam::Pose3, gtsam::Vector6, gtsam::Point3> {

private:
  // interpolater
  GaussianProcessInterpolatorPose3 GPbase_;

  double measured_; /** measurement */
  boost::optional<gtsam::Pose3> body_P_sensor_; ///< The pose of the sensor in the body frame

  typedef GPInterpolatedRangeFactorPose3 This;
  typedef gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Vector6, gtsam::Pose3, gtsam::Vector6,
      gtsam::Point3> Base;
  typedef GaussianProcessInterpolatorPose3 GPBase;


public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  GPInterpolatedRangeFactorPose3() {} /* Default constructor */

  GPInterpolatedRangeFactorPose3(double measured,
      const gtsam::SharedNoiseModel& meas_model, const gtsam::SharedNoiseModel& Qc_model,
      gtsam::Key poseKey1, gtsam::Key velKey1, gtsam::Key poseKey2, gtsam::Key velKey2,
      gtsam::Key pointKey,
      double delta_t, double tau, boost::optional<gtsam::Pose3> body_P_sensor = boost::none) :
        Base(meas_model, poseKey1, velKey1, poseKey2, velKey2, pointKey),
        GPbase_(Qc_model, delta_t, tau),
        measured_(measured), body_P_sensor_(body_P_sensor) {
  }

  virtual ~GPInterpolatedRangeFactorPose3() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** factor error */
  gtsam::Vector evaluateError(const gtsam::Pose3& pose1, const gtsam::Vector6& vel1,
      const gtsam::Pose3& pose2, const gtsam::Vector6& vel2, const gtsam::Point3& point,
      boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none,
      boost::optional<gtsam::Matrix&> H5 = boost::none) const {

    using namespace gtsam;

    // interpolate pose
    Matrix Hint1, Hint2, Hint3, Hint4;
    Pose3 pose;
    if (H1 || H2 || H3 || H4)
      pose = GPbase_.interpolatePose(pose1, vel1, pose2, vel2, Hint1, Hint2, Hint3, Hint4);
    else
      pose = GPbase_.interpolatePose(pose1, vel1, pose2, vel2);

    Matrix Hpose;     // jacobian of pose
    double hx;      // measure

    if (body_P_sensor_) {
      if (H1 || H2 || H3 || H4) {
        Matrix H0;
        hx = pose.compose(*body_P_sensor_, H0).range(point, Hpose, H5);
        Hpose = Hpose * H0;
        GPBase::updatePoseJacobians(Hpose, Hint1, Hint2, Hint3, Hint4, H1, H2, H3, H4);
      } else {
        hx = pose.compose(*body_P_sensor_).range(point, Hpose, H5);
      }
    } else {
      hx = pose.range(point, Hpose, H5);
      GPBase::updatePoseJacobians(Hpose, Hint1, Hint2, Hint3, Hint4, H1, H2, H3, H4);
    }

    return (Vector(1) << hx - measured_).finished();
  }

  /** return the measured */
  double measured() const {
    return measured_;
  }

  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
    const This *e = dynamic_cast<const This*> (&expected);
    return e != NULL
        && Base::equals(*e, tol)
    && this->GPbase_.equals(e->GPbase_)
    && fabs(this->measured_ - e->measured_) < tol
    && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
  }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "RangeFactor, range = " << measured_ << std::endl;
    if(this->body_P_sensor_)
      this->body_P_sensor_->print("Sensor pose in body frame: ");
    Base::print("", keyFormatter);
    GPbase_.print("Measurement taken from: ");
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor5",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(GPbase_);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
}; // GPInterpolatedRangeFactorPose3


} // namespace gpslam



/// traits
namespace gtsam {
template<>
struct traits<gpslam::GPInterpolatedRangeFactorPose3> : public Testable<gpslam::GPInterpolatedRangeFactorPose3> {};
}

