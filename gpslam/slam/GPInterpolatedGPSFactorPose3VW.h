/**
 *  @file  GPInterpolatedGPSFactorPose3VW.h
 *  @author Jing Dong
 *  @brief Translational measurement like GPS, GP interpolated version
 *  @date Oct 4, 2015
 **/

#pragma once

#include <gpslam/gp/GaussianProcessInterpolatorPose3VW.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/lexical_cast.hpp>


namespace gpslam {

/**
 * Binary factor for an translational measurement like GPS, GP interpolated
 */
class GPInterpolatedGPSFactorPose3VW: public gtsam::NoiseModelFactor6<gtsam::Pose3,
    gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {

private:

  // interpolater
  GaussianProcessInterpolatorPose3VW GPbase_;

  gtsam::Point3 measured_; /** measurement */
  boost::optional<gtsam::Pose3> body_P_sensor_; ///< The pose of the sensor in the body frame

  typedef GPInterpolatedGPSFactorPose3VW This;
  typedef gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Interpolator;
  typedef GaussianProcessInterpolatorPose3VW GPBase;


public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  GPInterpolatedGPSFactorPose3VW() {} /* Default constructor */

  /**
   * Constructor
   * @param body_P_sensor transformation from body to sensor
   */
  GPInterpolatedGPSFactorPose3VW(const gtsam::Point3& measured_point3,
      const gtsam::SharedNoiseModel& meas_model, const gtsam::SharedNoiseModel& Qc_model,
      gtsam::Key poseKey1, gtsam::Key velKey1, gtsam::Key omegaKey1,
      gtsam::Key poseKey2, gtsam::Key velKey2, gtsam::Key omegaKey2,
      double delta_t, double tau, boost::optional<gtsam::Pose3> body_P_sensor = boost::none) :
        Interpolator(meas_model, poseKey1, velKey1, omegaKey1, poseKey2, velKey2, omegaKey2),
        GPbase_(Qc_model, delta_t, tau),
        measured_(measured_point3), body_P_sensor_(body_P_sensor) {
  }

  virtual ~GPInterpolatedGPSFactorPose3VW() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** factor error */
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose1, const gtsam::Vector3& vel1, const gtsam::Vector3& omega1,
      const gtsam::Pose3& pose2, const gtsam::Vector3& vel2, const gtsam::Vector3& omega2,
      boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none,
      boost::optional<gtsam::Matrix&> H5 = boost::none, boost::optional<gtsam::Matrix&> H6 = boost::none) const {

    using namespace gtsam;

    // interpolate pose
    Matrix Hint1, Hint2, Hint3, Hint4, Hint5, Hint6;
    Pose3 pose;
    if (H1 || H2 || H3 || H4 || H5 || H6)
      pose = GPbase_.interpolatePose(pose1, vel1, omega1, pose2, vel2, omega2,
          Hint1, Hint2, Hint3, Hint4, Hint5, Hint6);
    else
      pose = GPbase_.interpolatePose(pose1, vel1, omega1, pose2, vel2, omega2);

    Matrix Hpose;     // jacobian of pose
    Point3 point_err;

    if (body_P_sensor_) {
      Matrix H0;
      point_err = pose.compose(*body_P_sensor_, H0).translation(Hpose) - measured_;
      Hpose = Hpose * H0;
      GPBase::updatePoseJacobians(Hpose, Hint1, Hint2, Hint3, Hint4, Hint5, Hint6,
          H1, H2, H3, H4, H5, H6);
    } else {
      point_err = pose.translation(Hpose) - measured_;
      GPBase::updatePoseJacobians(Hpose, Hint1, Hint2, Hint3, Hint4, Hint5, Hint6,
          H1, H2, H3, H4, H5, H6);
    }

    return point_err.vector();
  }

  /** return the measured */
  gtsam::Point3 measured() const {
    return measured_;
  }

  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
    const This *e = dynamic_cast<const This*> (&expected);
    return e != NULL
        && Base::equals(*e, tol) && GPbase_.equals(e->GPbase_)
        && (this->measured_ - e->measured_).norm() < tol
        && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
  }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "GPSFactor, point = " << measured_ << std::endl;
    if(this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
    Base::print("", keyFormatter);
    GPbase_.print("Measurement interpolated at: ");
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(GPbase_);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
}; // GPInterpolatedGPSFactorPose3VW

} // namespace gpslam


/// traits
namespace gtsam {
template<>
struct traits<gpslam::GPInterpolatedGPSFactorPose3VW> : public Testable<gpslam::GPInterpolatedGPSFactorPose3VW> {};
}

