/**
 * @file GaussianProcessInterpolatorPose2.h
 * @brief Base and utils for Gaussian Process Interpolated measurement factor, works only in SE(2)
 * @author Jing Dong
 */

#pragma once

#include <gpslam/gp/GPutils.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Matrix.h>


namespace gpslam {

/**
 * 4-way factor for Gaussian Process interpolator, SE(2) version
 * interpolate pose and velocity given consecutive poses and velocities
 */
class GaussianProcessInterpolatorPose2 {

private:
  typedef GaussianProcessInterpolatorPose2 This;
  double delta_t_;		// t_{i+1} - t_i
  double tau_;			// tau - t_i. we use tau as time interval from t_i instead of from t_0 as in Barfoot papers
  gtsam::Matrix3 Qc_;
  gtsam::Matrix6 Lambda_;
  gtsam::Matrix6 Psi_;

public:

  /// Default constructor: only for serialization
  GaussianProcessInterpolatorPose2() {}

  /**
   * Constructor
   * @param Qc noise model of Qc
   * @param delta_t the time between the two states
   * @param tau the time of interval status
   */
  GaussianProcessInterpolatorPose2(const gtsam::SharedNoiseModel& Qc_model, double delta_t, double tau) :
      delta_t_(delta_t), tau_(tau) {

    // Calcuate Lambda and Psi
    Qc_ = getQc(Qc_model);
    Lambda_ = calcLambda(Qc_, delta_t_, tau_);
    Psi_ = calcPsi(Qc_, delta_t_, tau_);
  }

  /** Virtual destructor */
  virtual ~GaussianProcessInterpolatorPose2() {}


  /// interpolate pose with Jacobians
  gtsam::Pose2 interpolatePose(
      const gtsam::Pose2& pose1, const gtsam::Vector3& vel1,
      const gtsam::Pose2& pose2, const gtsam::Vector3& vel2,
      gtsam::OptionalJacobian<3, 3> H1 = boost::none,
      gtsam::OptionalJacobian<3, 3> H2 = boost::none,
      gtsam::OptionalJacobian<3, 3> H3 = boost::none,
      gtsam::OptionalJacobian<3, 3> H4 = boost::none) const {

    using namespace gtsam;

    const Vector6 r1 = (Vector6() << Vector3::Zero(), vel1).finished();
    Matrix3 Hinv, Hcomp11, Hcomp12, Hlogmap;
    Vector3 r;
    if (H1 || H2 || H3 || H4)
      r = Pose2::Logmap(pose1.inverse(Hinv).compose(pose2, Hcomp11, Hcomp12), Hlogmap);
    else
      r = Pose2::Logmap(pose1.inverse().compose(pose2));
    const Vector6 r2 = (Vector6() << r, vel2).finished();

    Pose2 pose;
    if (H1 || H2 || H3 || H4) {
      Matrix3 Hcomp21, Hcomp22, Hexp;
      pose = pose1.compose(Pose2::Expmap(Lambda_.block<3, 6>(0, 0) * r1 + Psi_.block<3, 6>(0, 0) * r2, Hexp), Hcomp21, Hcomp22);
      Matrix3 Hexpr1 = Hcomp22*Hexp;
      if (H1) *H1 = Hcomp21 + Hexpr1*Psi_.block<3, 3>(0, 0)*Hlogmap*Hcomp11*Hinv;
      if (H2) *H2 = Hexpr1*Lambda_.block<3, 3>(0, 3);
      if (H3) *H3 = Hexpr1*Psi_.block<3, 3>(0, 0)*Hlogmap*Hcomp12;
      if (H4) *H4 = Hexpr1*Psi_.block<3, 3>(0, 3);
    } else {
      pose = pose1.compose(Pose2::Expmap(Lambda_.block<3, 6>(0, 0) * r1 + Psi_.block<3, 6>(0, 0) * r2));
    }

    return pose;
  }

  /// update jacobian based on interpolated jacobians
  static void updatePoseJacobians(const gtsam::Matrix& Hpose,  const gtsam::Matrix& Hint1,
      const gtsam::Matrix& Hint2, const gtsam::Matrix& Hint3, const gtsam::Matrix& Hint4,
      boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
      boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4) {
    if (H1) *H1 = Hpose * Hint1;
    if (H2) *H2 = Hpose * Hint2;
    if (H3) *H3 = Hpose * Hint3;
    if (H4) *H4 = Hpose * Hint4;
  }

  /// interpolate velocity with Jacobians
  /// TODO: implementation
  gtsam::Vector3 interpolateVelocity(
      const gtsam::Pose2& pose1, const gtsam::Vector3& vel1,
      const gtsam::Pose2& pose2, const gtsam::Vector3& vel2,
      gtsam::OptionalJacobian<3, 3> H1 = boost::none,
      gtsam::OptionalJacobian<3, 3> H2 = boost::none,
      gtsam::OptionalJacobian<3, 3> H3 = boost::none,
      gtsam::OptionalJacobian<3, 3> H4 = boost::none) const ;


  /**
   * Testables
   */

  /** equals specialized to this factor */
  virtual bool equals(const This& expected, double tol=1e-9) const {
    return fabs(this->delta_t_ - expected.delta_t_) < tol &&
        fabs(this->tau_ - expected.tau_) < tol &&
        gtsam::equal_with_abs_tol(this->Qc_, expected.Qc_, tol) &&
        gtsam::equal_with_abs_tol(this->Lambda_, expected.Lambda_, tol) &&
        gtsam::equal_with_abs_tol(this->Psi_, expected.Psi_, tol);
  }

  /** print contents */
  void print(const std::string& s="") const {
    std::cout << s << "GaussianProcessInterpolatorPose2" << std::endl;
    std::cout << "delta_t = " << delta_t_ << ", tau = " << tau_ << std::endl;
    //std::cout << "Qc = " << Qc_ << std::endl;
  }


private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(delta_t_);
    ar & BOOST_SERIALIZATION_NVP(tau_);
    using namespace boost::serialization;
    ar & make_nvp("Qc", make_array(Qc_.data(), Qc_.size()));
    ar & make_nvp("Lambda", make_array(Lambda_.data(), Lambda_.size()));
    ar & make_nvp("Psi", make_array(Psi_.data(), Psi_.size()));
  }
};

} // \ namespace gpslam


/// traits
namespace gtsam {
template<>
struct traits<gpslam::GaussianProcessInterpolatorPose2> : public Testable<
    gpslam::GaussianProcessInterpolatorPose2> {};
}
