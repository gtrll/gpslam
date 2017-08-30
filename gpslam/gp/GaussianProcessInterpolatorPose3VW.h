/**
 * @file GaussianProcessInterpolatorPose3VW.h
 * @brief Base and utils for Gaussian Process Interpolated measurement factor, SE(3). use V/W velocity representation
 * @author Jing Dong, Xinyan Yan
 */

#pragma once

#include <gpslam/gp/GPutils.h>
#include <gpslam/gp/Pose3utils.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>


namespace gpslam {

/**
 * 4-way factor for Gaussian Process interpolator, SE(3) version
 * 6-DOF velocity is represented by 3-DOF translational and 3-DOF rotational velocities (in body frame).
 * interpolate pose and velocity given consecutive poses and velocities
 */
class GaussianProcessInterpolatorPose3VW {

private:
  typedef GaussianProcessInterpolatorPose3VW This;
  double delta_t_;		// t_{i+1} - t_i
  double tau_;			// tau - t_i. we use tau as time interval from t_i instead of from t_0 as in Barfoot papers
  gtsam::Matrix6 Qc_;
  Matrix_12 Lambda_;
  Matrix_12 Psi_;

public:

  /// Default constructor: only for serialization
  GaussianProcessInterpolatorPose3VW() {}

  /**
   * Constructor
   * @param Qc noise model of Qc
   * @param delta_t the time between the two states
   * @param tau the time of interval status
   */
  GaussianProcessInterpolatorPose3VW(const gtsam::SharedNoiseModel& Qc_model, double delta_t, double tau) :
      delta_t_(delta_t), tau_(tau) {

    // Calcuate Lambda and Psi
    Qc_ = getQc(Qc_model);
    Lambda_ = calcLambda(Qc_, delta_t_, tau_);
    Psi_ = calcPsi(Qc_, delta_t_, tau_);
  }

  /** Virtual destructor */
  virtual ~GaussianProcessInterpolatorPose3VW() {}


  /// interpolate pose with Jacobians
  gtsam::Pose3 interpolatePose(
      const gtsam::Pose3& pose1, const gtsam::Vector3& v1, const gtsam::Vector3& omega1,
      const gtsam::Pose3& pose2, const gtsam::Vector3& v2, const gtsam::Vector3& omega2,
      gtsam::OptionalJacobian<6, 6> H1 = boost::none, gtsam::OptionalJacobian<6, 3> H2 = boost::none,
      gtsam::OptionalJacobian<6, 3> H3 = boost::none, gtsam::OptionalJacobian<6, 6> H4 = boost::none,
      gtsam::OptionalJacobian<6, 3> H5 = boost::none, gtsam::OptionalJacobian<6, 3> H6 = boost::none) const {

    using namespace gtsam;

    Matrix6 Hinv, Hcomp11, Hcomp12, Hlogmap;
    Vector6 r;
    if (H1 || H4)
      r = Pose3::Logmap(pose1.inverse(Hinv).compose(pose2, Hcomp11, Hcomp12), Hlogmap);
    else
      r = Pose3::Logmap(pose1.inverse().compose(pose2));

    const Matrix6 Jinv = rightJacobianPose3inv(r);

    // vel
    Matrix63 H1v, H1w, H2v, H2w;
    Matrix6 H1p, H2p;
    Vector6 vel1, vel2;
    if (H2 || H3 || H5 || H6) {
      vel1 = convertVWtoVb(v1, omega1, pose1, H1v, H1w, H1p);
      vel2 = convertVWtoVb(v2, omega2, pose2, H2v, H2w, H2p);
    } else {
      vel1 = convertVWtoVb(v1, omega1, pose1);
      vel2 = convertVWtoVb(v2, omega2, pose2);
    }
    const Vector_12 r1 = (Vector_12() << Vector6::Zero(), vel1).finished();
    const Vector_12 r2 = (Vector_12() << r, Jinv * vel2).finished();

    // pose
    Pose3 pose;
    if (H1 || H2 || H3 || H4) {
      Matrix6 Hcomp21, Hcomp22, Hexp;
      pose = pose1.compose(Pose3::Expmap(Lambda_.block<6, 12>(0, 0) * r1 + Psi_.block<6, 12>(0, 0) * r2, Hexp), Hcomp21, Hcomp22);
      Matrix6 Hexpr1 = Hcomp22*Hexp;
      Matrix6 Hvel1 = Hexpr1*Lambda_.block<6, 6>(0, 6);
      Matrix6 Hvel2 = Hexpr1*Psi_.block<6, 6>(0, 6) * Jinv;

      if (H1) {
        const Matrix6 tmp = Hlogmap * Hcomp11 * Hinv;
        const Matrix_12_6 dr2_dT1 = (Matrix_12_6() << tmp, jacobianMethodNumercialDiff(
            rightJacobianPose3inv, r, vel2) * tmp).finished();
        *H1 = Hcomp21 + Hexpr1*Psi_.block<6, 12>(0, 0)*dr2_dT1 + Hvel1*H1p;
      }

      if (H2) *H2 = Hvel1 * H1v;
      if (H3) *H3 = Hvel1 * H1w;

      if (H4) {
        const Matrix6 tmp = Hlogmap * Hcomp12;
        const Matrix_12_6 dr2_dT2 = (Matrix_12_6() << tmp, jacobianMethodNumercialDiff(
            rightJacobianPose3inv, r, vel2) * tmp).finished();
        *H4 = Hexpr1*Psi_.block<6, 12>(0, 0)*dr2_dT2 + Hvel2*H2p;
      }

      if (H5) *H5 = Hvel2 * H2v;
      if (H6) *H6 = Hvel2 * H2w;

    } else {
      pose = pose1.compose(Pose3::Expmap(Lambda_.block<6, 12>(0, 0) * r1 + Psi_.block<6, 12>(0, 0) * r2));
    }

    return pose;
  }

  /// update jacobian based on interpolated jacobians
  static void updatePoseJacobians(const gtsam::Matrix& Hpose,
      const gtsam::Matrix& Hint1, const gtsam::Matrix& Hint2, const gtsam::Matrix& Hint3,
      const gtsam::Matrix& Hint4, const gtsam::Matrix& Hint5, const gtsam::Matrix& Hint6,
      boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
      boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4,
      boost::optional<gtsam::Matrix&> H5, boost::optional<gtsam::Matrix&> H6) {
    if (H1) *H1 = Hpose * Hint1;
    if (H2) *H2 = Hpose * Hint2;
    if (H3) *H3 = Hpose * Hint3;
    if (H4) *H4 = Hpose * Hint4;
    if (H5) *H5 = Hpose * Hint5;
    if (H6) *H6 = Hpose * Hint6;
  }

  /// interpolate velocity with Jacobians
  /// TODO: implementation
  gtsam::Vector6 interpolateVelocity(
      const gtsam::Pose3& pose1, const gtsam::Vector3& v1, const gtsam::Vector3& omega1,
      const gtsam::Pose3& pose2, const gtsam::Vector3& v2, const gtsam::Vector3& omega2,
      gtsam::OptionalJacobian<6, 6> H1 = boost::none, gtsam::OptionalJacobian<6, 3> H2 = boost::none,
      gtsam::OptionalJacobian<6, 3> H3 = boost::none, gtsam::OptionalJacobian<6, 6> H4 = boost::none,
      gtsam::OptionalJacobian<6, 3> H5 = boost::none, gtsam::OptionalJacobian<6, 3> H6 = boost::none) const ;


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
    std::cout << s << "GaussianProcessInterpolatorPose3VW" << std::endl;
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
struct traits<gpslam::GaussianProcessInterpolatorPose3VW> : public Testable<
    gpslam::GaussianProcessInterpolatorPose3VW> {};
}
