/**
 *  @file  GaussianProcessPriorPose3VW.h
 *  @brief Pose3 GP prior, use V/W velocity representation
 *  @author Xinyan Yan, Jing Dong
 **/

#pragma once

#include <gpslam/gp/GPutils.h>
#include <gpslam/gp/Pose3utils.h>

#include <gtsam/geometry/concepts.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>

#include <boost/lexical_cast.hpp>

#include <ostream>


namespace gpslam {

/**
 * 4-way factor for Gaussian Process prior factor, SE(3) version
 * 6-DOF velocity is represented by 3-DOF translational and 3-DOF rotational velocities (in body frame).
 */
class GaussianProcessPriorPose3VW: public gtsam::NoiseModelFactor6<gtsam::Pose3,
    gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {

private:
  double delta_t_;
  typedef GaussianProcessPriorPose3VW This;
  typedef gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Base;

public:

  GaussianProcessPriorPose3VW() {}	/* Default constructor only for serialization */

  /// Constructor
  /// @param delta_t is the time between the two states
  GaussianProcessPriorPose3VW(
      gtsam::Key poseKey1, gtsam::Key velKey1, gtsam::Key omegaKey1,
      gtsam::Key poseKey2, gtsam::Key velKey2, gtsam::Key omegaKey2,
      double delta_t, const gtsam::SharedNoiseModel& Qc_model) :
    Base(gtsam::noiseModel::Gaussian::Covariance(calcQ<6>(getQc(Qc_model), delta_t)),
        poseKey1, velKey1, omegaKey1, poseKey2, velKey2, omegaKey2) {
    delta_t_ = delta_t;
  }

  virtual ~GaussianProcessPriorPose3VW() {}


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /// factor error function
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose1, const gtsam::Vector3& vel1, const gtsam::Vector3& omega1,
      const gtsam::Pose3& pose2, const gtsam::Vector3& vel2, const gtsam::Vector3& omega2,
      boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none,
      boost::optional<gtsam::Matrix&> H5 = boost::none, boost::optional<gtsam::Matrix&> H6 = boost::none) const {

    using namespace gtsam;

    Matrix6 Hinv, Hcomp1, Hcomp2, Hlogmap;
    Vector6 r;
    if (H1 || H4)
      r = Pose3::Logmap(pose1.inverse(Hinv).compose(pose2, Hcomp1, Hcomp2), Hlogmap);
    else
      r = Pose3::Logmap(pose1.inverse().compose(pose2));

    const Matrix6 Jinv = rightJacobianPose3inv(r);

    // convert body frame velocity to body center
    Matrix63 H1v, H1w, H2v, H2w;
    Matrix6 H1p, H2p;
    Matrix_12_6 Hv1, Hv2;
    Vector6 v1, v2;
    if (H2 || H3 || H5 || H6) {
      v1 = convertVWtoVb(vel1, omega1, pose1, H1v, H1w, H1p);
      v2 = convertVWtoVb(vel2, omega2, pose2, H2v, H2w, H2p);
      Hv1 = (Matrix_12_6() << -delta_t_ * Matrix6::Identity(), -Matrix6::Identity()).finished();
      Hv2 = (Matrix_12_6() << Matrix6::Zero(), Jinv).finished();
    } else {
      v1 = convertVWtoVb(vel1, omega1, pose1);
      v2 = convertVWtoVb(vel2, omega2, pose2);
    }

    // jacobians
    if (H1)  {
      const Matrix6 J_Ti = Hlogmap * Hcomp1 * Hinv;
      const Matrix6 Jdiff_Ti = jacobianMethodNumercialDiff(rightJacobianPose3inv, r,
          v2) * J_Ti;
      *H1 = (Matrix_12_6() << J_Ti-delta_t_*H1p, Jdiff_Ti-H1p).finished();
    }

    if (H2) *H2 = Hv1 * H1v;
    if (H3) *H3 = Hv1 * H1w;

    if (H4) {
      const Matrix6 J_Ti1 = Hlogmap * Hcomp2;
      const Matrix6 Jdiff_Ti1 = jacobianMethodNumercialDiff(rightJacobianPose3inv, r,
          v2) * J_Ti1;
      *H4 = (Matrix_12_6() << J_Ti1, Jdiff_Ti1 + Jinv*H2p).finished();
    }

    if (H5) *H5 = Hv2 * H2v;
    if (H6) *H6 = Hv2 * H2w;

    return (Vector(12) << (r - v1 * delta_t_), Jinv * v2 - v1).finished();
  }

  /** number of variables attached to this factor */
  size_t size() const {
    return 6;
  }

  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
    const This *e =  dynamic_cast<const This*> (&expected);
    return e != NULL && Base::equals(*e, tol) && fabs(this->delta_t_ - e->delta_t_) < tol;
  }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "4-way Gaussian Process Factor Pose3 VW" << std::endl;
    Base::print("", keyFormatter);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(delta_t_);
  }

}; // GaussianProcessPriorPose3


} // namespace gpslam



/// traits
namespace gtsam {
template<>
struct traits<gpslam::GaussianProcessPriorPose3VW> : public Testable<gpslam::GaussianProcessPriorPose3VW> {};
}

