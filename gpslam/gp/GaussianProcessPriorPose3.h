/**
 *  @file  GaussianProcessPriorPose3.h
 *  @brief Pose3 GP prior
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
#include <boost/serialization/export.hpp>

#include <ostream>


namespace gpslam {

/**
 * 4-way factor for Gaussian Process prior factor, SE(3) version
 */
class GaussianProcessPriorPose3: public gtsam::NoiseModelFactor4<gtsam::Pose3,
    gtsam::Vector6, gtsam::Pose3, gtsam::Vector6> {

private:
  double delta_t_;
  typedef GaussianProcessPriorPose3 This;
  typedef gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector6, gtsam::Pose3, gtsam::Vector6> Base;

public:

  GaussianProcessPriorPose3() {}	/* Default constructor only for serialization */

  /// Constructor
  /// @param delta_t is the time between the two states
  GaussianProcessPriorPose3(gtsam::Key poseKey1, gtsam::Key velKey1,
      gtsam::Key poseKey2, gtsam::Key velKey2,
      double delta_t, const gtsam::SharedNoiseModel& Qc_model) :
    Base(gtsam::noiseModel::Gaussian::Covariance(calcQ<6>(getQc(Qc_model), delta_t)),
        poseKey1, velKey1, poseKey2, velKey2) {
    delta_t_ = delta_t;
  }

  virtual ~GaussianProcessPriorPose3() {}


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /// factor error function
  gtsam::Vector evaluateError(const gtsam::Pose3& pose1, const gtsam::Vector6& vel1,
      const gtsam::Pose3& pose2, const gtsam::Vector6& vel2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none,
      boost::optional<gtsam::Matrix&> H4 = boost::none) const {

    using namespace gtsam;

    Matrix6 Hinv, Hcomp1, Hcomp2, Hlogmap;
    Vector6 r;
    if (H1 || H2 || H3 || H4)
      r = Pose3::Logmap(pose1.inverse(Hinv).compose(pose2, Hcomp1, Hcomp2), Hlogmap);
    else
      r = Pose3::Logmap(pose1.inverse().compose(pose2));

    const Matrix6 Jinv = rightJacobianPose3inv(r);

    // jacobians
    if (H1) {
      const Matrix6 J_Ti = Hlogmap * Hcomp1 * Hinv;
      const Matrix6 Jdiff_Ti = jacobianMethodNumercialDiff(rightJacobianPose3inv, r,
          vel2) * J_Ti;
      *H1 = (Matrix_12_6() << J_Ti, Jdiff_Ti).finished();
    }

    if (H2) *H2 = (Matrix_12_6() << -delta_t_ * Matrix6::Identity(), -Matrix6::Identity()).finished();

    if (H3) {
      const Matrix6 J_Ti1 = Hlogmap * Hcomp2;
      const Matrix6 Jdiff_Ti1 = jacobianMethodNumercialDiff(rightJacobianPose3inv, r,
          vel2) * J_Ti1;
      *H3 = (Matrix_12_6() << J_Ti1, Jdiff_Ti1).finished();
    }

    if (H4) *H4 = (Matrix_12_6() << Matrix6::Zero(), Jinv).finished();

    return (Vector(12) << (r - vel1 * delta_t_), (Jinv * vel2 - vel1)).finished();
  }

  /** number of variables attached to this factor */
  size_t size() const {
    return 4;
  }

  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
    const This *e =  dynamic_cast<const This*> (&expected);
    return e != NULL && Base::equals(*e, tol) && fabs(this->delta_t_ - e->delta_t_) < tol;
  }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "4-way Gaussian Process Factor Pose3" << std::endl;
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
struct traits<gpslam::GaussianProcessPriorPose3> : public Testable<gpslam::GaussianProcessPriorPose3> {};
}
