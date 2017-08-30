/**
 *  @file  GaussianProcessPriorPose2.h
 *  @brief Pose2 GP prior
 *  @author Jing Dong
 **/

#pragma once

#include <gpslam/gp/GPutils.h>

#include <gtsam/geometry/concepts.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>

#include <boost/lexical_cast.hpp>
#include <boost/serialization/export.hpp>

#include <ostream>


namespace gpslam {

/**
 * 4-way factor for Gaussian Process prior factor, SE(2) version
 */
class GaussianProcessPriorPose2: public gtsam::NoiseModelFactor4<gtsam::Pose2,
    gtsam::Vector3, gtsam::Pose2, gtsam::Vector3> {

private:
  double delta_t_;
  typedef GaussianProcessPriorPose2 This;
  typedef gtsam::NoiseModelFactor4<gtsam::Pose2, gtsam::Vector3, gtsam::Pose2, gtsam::Vector3> Base;

public:

  GaussianProcessPriorPose2() {}	/* Default constructor only for serialization */

  /// Constructor
  /// @param delta_t is the time between the two states
  GaussianProcessPriorPose2(gtsam::Key poseKey1, gtsam::Key velKey1,
      gtsam::Key poseKey2, gtsam::Key velKey2,
      double delta_t, const gtsam::SharedNoiseModel& Qc_model) :
    Base(gtsam::noiseModel::Gaussian::Covariance(calcQ<3>(getQc(Qc_model), delta_t)),
        poseKey1, velKey1, poseKey2, velKey2) {
    delta_t_ = delta_t;
  }

  virtual ~GaussianProcessPriorPose2() {}


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /// factor error function
  gtsam::Vector evaluateError(
      const gtsam::Pose2& pose1, const gtsam::Vector3& vel1,
      const gtsam::Pose2& pose2, const gtsam::Vector3& vel2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none,
      boost::optional<gtsam::Matrix&> H4 = boost::none) const {

    using namespace gtsam;

    Matrix3 Hinv, Hcomp1, Hcomp2, Hlogmap;
    Vector3 r;
    if (H1 || H2 || H3 || H4)
      r = Pose2::Logmap(pose1.inverse(Hinv).compose(pose2, Hcomp1, Hcomp2), Hlogmap);
    else
      r = Pose2::Logmap(pose1.inverse().compose(pose2));

    // jacobians
    if (H1) *H1 = (Matrix63() << Hlogmap * Hcomp1 * Hinv, Matrix3::Zero()).finished();
    if (H2) *H2 = (Matrix63() << -delta_t_ * Matrix3::Identity(), -Matrix3::Identity()).finished();
    if (H3) *H3 = (Matrix63() << Hlogmap * Hcomp2, Matrix3::Zero()).finished();
    if (H4) *H4 = (Matrix63() << Matrix3::Zero(), Matrix3::Identity()).finished();

    return (Vector(6) << (r - vel1 * delta_t_), (vel2 - vel1)).finished();
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
    std::cout << s << "4-way Gaussian Process Factor Pose2" << std::endl;
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

}; // GaussianProcessPriorPose2

} // namespace gpslam


/// traits
namespace gtsam {
template<>
struct traits<gpslam::GaussianProcessPriorPose2> : public Testable<gpslam::GaussianProcessPriorPose2> {};
}
