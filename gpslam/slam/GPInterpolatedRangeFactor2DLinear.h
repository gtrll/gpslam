/**
 *  @file  GPInterpolatedRangeFactor2DLinear.h
 *  @brief range factor for 2D linear pose, interpolated
 *  @author Jing Dong
 **/

#pragma once

#include <gpslam/gp/GaussianProcessInterpolatorLinear.h>

#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Vector.h>


namespace gpslam {

/**
 * Binary factor for range measurement, GP interpolated, 2D linear pose version
 */
class GPInterpolatedRangeFactor2DLinear: public gtsam::NoiseModelFactor5<gtsam::Vector3, gtsam::Vector3,
    gtsam::Vector3, gtsam::Vector3, gtsam::Point2> {

private:
  // interpolater
  GaussianProcessInterpolatorLinear<3> GPbase_;

  double measured_; /** measurement */

  typedef GPInterpolatedRangeFactor2DLinear This;
  typedef gtsam::NoiseModelFactor5<gtsam::Vector3, gtsam::Vector3, gtsam::Vector3,
      gtsam::Vector3, gtsam::Point2> Base;
  typedef GaussianProcessInterpolatorLinear<3> GPBase;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  GPInterpolatedRangeFactor2DLinear() {} /* Default constructor */

  GPInterpolatedRangeFactor2DLinear(double measured,
      gtsam::Key pose1Key, gtsam::Key vel1Key, gtsam::Key pose2Key, gtsam::Key vel2Key,
      gtsam::Key pointKey,
      const gtsam::SharedNoiseModel& meas_model, const gtsam::SharedNoiseModel& Qc_model,
      double delta_t, double tau) :
        Base(meas_model, pose1Key, vel1Key, pose2Key, vel2Key, pointKey),
        GPbase_(Qc_model, delta_t, tau),
        measured_(measured) {
  }

  virtual ~GPInterpolatedRangeFactor2DLinear() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** factor error */
  gtsam::Vector evaluateError(const gtsam::Vector3& pose1, const gtsam::Vector3& vel1,
      const gtsam::Vector3& pose2, const gtsam::Vector3& vel2, const gtsam::Point2& point,
      boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none,
      boost::optional<gtsam::Matrix&> H5 = boost::none) const {

    using namespace gtsam;

    // interpolate pose
    Matrix3 Hint1, Hint2, Hint3, Hint4;
    Vector3 pose;
    if (H1 || H2 || H3 || H4)
      pose = GPbase_.interpolatePose(pose1, vel1, pose2, vel2, Hint1, Hint2, Hint3, Hint4);
    else
      pose = GPbase_.interpolatePose(pose1, vel1, pose2, vel2);

    Point2 d = point - Point2(pose(0), pose(1));
    Matrix12 H;
    double r = d.norm(H);

    Matrix Hpose;
    if (H1 || H2 || H3 || H4) {
      Hpose = (Matrix13() << -H, 0.0).finished();
      GPBase::updatePoseJacobians(Hpose, Hint1, Hint2, Hint3, Hint4, H1, H2, H3, H4);
    }

    if (H5) *H5 = H;
    return (Vector(1) << r - measured_).finished();
  }

  /** return the measured */
  double measured() const {
    return measured_;
  }

  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
    const This *e = dynamic_cast<const This*> (&expected);
    return e != NULL && Base::equals(*e, tol)
        && fabs(this->measured_ - e->measured_) < tol;
  }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "RangeFactor, range = " << measured_ << std::endl;
    Base::print("", keyFormatter);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }

}; // GPInterpolatedRangeFactor2DLinear


} // namespace gpslam


/// traits
namespace gtsam {
template<>
struct traits<gpslam::GPInterpolatedRangeFactor2DLinear> : public Testable<gpslam::GPInterpolatedRangeFactor2DLinear> {};
}

