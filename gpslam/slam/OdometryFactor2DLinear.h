/**
*  @file  OdometryFactor2DLinear.H
*  @brief 2-way odometry factor, 2D linear pose version
*  @author Xinyan Yan, Jing Dong
*/


#pragma once

#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Pose2.h>

#include <ostream>


namespace gpslam {

/**
 * 2-way odometry factor, 2D linear pose version
 */
class OdometryFactor2DLinear: public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {

private:
  gtsam::Vector3 measured_; /** measurement  distdot, 0, thetadot */
  typedef OdometryFactor2DLinear This;
  typedef gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> Base;

public:

  OdometryFactor2DLinear() {}  /* Default constructor */

  /**
   * @param betweenMeasured odometry measurement [dx dy dtheta] in body frame
   */
  OdometryFactor2DLinear(gtsam::Key pose1Key, gtsam::Key pose2Key, const gtsam::Vector3& betweenMeasured,
      const gtsam::SharedNoiseModel& model) :
    Base(model, pose1Key, pose2Key), measured_(betweenMeasured) {}

  virtual ~OdometryFactor2DLinear() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  // Calculate error and Jacobians
  gtsam::Vector evaluateError(const gtsam::Vector3& pose1, const gtsam::Vector3& pose2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const {

    using namespace gtsam;

    Vector3 vec_diff = pose2 - pose1;

    Point2 point_diff_local;
    if (H1 || H2) {
      Matrix Hrot, Hp;
      point_diff_local = Rot2(pose1(2)).unrotate(Point2(vec_diff(0), vec_diff(1)), Hrot, Hp);

      // compute Jacobian
      if (H1) *H1 = (Matrix3() << -Hp, Hrot, Matrix12::Zero(), -1).finished();
      if (H2) *H2 = (Matrix3() << Hp, Matrix21::Zero(), Matrix12::Zero(), 1).finished();

    } else {
      point_diff_local = Rot2(pose1(2)).unrotate(Point2(vec_diff(0), vec_diff(1)));

    }
    return (Vector(3) << point_diff_local.x() - measured_(0),
        point_diff_local.y() - measured_(1),
        vec_diff(2) - measured_(2)).finished();

  }


  /** return the measured */
  const gtsam::Vector3& measured() const {
    return measured_;
  }

  /** number of variables attached to this factor */
  size_t size() const {
    return 2;
  }

  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
    const This *e =  dynamic_cast<const This*> (&expected);
    return e != NULL && Base::equals(*e, tol)
        && gtsam::equal_with_abs_tol(this->measured_, e->measured_, tol);
  }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "2-way projected odometry factor" << std::endl;
    Base::print("", keyFormatter);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor2",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
}; // OdometryFactor2DLinear

} // namespace gpslam



/// traits
namespace gtsam {
template<>
struct traits<gpslam::OdometryFactor2DLinear> : public Testable<gpslam::OdometryFactor2DLinear> {};
}


