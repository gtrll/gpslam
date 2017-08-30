/**
 *  @file  RangeBearingFactor2DLinear.h
 *  @brief range + bearing factor for 2D linear pose
 *  @author Xinyan Yan, Jing Dong
 *  @date  Nov 3, 2015
 **/

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Vector.h>

namespace gpslam {

/**
 * range and bearing measurement, 2D linear pose version
 */
class RangeBearingFactor2DLinear: public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Point2> {
private:

  // measurements
  double range_;
  gtsam::Rot2 bearing_;

  typedef RangeBearingFactor2DLinear This;
  typedef gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Point2> Base;

public:

  RangeBearingFactor2DLinear() {} /* Default constructor */

  RangeBearingFactor2DLinear(gtsam::Key poseKey, gtsam::Key pointKey,
      double range, const gtsam::Rot2& bearing,
      const gtsam::SharedNoiseModel& model) :
        Base(model, poseKey, pointKey), range_(range), bearing_(bearing) {
  }

  virtual ~RangeBearingFactor2DLinear() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** h(x)-z */
  gtsam::Vector evaluateError(const gtsam::Vector3& pose, const gtsam::Point2& point,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const {

    using namespace gtsam;

    Pose2 pose2(pose(0), pose(1), pose(2));
    Point2 rel_point = pose2.transform_to(point);
    Rot2 expect_rot = Rot2::atan2(rel_point.y(), rel_point.x());
    Matrix12 Hnorm;
    double expect_d = Point2(point - pose2.t()).norm(Hnorm);

    // jacobians
    if (H1 || H2) {
      Matrix12 tmp;
      if (expect_d > 1e-5) {
        double expect_d2 = expect_d * expect_d;
        tmp = (Matrix12() << -rel_point.y()/expect_d2, rel_point.x()/expect_d2).finished();
      } else {
        tmp = Matrix12::Zero();
      }
      Matrix13 H11, H21;
      Matrix12 H12, H22;
      if (H1) {
        Vector2 t(rel_point.y(), -rel_point.x());
        H11 = tmp * (Matrix23() << -pose2.r().transpose(), t).finished();
        H21 = (Matrix13() << -Hnorm, 0.0).finished();
        *H1 = (Matrix23() << H11, H21).finished();
      }
      if (H2) {
        H12 = tmp * pose2.r().transpose();
        H22 = Hnorm;
        *H2 = (Matrix2() << H12, H22).finished();
      }
    }

    return (Vector(2) << Rot2::Logmap(bearing_.between(expect_rot)), expect_d - range_).finished();
  }

  /** return the measured */
  double range() const {
    return range_;
  }
  const gtsam::Rot2& bearing() const {
    return bearing_;
  }

  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
    const This *e = dynamic_cast<const This*> (&expected);
    return e != NULL && Base::equals(*e, tol)
        && fabs(this->range_ - e->range_) < tol
        && this->bearing_.equals(e->bearing_, tol);
  }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "RangeBearingFactor, range = " << range_
        << ", bearing = "; bearing_.print();
    Base::print("", keyFormatter);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(range_);
    ar & BOOST_SERIALIZATION_NVP(bearing_);
  }

}; // RangeBearingFactor2DLinear

} // namespace gpslam


/// traits
namespace gtsam {
template<>
struct traits<gpslam::RangeBearingFactor2DLinear> : public Testable<gpslam::RangeBearingFactor2DLinear> {};
}

