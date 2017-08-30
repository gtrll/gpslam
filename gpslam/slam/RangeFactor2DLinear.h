/**
 *  @file  RangeFactor2DLinear.h
 *  @brief range factor for 2D linear pose
 *  @author Xinyan Yan, Jing Dong
 **/

#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Vector.h>


namespace gpslam {

/**
 * range measurement, 2D linear pose version
 */
class RangeFactor2DLinear: public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Point2> {
private:

  double measured_; /** measurement */
  typedef RangeFactor2DLinear This;
  typedef gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Point2> Base;

public:

  RangeFactor2DLinear() {} /* Default constructor */

  RangeFactor2DLinear(gtsam::Key poseKey, gtsam::Key pointKey, double measured,
      const gtsam::SharedNoiseModel& model) :
        Base(model, poseKey, pointKey), measured_(measured) {
  }

  virtual ~RangeFactor2DLinear() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** h(x)-z */
  gtsam::Vector evaluateError(const gtsam::Vector3& pose, const gtsam::Point2& point,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const {

    using namespace gtsam;

    Point2 d = point - Point2(pose(0), pose(1));
    Matrix12 H;
    double r = d.norm(H);

    if(H1) *H1 = (Matrix13() << -H, 0.0).finished();
    if(H2) *H2 = H;
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

}; // RangeFactor2DLinear

} // namespace gpslam


/// traits
namespace gtsam {
template<>
struct traits<gpslam::RangeFactor2DLinear> : public Testable<gpslam::RangeFactor2DLinear> {};
}

