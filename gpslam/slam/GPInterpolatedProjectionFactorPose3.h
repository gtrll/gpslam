/**
 * @file GPInterpolatedProjectionFactorPose3.h
 * @brief GP Interpolation projection measurement factor for Pose3
 * @author Chris Beall, Richard Roberts, Frank Dellaert, Alex Cunningham
 * @author Jing Dong, Xinyan Yan
 */

#pragma once

#include <gpslam/gp/GaussianProcessInterpolatorPose3.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>


namespace gpslam {

/**
 * Non-linear factor for a constraint derived from a 2D measurement. The calibration is known here.
 * GP interpolated version
 */

template <class CALIBRATION = gtsam::Cal3_S2>
class GPInterpolatedProjectionFactorPose3: public gtsam::NoiseModelFactor5<gtsam::Pose3,
    gtsam::Vector6, gtsam::Pose3, gtsam::Vector6, gtsam::Point3> {

protected:
  // interpolater
  GaussianProcessInterpolatorPose3 GPbase_;

  // Keep a copy of measurement and calibration for I/O
  gtsam::Point2 measured_;	                        ///< 2D measurement
  boost::shared_ptr<CALIBRATION> K_;        ///< shared pointer to calibration object
  boost::optional<gtsam::Pose3> body_P_sensor_;    ///< The pose of the sensor in the body frame

  // verbosity handling for Cheirality Exceptions
  bool throwCheirality_;    ///< If true, rethrows Cheirality exceptions (default: false)
  bool verboseCheirality_;  ///< If true, prints text for Cheirality exceptions (default: false)

  // typedefs
  typedef GPInterpolatedProjectionFactorPose3<CALIBRATION> This;
  typedef gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Vector6, gtsam::Pose3,
      gtsam::Vector6, gtsam::Point3> Base;
  typedef GaussianProcessInterpolatorPose3 GPBase;

public:


  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor: do nothing
  GPInterpolatedProjectionFactorPose3() {}

  /**
   * Constructor
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param meas_model is the standard deviation
   * @param pointKey is the index of the landmark
   * @param K shared pointer to the constant calibration
   */
  GPInterpolatedProjectionFactorPose3(const gtsam::Point2& measured,
      const gtsam::SharedNoiseModel& cam_model, const gtsam::SharedNoiseModel& Qc_model,
      gtsam::Key poseKey1, gtsam::Key velKey1, gtsam::Key poseKey2, gtsam::Key velKey2,
      gtsam::Key pointKey,
      double delta_t, double tau, const boost::shared_ptr<CALIBRATION>& K,
      boost::optional<gtsam::Pose3> body_P_sensor = boost::none,
      bool throwCheirality = false, bool verboseCheirality = false) :

        Base(cam_model, poseKey1, velKey1, poseKey2, velKey2, pointKey),
        GPbase_(Qc_model, delta_t, tau), measured_(measured), K_(K),
        body_P_sensor_(body_P_sensor), throwCheirality_(throwCheirality),
        verboseCheirality_(verboseCheirality){}

  /** Virtual destructor */
  virtual ~GPInterpolatedProjectionFactorPose3() {}


  /// Evaluate error h(x)-z and optionally derivatives
  gtsam::Vector evaluateError(const gtsam::Pose3& pose1, const gtsam::Vector6& vel1,
      const gtsam::Pose3& pose2, const gtsam::Vector6& vel2, const gtsam::Point3& point,
      boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none,
      boost::optional<gtsam::Matrix&> H5 = boost::none) const {

    using namespace gtsam;

    // interpolate pose
    Matrix Hint1, Hint2, Hint3, Hint4;
    Pose3 pose;
    if (H1 || H2 || H3 || H4)
      pose = GPbase_.interpolatePose(pose1, vel1, pose2, vel2, Hint1, Hint2, Hint3, Hint4);
    else
      pose = GPbase_.interpolatePose(pose1, vel1, pose2, vel2);

    // jacobian of pose
    Matrix Hpose;

    try {

      if (body_P_sensor_) {
        if (H1 || H2 || H3 || H4) {
          Matrix H0;
          PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_, H0), *K_);
          Point2 reprojectionError(camera.project(point, Hpose, H5) - measured_);
          Hpose = Hpose * H0;
          GPBase::updatePoseJacobians(Hpose, Hint1, Hint2, Hint3, Hint4, H1, H2, H3, H4);
          return reprojectionError.vector();
        } else {
          PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_), *K_);
          Point2 reprojectionError(camera.project(point, Hpose, H5) - measured_);
          return reprojectionError.vector();
        }
      } else {
        PinholeCamera<CALIBRATION> camera(pose, *K_);
        Point2 reprojectionError(camera.project(point, Hpose, H5) - measured_);
        GPBase::updatePoseJacobians(Hpose, Hint1, Hint2, Hint3, Hint4, H1, H2, H3, H4);
        return reprojectionError.vector();
      }

    } catch( CheiralityException& e) {
      // catch CheiralityException
      if (H1) *H1 = Matrix::Zero(2,6);
      if (H2) *H2 = Matrix::Zero(2,6);
      if (H3) *H3 = Matrix::Zero(2,6);
      if (H4) *H4 = Matrix::Zero(2,6);
      if (H5) *H5 = Matrix::Zero(2,3);
      if (verboseCheirality_)
        std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
            " moved behind camera " << DefaultKeyFormatter(this->key1()) << std::endl;
      if (throwCheirality_)
        throw e;
    }

    // no throw case
    return Vector::Ones(2) * 2.0 * K_->fx();
  }

  /** return the measurement */
  const gtsam::Point2& measured() const {
    return measured_;
  }

  /** return the calibration object */
  inline const boost::shared_ptr<CALIBRATION> calibration() const {
    return K_;
  }


  /**
   * Testables
   */

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "GPInterpolatedProjectionFactor, z = "; measured_.print();
    Base::print("", keyFormatter);
    GPbase_.print("Measurement taken from: ");
  }

  /// equals
  virtual bool equals(const gtsam::NonlinearFactor& p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol)
        && GPbase_.equals(e->GPbase_)
        && this->measured_.equals(e->measured_, tol)
        && this->K_->equals(*e->K_, tol)
        && this->throwCheirality_ == e->throwCheirality_
        && this->verboseCheirality_ == e->verboseCheirality_;
  }


private:
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(GPbase_);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(K_);
    ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
    ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
  }
};


} // \ namespace gpslam


/// traits
namespace gtsam {
template<class CALIBRATION>
struct traits<gpslam::GPInterpolatedProjectionFactorPose3<CALIBRATION> > :
    public Testable<gpslam::GPInterpolatedProjectionFactorPose3<CALIBRATION> > {};
}


