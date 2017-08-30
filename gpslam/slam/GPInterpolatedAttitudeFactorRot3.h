/**
 *  @file  GPInterpolatedAttitudeFactorRot3.h
 *  @author Frank Dellaert, Jing Dong
 *  @brief Use Acceleration to get Attitude, GP interpolated version
 **/

#pragma once

#include <gpslam/gp/GaussianProcessInterpolatorRot3.h>

#include <gtsam/navigation/AttitudeFactor.h>


namespace gpslam {

/**
 * Binary factor for an attitude measurement, GP interpolated
 */
class GPInterpolatedAttitudeFactorRot3: public gtsam::NoiseModelFactor4<
    gtsam::Rot3, gtsam::Vector3, gtsam::Rot3, gtsam::Vector3>, public gtsam::AttitudeFactor {

private:
  // interpolater
  GaussianProcessInterpolatorRot3 GPbase_;

  typedef GPInterpolatedAttitudeFactorRot3 This;
  typedef gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector3, gtsam::Rot3, gtsam::Vector3> Base;
  typedef GaussianProcessInterpolatorRot3 GPBase;


public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  GPInterpolatedAttitudeFactorRot3() {} /* Default constructor */

  /**
   * Constructor
   * @param nZ measured direction in navigation frame
   * @param meas_model Gaussian noise model
   * @param bRef reference direction in body frame (default Z-axis)
   */
  GPInterpolatedAttitudeFactorRot3(
      gtsam::Key poseKey1, gtsam::Key velKey1, gtsam::Key poseKey2, gtsam::Key velKey2,
      double delta_t, double tau, const gtsam::SharedNoiseModel& Qc_model,
      const gtsam::SharedNoiseModel& meas_model, const gtsam::Unit3& nZ,
      const gtsam::Unit3& bRef = gtsam::Unit3(0, 0, 1)) :
        Base(meas_model, poseKey1, velKey1, poseKey2, velKey2),
        gtsam::AttitudeFactor(nZ, bRef), GPbase_(Qc_model, delta_t, tau) {
  }

  virtual ~GPInterpolatedAttitudeFactorRot3() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** factor error */
  gtsam::Vector evaluateError(const gtsam::Rot3& pose1, const gtsam::Vector3& vel1,
      const gtsam::Rot3& pose2, const gtsam::Vector3& vel2,
      boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none) const {

    using namespace gtsam;

    // interpolate pose
    if (H1 || H2 || H3 || H4) {
      Matrix Hint1, Hint2, Hint3, Hint4;
      Rot3 pose = GPbase_.interpolatePose(pose1, vel1, pose2, vel2, Hint1, Hint2, Hint3, Hint4);

      Matrix Hrot;
      Vector err = attitudeError(pose, Hrot);
      GPBase::updatePoseJacobians(Hrot, Hint1, Hint2, Hint3, Hint4, H1, H2, H3, H4);

      return err;

    } else {
      Rot3 pose = GPbase_.interpolatePose(pose1, vel1, pose2, vel2);
      return attitudeError(pose);
    }
  }


  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
    const This *e = dynamic_cast<const This*> (&expected);
    return e != NULL && Base::equals(*e, tol)
        && this->GPbase_.equals(e->GPbase_);
  }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "GP Interpolated AttitudeFactor" << std::endl;
    Base::print("", keyFormatter);
    GPbase_.print("Measurement interpolated at: ");
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor4",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(GPbase_);
  }

}; // GPInterpolatedAttitudeFactorRot3

} // namespace gpslam


/// traits
namespace gtsam {
template<>
struct traits<gpslam::GPInterpolatedAttitudeFactorRot3> : public Testable<gpslam::GPInterpolatedAttitudeFactorRot3> {};
}

