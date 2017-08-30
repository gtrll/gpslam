/**
 * @file GaussianProcessInterpolatorLinear.h
 * @brief Base and utils for Gaussian Process Interpolated measurement factor, linear version
 * @author Jing Dong, Xinyan Yan
 * @date Oct 26, 2015
 */

#pragma once

#include <gpslam/gp/GPutils.h>

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Matrix.h>

#include <boost/serialization/array.hpp>


namespace gpslam {

/**
 * 4-way factor for Gaussian Process interpolator, linear version
 * interpolate pose and velocity given consecutive poses and velocities
 */
template <int Dim>
class GaussianProcessInterpolatorLinear {

private:
  typedef GaussianProcessInterpolatorLinear<Dim> This;
  typedef Eigen::Matrix<double, Dim, 1> VectorDim;
  typedef Eigen::Matrix<double, 2*Dim, 1> Vector2Dim;
  typedef Eigen::Matrix<double, Dim, Dim> MatrixDim;
  typedef Eigen::Matrix<double, 2*Dim, 2*Dim> Matrix2Dim;

  double delta_t_;		// t_{i+1} - t_i
  double tau_;			// tau - t_i. we use tau as time interval from t_i instead of from t_0 as in Barfoot papers

  MatrixDim Qc_;
  Matrix2Dim Lambda_;
  Matrix2Dim Psi_;

public:

  /// Default constructor: only for serialization
  GaussianProcessInterpolatorLinear() {}

  /**
   * Constructor
   * @param Qc noise model of Qc
   * @param delta_t the time between the two states
   * @param tau the time of interval status
   */
  GaussianProcessInterpolatorLinear(const gtsam::SharedNoiseModel& Qc_model,
      double delta_t, double tau) :
      delta_t_(delta_t), tau_(tau) {

    // check noise model dim
    assert(Qc_model->dim() == Dim);

    // Calcuate Lambda and Psi
    Qc_ = getQc(Qc_model);
    Lambda_ = calcLambda(Qc_, delta_t_, tau_);
    Psi_ = calcPsi(Qc_, delta_t_, tau_);
  }

  /** Virtual destructor */
  virtual ~GaussianProcessInterpolatorLinear() {}


  /// interpolate pose with Jacobians
  VectorDim interpolatePose(
      const VectorDim& pose1, const VectorDim& vel1,
      const VectorDim& pose2, const VectorDim& vel2,
      gtsam::OptionalJacobian<Dim, Dim> H1 = boost::none,
      gtsam::OptionalJacobian<Dim, Dim> H2 = boost::none,
      gtsam::OptionalJacobian<Dim, Dim> H3 = boost::none,
      gtsam::OptionalJacobian<Dim, Dim> H4 = boost::none) const {

    // state vector
    Vector2Dim x1 = (Vector2Dim() << pose1, vel1).finished();
    Vector2Dim x2 = (Vector2Dim() << pose2, vel2).finished();

    // jacobians
    if (H1) *H1 = Lambda_.template block<Dim,Dim>(0, 0);
    if (H2) *H2 = Lambda_.template block<Dim,Dim>(0, Dim);
    if (H3) *H3 = Psi_.template block<Dim,Dim>(0, 0);
    if (H4) *H4 = Psi_.template block<Dim,Dim>(0, Dim);

    // interpolate pose (just calculate upper part of the interpolated state vector to save time)
    return Lambda_.template block<Dim,2*Dim>(0, 0) * x1 + Psi_.template block<Dim,2*Dim>(0, 0) * x2;
  }


  /// update jacobian based on interpolated jacobians
  static void updatePoseJacobians(const gtsam::Matrix& Hpose,  const gtsam::Matrix& Hint1,
      const gtsam::Matrix& Hint2, const gtsam::Matrix& Hint3, const gtsam::Matrix& Hint4,
      boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
      boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4) {
    if (H1) *H1 = Hpose * Hint1;
    if (H2) *H2 = Hpose * Hint2;
    if (H3) *H3 = Hpose * Hint3;
    if (H4) *H4 = Hpose * Hint4;
  }


  /// interpolate velocity with Jacobians
  VectorDim interpolateVelocity(
      const VectorDim& pose1, const VectorDim& vel1,
      const VectorDim& pose2, const VectorDim& vel2,
      gtsam::OptionalJacobian<Dim, Dim> H1 = boost::none,
      gtsam::OptionalJacobian<Dim, Dim> H2 = boost::none,
      gtsam::OptionalJacobian<Dim, Dim> H3 = boost::none,
      gtsam::OptionalJacobian<Dim, Dim> H4 = boost::none) const {

    // state vector
    Vector2Dim x1 = (Vector2Dim() << pose1, vel1).finished();
    Vector2Dim x2 = (Vector2Dim() << pose2, vel2).finished();

    // jacobians
    if (H1) *H1 = Lambda_.template block<Dim,Dim>(Dim, 0);
    if (H2) *H2 = Lambda_.template block<Dim,Dim>(Dim, Dim);
    if (H3) *H3 = Psi_.template block<Dim,Dim>(Dim, 0);
    if (H4) *H4 = Psi_.template block<Dim,Dim>(Dim, Dim);

    // interpolate pose (just calculate lower part of the interpolated state vector to save time)
    return Lambda_.template block<Dim,2*Dim>(Dim, 0) * x1 + Psi_.template block<Dim,2*Dim>(Dim, 0) * x2;
  }

  /** demensions */
  size_t dim() const { return Dim; }


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
    std::cout << s << "GaussianProcessInterpolatorLinear<" << Dim << ">" << std::endl;
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
template<int Dim>
struct traits<gpslam::GaussianProcessInterpolatorLinear<Dim> > : public Testable<
    gpslam::GaussianProcessInterpolatorLinear<Dim> > {};
}

