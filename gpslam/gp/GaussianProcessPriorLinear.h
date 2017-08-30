/**
 *  @file  GaussianProcessPriorLinear.h
 *  @brief Linear GP prior, see Barfoot14rss
 *  @author Xinyan Yan, Jing Dong
 **/


#pragma once

#include <gpslam/gp/GPutils.h>

#include <boost/lexical_cast.hpp>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>


namespace gpslam {

/**
 * 4-way factor for Gaussian Process prior factor, linear version
 */
template <int Dim>
class GaussianProcessPriorLinear: public gtsam::NoiseModelFactor4<
    Eigen::Matrix<double, Dim, 1>, Eigen::Matrix<double, Dim, 1>,
    Eigen::Matrix<double, Dim, 1>, Eigen::Matrix<double, Dim, 1> > {

private:
  double delta_t_;

  typedef Eigen::Matrix<double, Dim, 1> VectorDim;
  typedef Eigen::Matrix<double, 2*Dim, 1> Vector2Dim;
  typedef Eigen::Matrix<double, Dim, Dim> MatrixDim;
  typedef Eigen::Matrix<double, 2*Dim, Dim> Matrix21Dim;
  typedef Eigen::Matrix<double, 2*Dim, 2*Dim> Matrix2Dim;
  typedef GaussianProcessPriorLinear<Dim> This;
  typedef gtsam::NoiseModelFactor4<VectorDim, VectorDim, VectorDim, VectorDim> Base;

public:

  GaussianProcessPriorLinear() {}	/* Default constructor only for serialization */

  /// Constructor
  /// @param delta_t is the time between the two states
  GaussianProcessPriorLinear(gtsam::Key poseKey1, gtsam::Key velKey1,
      gtsam::Key poseKey2, gtsam::Key velKey2,
      double delta_t, const gtsam::SharedNoiseModel& Qc_model) :
        Base(gtsam::noiseModel::Gaussian::Covariance(calcQ<Dim>(getQc(Qc_model), delta_t)),
        poseKey1, velKey1, poseKey2, velKey2) {
    delta_t_ = delta_t;
  }

  virtual ~GaussianProcessPriorLinear() {}


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }


  /// factor error function
  gtsam::Vector evaluateError(
      const VectorDim& pose1, const VectorDim& vel1,
      const VectorDim& pose2, const VectorDim& vel2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none,
      boost::optional<gtsam::Matrix&> H4 = boost::none) const {

    // state vector
    Vector2Dim x1 = (Vector2Dim() << pose1, vel1).finished();
    Vector2Dim x2 = (Vector2Dim() << pose2, vel2).finished();

    // Jacobians
    if (H1) *H1 = (Matrix21Dim() << MatrixDim::Identity(),            MatrixDim::Zero()).finished();
    if (H2) *H2 = (Matrix21Dim() << delta_t_ * MatrixDim::Identity(), MatrixDim::Identity()).finished();
    if (H3) *H3 = (Matrix21Dim() << -1.0 * MatrixDim::Identity(),     MatrixDim::Zero()).finished();
    if (H4) *H4 = (Matrix21Dim() << MatrixDim::Zero(),                -1.0 * MatrixDim::Identity()).finished();

    // transition matrix & error
    return calcPhi<Dim>(delta_t_) * x1 - x2;
  }


  /** demensions */
  size_t dim() const { return Dim; }

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
    std::cout << s << "4-way Gaussian Process Factor Linear<" << Dim << ">" << std::endl;
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

}; // GaussianProcessPriorLinear


} // namespace gpslam



/// traits
namespace gtsam {
template<int Dim>
struct traits<gpslam::GaussianProcessPriorLinear<Dim> > : public Testable<
    gpslam::GaussianProcessPriorLinear<Dim> > {};
}
