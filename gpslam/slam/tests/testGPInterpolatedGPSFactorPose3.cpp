/**
*  @file testGPInterpolatedGPSFactorPose3.cpp
*  @author Jing Dong
*  @date Oct 4, 2015
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <gpslam/slam/GPInterpolatedGPSFactorPose3.h>
#include <gpslam/gp/GaussianProcessPriorPose3.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpslam;


typedef GPInterpolatedGPSFactorPose3 GPSFactor;

// error function wrapper for bind 10 params, by ignore default boost::none
inline Vector errorWrapper(const GPSFactor& factor,
    const Pose3& pose1, const Vector6& vel1,
    const Pose3& pose2, const Vector6& vel2) {
  return factor.evaluateError(pose1, vel1, pose2, vel2);
}

/* ************************************************************************** */
TEST(GPInterpolatedGPSFactorPose3, error) {

  noiseModel::Isotropic::shared_ptr model_range =
      noiseModel::Isotropic::Sigma(3, 0.1);
  Matrix Qc = 0.001 * Matrix::Identity(6,6);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);
  double dt = 0.1, tau = 0.04;
  Pose3 body_T_sensor = Pose3(Rot3::Ypr(1.0, 0.4, 0.5), Point3(0.3, 0.6, -0.7));
  Pose3 p1, p2;
  Vector6 v1, v2;
  Point3 gps_meas;
  GPSFactor factor;
  Vector expect, actual;
  Matrix expectH1, expectH2, expectH3, expectH4;
  Matrix actualH1, actualH2, actualH3, actualH4;

  // test at origin, land is in front of z
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  p2 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  v1 = (Vector6() << 0, 0, 0, 0, 0, 0).finished();
  v2 = (Vector6() << 0, 0, 0, 0, 0, 0).finished();
  gps_meas = Point3(0, 0, 0);
  factor = GPSFactor(gps_meas, model_range, Qc_model, 0, 0, 0, 0, dt, tau);
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2,
      actualH3, actualH4);
  expect = Vector::Zero(3);
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));


  // interpolat at origin: forward velocity
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(-0.04, 0, 0));
  p2 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0.06, 0, 0));
  v1 = (Vector6() << 0, 0, 0, 1, 0, 0).finished();
  v2 = (Vector6() << 0, 0, 0, 1, 0, 0).finished();
  gps_meas = Point3(0, 0, 0);
  factor = GPSFactor(gps_meas, model_range, Qc_model, 0, 0, 0, 0, dt, tau);
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2,
      actualH3, actualH4);
  expect = Vector::Zero(3);
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2)), p2, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1)), v2, 1e-4);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));


  // interpolat at origin: rotate
  p1 = Pose3(Rot3::Ypr(-0.04, 0, 0), Point3(0, 0, 0));
  p2 = Pose3(Rot3::Ypr(0.06, 0, 0), Point3(0, 0, 0));
  v1 = (Vector6() << 0, 0, 1, 0, 0, 0).finished();
  v2 = (Vector6() << 0, 0, 1, 0, 0, 0).finished();
  gps_meas = Point3(0, 0, 0);
  factor = GPSFactor(gps_meas, model_range, Qc_model, 0, 0, 0, 0, dt, tau);
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2,
      actualH3, actualH4);
  expect = Vector::Zero(3);
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2)), p2, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1)), v2, 1e-4);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));


  // forward velocity with body_P_sensor
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  p2 = Pose3(Rot3::Ypr(0, 0, 0), Point3(1.5, 0, 0));
  v1 = (Vector6() << 0, 0, 0, 15, 0, 0).finished();
  v2 = (Vector6() << 0, 0, 0, 15, 0, 0).finished();
  // ground truth pose:
  Pose3 ture_pose = Pose3(Rot3::Ypr(0, 0, 0), Point3(0.6, 0, 0));
  gps_meas = (ture_pose.compose(body_T_sensor)).translation();
  factor = GPSFactor(gps_meas, model_range, Qc_model, 0, 0, 0, 0, dt, tau, body_T_sensor);
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2,
      actualH3, actualH4);
  expect = Vector::Zero(3);
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2)), p2, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1)), v2, 1e-4);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));


  // some random stuff: just for test jacobians
  p1 = Pose3(Rot3::Ypr(1.3, 2.4, 1.2), Point3(0.2, 0.3, 0.4));
  p2 = Pose3(Rot3::Ypr(0.5, 6.5, 1.1), Point3(1.5, 0.7, 0.5));
  v1 = (Vector6() << 1.0, 2.0, 0.4, 15, 0.3, 0.2).finished();
  v2 = (Vector6() << 2.0, 0.2, 0.1, 17, 0.4, 0.7).finished();
  // ground truth pose:
  gps_meas = Point3(0, 0, 0);
  factor = GPSFactor(gps_meas, model_range, Qc_model, 0, 0, 0, 0, dt, tau, body_T_sensor);
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2,
      actualH3, actualH4);
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2)), p2, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1)), v2, 1e-4);
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
}

/* ************************************************************************** */
TEST(GPInterpolatedGPSFactorPose3, optimization) {
  /**
   * A simple graph:
   *
   *  gps1,2,3
   *  /  \
   * x1   x2
   *  \  /
   *   gp
   *  /  \
   * v1  v2
   * |   |
   * pv1 pv2
   *
   * p1 and p2 are constrained by GP prior and measurement, velcity is known.
   * 3 measurements are given in different places (on a line)
   */

  noiseModel::Isotropic::shared_ptr model_prior =  noiseModel::Isotropic::Sigma(6, 0.01);
  noiseModel::Isotropic::shared_ptr model_prior_loss =  noiseModel::Isotropic::Sigma(6, 100);
  noiseModel::Isotropic::shared_ptr model_gps =  noiseModel::Isotropic::Sigma(3, 0.1);
  double delta_t = 0.1, tau1 = -0.1, tau2 = 0.05, tau3 = 0.2;
  Matrix Qc = 0.01 * Matrix::Identity(6,6);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);

  // ground truth
  Pose3 p1(Rot3(), Point3(0,0,0)), p2(Rot3(), Point3(1,0,0));
  Pose3 pcam1(Rot3(), Point3(-1,0,0)), pcam2(Rot3(), Point3(0.5,0,0)),
      pcam3(Rot3(), Point3(2,0,0));
  Vector6 v1 = (Vector6() << 0, 0, 0, 10, 0, 0).finished();
  Vector6 v2 = (Vector6() << 0, 0, 0, 10, 0, 0).finished();

  // init values
  Pose3 p1i(Rot3::Ypr(0.1, 0.1, -0.1), Point3(0.04, 0.1, -0.06));
  Pose3 p2i(Rot3::Ypr(-0.1, 0.1, -0.1), Point3(1.05, -0.1, 0.1));
  Vector6 v1i = (Vector6() << -0.1, 0, 0, 9.8, 0, 0.2).finished();
  Vector6 v2i = (Vector6() << 0, 0, 0.2, 9.7, 0, -0.1).finished();

  // landmark and measurements
  Point3 meas1 = pcam1.translation();
  Point3 meas2 = pcam2.translation();
  Point3 meas3 = pcam3.translation();

  NonlinearFactorGraph graph;
  graph.add(PriorFactor<Pose3>(Symbol('x', 1), p1, model_prior_loss));
  //graph.add(PriorFactor<Pose3>(Symbol('x', 2), p2, model_prior_loss));
  graph.add(PriorFactor<Vector6>(Symbol('v', 1), v1, model_prior));
  graph.add(PriorFactor<Vector6>(Symbol('v', 2), v2, model_prior));
  graph.add(GaussianProcessPriorPose3(Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), delta_t, Qc_model));
  graph.add(GPSFactor(meas1, model_gps, Qc_model, Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), delta_t, tau1));
  graph.add(GPSFactor(meas2, model_gps, Qc_model, Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), delta_t, tau2));
  graph.add(GPSFactor(meas3, model_gps, Qc_model, Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), delta_t, tau3));

  Values init_values;
  init_values.insert(Symbol('x', 1), p1i);
  init_values.insert(Symbol('v', 1), v1i);
  init_values.insert(Symbol('x', 2), p2i);
  init_values.insert(Symbol('v', 2), v2i);

  GaussNewtonParams parameters;
  parameters.setVerbosity("ERROR");
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values values = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-6);
  EXPECT(assert_equal(p1, values.at<Pose3>(Symbol('x', 1)), 1e-6));
  EXPECT(assert_equal(p2, values.at<Pose3>(Symbol('x', 2)), 1e-6));
  EXPECT(assert_equal(v1, values.at<Vector6>(Symbol('v', 1)), 1e-6));
  EXPECT(assert_equal(v1, values.at<Vector6>(Symbol('v', 2)), 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
