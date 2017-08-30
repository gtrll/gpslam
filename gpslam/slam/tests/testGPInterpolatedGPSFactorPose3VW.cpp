/**
*  @file testGPInterpolatedGPSFactorPose3VW.cpp
*  @author Jing Dong
*  @date Nov 21, 2015
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

#include <gpslam/slam/GPInterpolatedGPSFactorPose3VW.h>
#include <gpslam/gp/GaussianProcessPriorPose3VW.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpslam;


typedef GPInterpolatedGPSFactorPose3VW GPSFactor;

// error function wrapper for bind 10 params, by ignore default boost::none
inline Vector errorWrapper(const GPSFactor& factor,
    const Pose3& pose1, const Vector3& vel1, const Vector3& omg1,
    const Pose3& pose2, const Vector3& vel2, const Vector3& omg2) {
  return factor.evaluateError(pose1, vel1, omg1, pose2, vel2, omg2);
}

/* ************************************************************************** */
TEST(GPInterpolatedGPSFactorPose3VW, error) {

  noiseModel::Isotropic::shared_ptr model_range =
      noiseModel::Isotropic::Sigma(3, 0.1);
  Matrix Qc = 0.001 * Matrix::Identity(6,6);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);
  double dt = 0.1, tau = 0.04;
  Pose3 body_T_sensor = Pose3(Rot3::Ypr(1.4, 4.4, -0.5), Point3(0.3, 0.6, -0.7));
  Pose3 body_T_sensor_rotonly = Pose3(Rot3::Ypr(1.4, 4.4, -0.5), Point3(0, 0, 0));
  Pose3 p1, p2;
  Vector3 v1, v2, w1, w2;
  Point3 gps_meas;
  GPSFactor factor;
  Vector expect, actual;
  Matrix expectH1, expectH2, expectH3, expectH4, expectH5, expectH6;
  Matrix actualH1, actualH2, actualH3, actualH4, actualH5, actualH6;

  // test at origin, land is in front of z
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  p2 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  v1 = Vector3(0, 0, 0); w1 = Vector3(0, 0, 0);
  v2 = Vector3(0, 0, 0); w2 = Vector3(0, 0, 0);
  gps_meas = Point3(0, 0, 0);
  factor = GPSFactor(gps_meas, model_range, Qc_model, 0, 0, 0, 0, 0, 0, dt, tau);
  actual = factor.evaluateError(p1, v1, w1, p2, v2, w2, actualH1, actualH2,
      actualH3, actualH4, actualH5, actualH6);
  expect = Vector::Zero(3);
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, w1, p2, v2, w2)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, _1, w1, p2, v2, w2)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, p2, v2, w2)), w1, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, _1, v2, w2)), p2, 1e-6);
  expectH5 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, _1, w2)), v2, 1e-6);
  expectH6 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, v2, _1)), w2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));
  EXPECT(assert_equal(expectH6, actualH6, 1e-6));


  // interpolat at origin: forward velocity
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(-0.04, 0.04, 0));
  p2 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0.06, -0.06, 0));
  v1 = Vector3(1, -1, 0); w1 = Vector3(0, 0, 0);
  v2 = Vector3(1, -1, 0); w2 = Vector3(0, 0, 0);
  gps_meas = Point3(0, 0, 0);
  factor = GPSFactor(gps_meas, model_range, Qc_model, 0, 0, 0, 0, 0, 0, dt, tau);
  actual = factor.evaluateError(p1, v1, w1, p2, v2, w2, actualH1, actualH2,
      actualH3, actualH4, actualH5, actualH6);
  expect = Vector::Zero(3);
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, w1, p2, v2, w2)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, _1, w1, p2, v2, w2)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, p2, v2, w2)), w1, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, _1, v2, w2)), p2, 1e-4);
  expectH5 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, _1, w2)), v2, 1e-4);
  expectH6 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, v2, _1)), w2, 1e-4);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));
  EXPECT(assert_equal(expectH6, actualH6, 1e-6));


  // interpolat at origin: rotate
  p1 = Pose3(Rot3::Ypr(-0.04, 0, 0), Point3(0, 0, 0));
  p2 = Pose3(Rot3::Ypr(0.06, 0, 0), Point3(0, 0, 0));
  v1 = Vector3(0, 0, 0); w1 = Vector3(0, 0, 1);
  v2 = Vector3(0, 0, 0); w2 = Vector3(0, 0, 1);
  gps_meas = Point3(0, 0, 0);
  factor = GPSFactor(gps_meas, model_range, Qc_model, 0, 0, 0, 0, 0, 0, dt, tau);
  actual = factor.evaluateError(p1, v1, w1, p2, v2, w2, actualH1, actualH2,
      actualH3, actualH4, actualH5, actualH6);
  expect = Vector::Zero(3);
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, w1, p2, v2, w2)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, _1, w1, p2, v2, w2)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, p2, v2, w2)), w1, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, _1, v2, w2)), p2, 1e-6);
  expectH5 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, _1, w2)), v2, 1e-6);
  expectH6 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, v2, _1)), w2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));
  EXPECT(assert_equal(expectH6, actualH6, 1e-6));


  // forward velocity with body_P_sensor
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(1, 0, 0));
  p2 = Pose3(Rot3::Ypr(0, 0, 0), Point3(2.5, 0.5, 0));
  v1 = Vector3(15, 5, 0); w1 = Vector3(0, 0, 0);
  v2 = Vector3(15, 5, 0); w2 = Vector3(0, 0, 0);
  // ground truth pose
  Pose3 ture_pose = Pose3(Rot3::Ypr(0, 0.0, 0.0), Point3(1.6, 0.2, 0));
  gps_meas = (ture_pose.compose(body_T_sensor)).translation();
  factor = GPSFactor(gps_meas, model_range, Qc_model, 0, 0, 0, 0, 0, 0, dt, tau, body_T_sensor);
  actual = factor.evaluateError(p1, v1, w1, p2, v2, w2, actualH1, actualH2,
      actualH3, actualH4, actualH5, actualH6);
  expect = Vector::Zero(3);
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, w1, p2, v2, w2)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, _1, w1, p2, v2, w2)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, p2, v2, w2)), w1, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, _1, v2, w2)), p2, 1e-4);
  expectH5 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, _1, w2)), v2, 1e-4);
  expectH6 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, v2, _1)), w2, 1e-4);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));
  EXPECT(assert_equal(expectH6, actualH6, 1e-6));

  // forward velocity with body_P_sensor_rotonly
  // test non-zero error
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(1, 0, 0));
  p2 = Pose3(Rot3::Ypr(0, 0, 0), Point3(2.5, 0.5, 0));
  v1 = Vector3(15, 5, 0); w1 = Vector3(0, 0, 0);
  v2 = Vector3(15, 5, 0); w2 = Vector3(0, 0, 0);
  // ground truth pose
  gps_meas = Point3(0, 0, 0);
  factor = GPSFactor(gps_meas, model_range, Qc_model, 0, 0, 0, 0, 0, 0, dt, tau, body_T_sensor_rotonly);
  actual = factor.evaluateError(p1, v1, w1, p2, v2, w2, actualH1, actualH2,
      actualH3, actualH4, actualH5, actualH6);
  expect = Vector3(1.6, 0.2, 0);
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, w1, p2, v2, w2)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, _1, w1, p2, v2, w2)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, p2, v2, w2)), w1, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, _1, v2, w2)), p2, 1e-4);
  expectH5 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, _1, w2)), v2, 1e-4);
  expectH6 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, v2, _1)), w2, 1e-4);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));
  EXPECT(assert_equal(expectH6, actualH6, 1e-6));


  // rotate with body_P_sensor
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  p2 = Pose3(Rot3::Ypr(1.0, 0, 0), Point3(0, 0, 0));
  v1 = Vector3(0, 0, 0); w1 = Vector3(0, 0, 10);
  v2 = Vector3(0, 0, 0); w2 = Vector3(0, 0, 10);
  // ground truth pose
  ture_pose = Pose3(Rot3::Ypr(0.4, 0.0, 0.0), Point3(0, 0, 0));
  gps_meas = (ture_pose.compose(body_T_sensor)).translation();
  factor = GPSFactor(gps_meas, model_range, Qc_model, 0, 0, 0, 0, 0, 0, dt, tau, body_T_sensor);
  actual = factor.evaluateError(p1, v1, w1, p2, v2, w2, actualH1, actualH2,
      actualH3, actualH4, actualH5, actualH6);
  expect = Vector::Zero(3);
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, w1, p2, v2, w2)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, _1, w1, p2, v2, w2)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, p2, v2, w2)), w1, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, _1, v2, w2)), p2, 1e-4);
  expectH5 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, _1, w2)), v2, 1e-4);
  expectH6 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, v2, _1)), w2, 1e-4);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));
  EXPECT(assert_equal(expectH6, actualH6, 1e-6));


  // some random stuff: just for test jacobians
  p1 = Pose3(Rot3::Ypr(0.4, -0.8, 0.2), Point3(3, -8, 2));
  p2 = Pose3(Rot3::Ypr(0.1, 0.3, -0.5), Point3(-9, 3, 4));
  v1 = Vector3(0.1, -0.2, -1.4); w1 = Vector3(0.5, 0.9, 0.7);
  v1 = Vector3(0.6, 0.3, -0.9); w1 = Vector3(0.4, -0.2, 0.8);
  // ground truth pose:
  gps_meas = Point3(0, 0, 0);
  factor = GPSFactor(gps_meas, model_range, Qc_model, 0, 0, 0, 0, 0, 0, dt, tau, body_T_sensor);
  actual = factor.evaluateError(p1, v1, w1, p2, v2, w2, actualH1, actualH2,
      actualH3, actualH4, actualH5, actualH6);
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, w1, p2, v2, w2)), p1, 1e-8);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, _1, w1, p2, v2, w2)), v1, 1e-8);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, p2, v2, w2)), w1, 1e-8);
  expectH4 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, _1, v2, w2)), p2, 1e-8);
  expectH5 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, _1, w2)), v2, 1e-8);
  expectH6 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, w1, p2, v2, _1)), w2, 1e-6);
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));
  EXPECT(assert_equal(expectH6, actualH6, 1e-6));

}

/* ************************************************************************** */
TEST(GPInterpolatedGPSFactorPose3VW, optimization) {
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

  noiseModel::Isotropic::shared_ptr model_prior =  noiseModel::Isotropic::Sigma(6, 0.1);
  noiseModel::Isotropic::shared_ptr model_prior_loss =  noiseModel::Isotropic::Sigma(6, 100);
  noiseModel::Isotropic::shared_ptr model_gps =  noiseModel::Isotropic::Sigma(3, 0.01);
  double delta_t = 0.1, tau1 = -0.1, tau2 = 0.05, tau3 = 0.2;
  Matrix Qc = 0.01 * Matrix::Identity(6,6);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);

  // ground truth
  Pose3 p1(Rot3(), Point3(0,0,0)), p2(Rot3(), Point3(1,0,0));
  Pose3 pcam1(Rot3(), Point3(-1,0,0)), pcam2(Rot3(), Point3(0.5,0,0)),
      pcam3(Rot3(), Point3(2,0,0));
  Vector3 v1 = Vector3(10, 0, 0), w1 = Vector3(0, 0, 0);
  Vector3 v2 = Vector3(10, 0, 0), w2 = Vector3(0, 0, 0);

  // init values
  Pose3 p1i(Rot3::Ypr(0.1, -0.1, -0.1), Point3(0.1, 0.1, -0.1));
  Pose3 p2i(Rot3::Ypr(-0.1, 0.1, -0.1), Point3(1.1, -0.1, 0.1));
  Vector3 v1i = Vector3(9.8, -0.1, -0.05), w1i = Vector3(0.1, -0.1, 0.1);
  Vector3 v2i = Vector3(10.2, 0.03, -0.1), w2i = Vector3(-0.1, 0.1, 0.1);

  // landmark and measurements
  Point3 meas1 = pcam1.translation();
  Point3 meas2 = pcam2.translation();
  Point3 meas3 = pcam3.translation();

  NonlinearFactorGraph graph;
  graph.add(PriorFactor<Pose3>(Symbol('x', 1), p1, model_prior));
  graph.add(PriorFactor<Pose3>(Symbol('x', 2), p2, model_prior));

  graph.add(GaussianProcessPriorPose3VW(Symbol('x', 1), Symbol('v', 1), Symbol('w', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('w', 2), delta_t, Qc_model));
  graph.add(GPSFactor(meas1, model_gps, Qc_model, Symbol('x', 1), Symbol('v', 1), Symbol('w', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('w', 2), delta_t, tau1));
  graph.add(GPSFactor(meas2, model_gps, Qc_model, Symbol('x', 1), Symbol('v', 1), Symbol('w', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('w', 2), delta_t, tau2));
  graph.add(GPSFactor(meas3, model_gps, Qc_model, Symbol('x', 1), Symbol('v', 1), Symbol('w', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('w', 2), delta_t, tau3));

  Values init_values;
  init_values.insert(Symbol('x', 1), p1i);
  init_values.insert(Symbol('v', 1), v1i);
  init_values.insert(Symbol('w', 1), w1i);
  init_values.insert(Symbol('x', 2), p2i);
  init_values.insert(Symbol('v', 2), v2i);
  init_values.insert(Symbol('w', 2), w2i);

  GaussNewtonParams parameters;
  parameters.setVerbosity("ERROR");
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values values = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-6);
  EXPECT(assert_equal(p1, values.at<Pose3>(Symbol('x', 1)), 1e-6));
  EXPECT(assert_equal(p2, values.at<Pose3>(Symbol('x', 2)), 1e-6));
  EXPECT(assert_equal(v1, values.at<Vector3>(Symbol('v', 1)), 1e-6));
  EXPECT(assert_equal(v2, values.at<Vector3>(Symbol('v', 2)), 1e-6));
  EXPECT(assert_equal(w1, values.at<Vector3>(Symbol('w', 1)), 1e-6));
  EXPECT(assert_equal(w2, values.at<Vector3>(Symbol('w', 2)), 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
