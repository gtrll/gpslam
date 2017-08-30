/**
*  @file testGPInterpolatedRangeFactorPose3.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <gpslam/slam/GPInterpolatedRangeFactorPose3.h>
#include <gpslam/gp/GaussianProcessPriorPose3.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpslam;


typedef GPInterpolatedRangeFactorPose3 RangeFactor;

// error function wrapper for bind 10 params, by ignore default boost::none
inline Vector errorWrapper(const RangeFactor& factor,
    const Pose3& pose1, const Vector6& vel1,
    const Pose3& pose2, const Vector6& vel2, const Point3& point) {
  return factor.evaluateError(pose1, vel1, pose2, vel2, point);
}

/* ************************************************************************** */
TEST(GPInterpolatedRangeFactorPose3, range) {

  noiseModel::Isotropic::shared_ptr model_range =
      noiseModel::Isotropic::Sigma(1, 0.1);
  Matrix Qc = 0.001 * Matrix::Identity(6,6);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);
  double dt = 0.1, tau = 0.04;
  Pose3 body_T_sensor = Pose3(Rot3::Ypr(1.0, 0.4, 0.5), Point3(0.3, 0.6, -0.7));
  Pose3 p1, p2;
  Vector6 v1, v2;
  Point3 land;
  double meas;
  RangeFactor factor;
  Vector expect, actual;
  Matrix expectH1, expectH2, expectH3, expectH4, expectH5;
  Matrix actualH1, actualH2, actualH3, actualH4, actualH5;

  // test at origin, land is in front of z
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  p2 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  v1 = (Vector6() << 0, 0, 0, 0, 0, 0).finished();
  v2 = (Vector6() << 0, 0, 0, 0, 0, 0).finished();
  land = Point3(0, 0, 10);
  meas = 10;
  factor = RangeFactor(meas, model_range, Qc_model, 0, 0, 0, 0, 0, dt, tau);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expect = ((Vector(1) << 0).finished());
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2, land)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2, land)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2, land)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1, land)), v2, 1e-6);
  expectH5 = numericalDerivative11(boost::function<Vector(const Point3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, v2, _1)), land, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));


  // interpolat at origin: forward velocity
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(-0.04, 0, 0));
  p2 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0.06, 0, 0));
  v1 = (Vector6() << 0, 0, 0, 1, 0, 0).finished();
  v2 = (Vector6() << 0, 0, 0, 1, 0, 0).finished();
  land = Point3(0, 0, 10);
  meas = 10;
  factor = RangeFactor(meas, model_range, Qc_model, 0, 0, 0, 0, 0, dt, tau);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expect = ((Vector(1) << 0).finished());
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2, land)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2, land)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2, land)), p2, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1, land)), v2, 1e-4);
  expectH5 = numericalDerivative11(boost::function<Vector(const Point3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, v2, _1)), land, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));


  // interpolat at origin: rotate
  p1 = Pose3(Rot3::Ypr(-0.04, 0, 0), Point3(0, 0, 0));
  p2 = Pose3(Rot3::Ypr(0.06, 0, 0), Point3(0, 0, 0));
  v1 = (Vector6() << 0, 0, 1, 0, 0, 0).finished();
  v2 = (Vector6() << 0, 0, 1, 0, 0, 0).finished();
  land = Point3(0, 0, 10);
  meas = 10;
  factor = RangeFactor(meas, model_range, Qc_model, 0, 0, 0, 0, 0, dt, tau);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expect = ((Vector(1) << 0).finished());
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2, land)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2, land)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2, land)), p2, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1, land)), v2, 1e-4);
  expectH5 = numericalDerivative11(boost::function<Vector(const Point3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, v2, _1)), land, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));


  // forward velocity with random landmark: ground truth measurement by pinhole camera
  // with body_P_sensor
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  p2 = Pose3(Rot3::Ypr(0, 0, 0), Point3(1.5, 0, 0));
  v1 = (Vector6() << 0, 0, 0, 15, 0, 0).finished();
  v2 = (Vector6() << 0, 0, 0, 15, 0, 0).finished();
  // ground truth pose:
  Pose3 ture_pose = Pose3(Rot3::Ypr(0, 0, 0), Point3(0.6, 0, 0));
  land = Point3(3.4, 1.2, 10);
  meas = (ture_pose.compose(body_T_sensor)).range(land);
  factor = RangeFactor(meas, model_range, Qc_model, 0, 0, 0, 0, 0, dt, tau, body_T_sensor);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expect = ((Vector(1) << 0).finished());
  expectH1 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2, land)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2, land)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Vector(const Pose3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2, land)), p2, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector6&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1, land)), v2, 1e-4);
  expectH5 = numericalDerivative11(boost::function<Vector(const Point3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, v2, _1)), land, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  //EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-5));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));

}

/* ************************************************************************** */
TEST(GPInterpolatedRangeFactorPose3, optimization) {
  /**
   * A simple graph:
   *
   *   l1 - pl1_loss
   *   |
   *  range1,2,3
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
   * landmark need a prior since it's a diverge example
   */


  noiseModel::Isotropic::shared_ptr model_prior =  noiseModel::Isotropic::Sigma(6, 0.01);
  noiseModel::Isotropic::shared_ptr model_prior3_loss = noiseModel::Isotropic::Sigma(3, 0.1);
  noiseModel::Isotropic::shared_ptr model_cam = noiseModel::Isotropic::Sigma(1, 0.1);
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
  Pose3 p1i(Rot3::Ypr(0.1, 0.2, 0.4), Point3(0.2, 0.3, -0.2));
  Pose3 p2i(Rot3::Ypr(-0.1, -0.2, -0.4), Point3(1.2, -0.3, 0.2));
  Vector6 v1i = (Vector6() << -0.1, 0, 0, 0.8, 0, 0.2).finished();
  Vector6 v2i = (Vector6() << 0, 0, 0.2, 1.2, 0, -0.1).finished();

  // landmark and measurements
  Point3 land = Point3(0.4, 1.2, 3);
  Point3 landi = Point3(0.3, 1.1, 2.9);
  double meas1 = pcam1.range(land);
  double meas2 = pcam2.range(land);
  double meas3 = pcam3.range(land);

  NonlinearFactorGraph graph;
  graph.add(PriorFactor<Pose3>(Symbol('x', 1), p1, model_prior));
  graph.add(PriorFactor<Pose3>(Symbol('x', 2), p2, model_prior));
  graph.add(PriorFactor<Point3>(Symbol('l', 1), land, model_prior3_loss));
  graph.add(PriorFactor<Vector6>(Symbol('v', 1), v1, model_prior));
  graph.add(PriorFactor<Vector6>(Symbol('v', 2), v2, model_prior));
  graph.add(GaussianProcessPriorPose3(Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), delta_t, Qc_model));
  graph.add(RangeFactor(meas1, model_cam, Qc_model, Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('l', 1), delta_t, tau1));
  graph.add(RangeFactor(meas2, model_cam, Qc_model, Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('l', 1), delta_t, tau2));
  graph.add(RangeFactor(meas3, model_cam, Qc_model, Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('l', 1), delta_t, tau3));

  Values init_values;
  init_values.insert(Symbol('x', 1), p1i);
  init_values.insert(Symbol('v', 1), v1i);
  init_values.insert(Symbol('x', 2), p2i);
  init_values.insert(Symbol('v', 2), v2i);
  init_values.insert(Symbol('l', 1), landi);

  GaussNewtonParams parameters;
  parameters.setVerbosity("ERROR");
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values values = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-6);
  EXPECT(assert_equal(p1, values.at<Pose3>(Symbol('x', 1)), 1e-6));
  EXPECT(assert_equal(p2, values.at<Pose3>(Symbol('x', 2)), 1e-6));
  EXPECT(assert_equal(v1, values.at<Vector6>(Symbol('v', 1)), 1e-6));
  EXPECT(assert_equal(v2, values.at<Vector6>(Symbol('v', 2)), 1e-6));
  EXPECT(assert_equal(land, values.at<Point3>(Symbol('l', 1)), 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
