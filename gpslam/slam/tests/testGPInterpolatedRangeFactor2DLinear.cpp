/**
*  @file testGPInterpolatedRangeFactor2DLinear.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <gpslam/slam/GPInterpolatedRangeFactor2DLinear.h>
#include <gpslam/gp/GaussianProcessPriorLinear.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpslam;


typedef GPInterpolatedRangeFactor2DLinear RangeFactor;

// error function wrapper for bind 10 params, by ignore default boost::none
inline Vector errorWrapper(const RangeFactor& factor,
    const Vector3& pose1, const Vector3& vel1,
    const Vector3& pose2, const Vector3& vel2, const Point2& point) {
  return factor.evaluateError(pose1, vel1, pose2, vel2, point);
}

/* ************************************************************************** */
TEST(GPInterpolatedRangeFactor2DLinear, range) {

  noiseModel::Isotropic::shared_ptr model_range =
      noiseModel::Isotropic::Sigma(1, 0.1);
  Matrix Qc = 0.001 * Matrix::Identity(3,3);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);
  double dt = 0.1, tau = 0.04;
  Vector3 p1, p2;
  Vector3 v1, v2;
  Point2 land;
  double meas;
  RangeFactor factor;
  Vector expect, actual;
  Matrix expectH1, expectH2, expectH3, expectH4, expectH5;
  Matrix actualH1, actualH2, actualH3, actualH4, actualH5;

  // test at origin, land is in front of z
  p1 = (Vector3() << 0, 0, 0).finished();
  p2 = (Vector3() << 0, 0, 0).finished();
  v1 = (Vector3() << 0, 0, 0).finished();
  v2 = (Vector3() << 0, 0, 0).finished();
  land = Point2(0, 10);
  meas = 10;
  factor = RangeFactor(meas, 0, 0, 0, 0, 0, model_range, Qc_model, dt, tau);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expect = ((Vector(1) << 0).finished());
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2, land)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2, land)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2, land)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1, land)), v2, 1e-6);
  expectH5 = numericalDerivative11(boost::function<Vector(const Point2&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, v2, _1)), land, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));


  // interpolat at origin: forward velocity
  p1 = (Vector3() << -0.04, 0, 0).finished();
  p2 = (Vector3() << 0.06, 0, 0).finished();
  v1 = (Vector3() << 1, 0, 0).finished();
  v2 = (Vector3() << 1, 0, 0).finished();
  land = Point2(0, 10);
  meas = 10;
  factor = RangeFactor(meas, 0, 0, 0, 0, 0, model_range, Qc_model, dt, tau);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expect = ((Vector(1) << 0).finished());
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2, land)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2, land)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2, land)), p2, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1, land)), v2, 1e-4);
  expectH5 = numericalDerivative11(boost::function<Vector(const Point2&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, v2, _1)), land, 1e-4);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));


  // interpolat at origin: rotate
  p1 = (Vector3() << 0, 0, -0.04).finished();
  p2 = (Vector3() << 0, 0, 0.06).finished();
  v1 = (Vector3() << 0, 0, 1).finished();
  v2 = (Vector3() << 0, 0, 1).finished();
  land = Point2(0, 10);
  meas = 10;
  factor = RangeFactor(meas, 0, 0, 0, 0, 0, model_range, Qc_model, dt, tau);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expect = ((Vector(1) << 0).finished());
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2, land)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2, land)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2, land)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1, land)), v2, 1e-6);
  expectH5 = numericalDerivative11(boost::function<Vector(const Point2&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, v2, _1)), land, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));


  // interpolat at origin: rotate with non-zero rotation bias
  p1 = (Vector3() << 0, 0, 10 * M_PI - 0.04).finished();
  p2 = (Vector3() << 0, 0, 10 * M_PI + 0.06).finished();
  v1 = (Vector3() << 0, 0, 1).finished();
  v2 = (Vector3() << 0, 0, 1).finished();
  land = Point2(0, 10);
  meas = 10;
  factor = RangeFactor(meas, 0, 0, 0, 0, 0, model_range, Qc_model, dt, tau);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expect = ((Vector(1) << 0).finished());
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2, land)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2, land)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2, land)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1, land)), v2, 1e-6);
  expectH5 = numericalDerivative11(boost::function<Vector(const Point2&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, v2, _1)), land, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));


  // forward velocity with random landmark: ground truth measurement by pinhole camera
  p1 = (Vector3() << 0, 0, 0).finished();
  p2 = (Vector3() << 1.5, 0, 0).finished();
  v1 = (Vector3() << 15, 0, 0).finished();
  v2 = (Vector3() << 15, 0, 0).finished();
  // ground truth pose:
  Pose2 ture_pose = Pose2(0.6, 0, 0);
  land = Point2(3.4, 1.2);
  meas = ture_pose.range(land);
  factor = RangeFactor(meas, 0, 0, 0, 0, 0, model_range, Qc_model, dt, tau);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expect = ((Vector(1) << 0).finished());
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2, land)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2, land)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2, land)), p2, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1, land)), v2, 1e-4);
  expectH5 = numericalDerivative11(boost::function<Vector(const Point2&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, v2, _1)), land, 1e-4);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));

  // random stuff to test jacobians
  p1 = (Vector3() << 5.34, 7.1, -4.32).finished();
  p2 = (Vector3() << 1.5, -2.2, 3.0).finished();
  v1 = (Vector3() << 15, 21.3, 32).finished();
  v2 = (Vector3() << -15, 4.2, -30).finished();
  // ground truth pose:
  land = Point2(3.4, 1.2);
  factor = RangeFactor(meas, 0, 0, 0, 0, 0, model_range, Qc_model, dt, tau);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, _1, v1, p2, v2, land)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, _1, p2, v2, land)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, _1, v2, land)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, _1, land)), v2, 1e-6);
  expectH5 = numericalDerivative11(boost::function<Vector(const Point2&)>(
      boost::bind(&errorWrapper, factor, p1, v1, p2, v2, _1)), land, 1e-6);
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));

}

/* ************************************************************************** */
TEST(GPInterpolatedRangeFactor2DLinear, optimization) {
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

  const double rotation_bias = 10 * M_PI;

  noiseModel::Isotropic::shared_ptr model_prior =  noiseModel::Isotropic::Sigma(3, 0.01);
  noiseModel::Isotropic::shared_ptr model_prior2_loss = noiseModel::Isotropic::Sigma(2, 0.1);
  noiseModel::Isotropic::shared_ptr model_cam = noiseModel::Isotropic::Sigma(1, 0.1);
  double delta_t = 0.5, tau1 = 0.05, tau2 = 0.25, tau3 = 0.45;
  Matrix Qc = 0.01 * Matrix::Identity(3,3);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);

  // ground truth
  Vector3 p1 = (Vector3() << 0,0, rotation_bias + 0).finished(),
      p2 = (Vector3() << 5,0, rotation_bias + 0).finished();
  Vector3 pcam1 = (Vector3() << 0.5,0,rotation_bias + 0).finished(),
      pcam2 = (Vector3() << 2.5,0, rotation_bias + 0).finished(),
      pcam3 = (Vector3() << 4.5,0, rotation_bias + 0).finished();
  Vector3 v1 = (Vector3() << 10, 0, 0).finished();
  Vector3 v2 = (Vector3() << 10, 0, 0).finished();

  // init values
  Vector3 p1i = (Vector3() << 0.1, 0.1, rotation_bias - 0.1).finished();
  Vector3 p2i = (Vector3() << 5.1, -0.1, rotation_bias + 0.1).finished();
  Vector3 v1i = (Vector3() << 9.8, 0, 0.2).finished();
  Vector3 v2i = (Vector3() << 10.2, 0, -0.1).finished();

  // landmark and measurements
  Point2 land = Point2(2.4, 3.2);
  Point2 landi = Point2(2.3, 3.1);
  double meas1 = Pose2(pcam1(0), pcam1(1), pcam1(2)).range(land);
  double meas2 = Pose2(pcam2(0), pcam2(1), pcam2(2)).range(land);
  double meas3 = Pose2(pcam3(0), pcam3(1), pcam3(2)).range(land);

  NonlinearFactorGraph graph;
  graph.add(PriorFactor<Vector3>(Symbol('x', 1), p1, model_prior));
  graph.add(PriorFactor<Vector3>(Symbol('x', 2), p2, model_prior));
  graph.add(PriorFactor<Point2>(Symbol('l', 1), land, model_prior2_loss));
  graph.add(PriorFactor<Vector3>(Symbol('v', 1), v1, model_prior));
  graph.add(PriorFactor<Vector3>(Symbol('v', 2), v2, model_prior));

  graph.add(GaussianProcessPriorLinear<3>(Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), delta_t, Qc_model));

  graph.add(RangeFactor(meas1, Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('l', 1), model_cam, Qc_model, delta_t, tau1));
  graph.add(RangeFactor(meas2, Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('l', 1), model_cam, Qc_model, delta_t, tau2));
  graph.add(RangeFactor(meas3, Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('l', 1), model_cam, Qc_model, delta_t, tau3));

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

  EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-4);
  EXPECT(assert_equal(p1, values.at<Vector3>(Symbol('x', 1)), 1e-4));
  EXPECT(assert_equal(p2, values.at<Vector3>(Symbol('x', 2)), 1e-4));
  EXPECT(assert_equal(v1, values.at<Vector3>(Symbol('v', 1)), 1e-4));
  EXPECT(assert_equal(v2, values.at<Vector3>(Symbol('v', 2)), 1e-4));
  EXPECT(assert_equal(land, values.at<Point2>(Symbol('l', 1)), 1e-4));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
