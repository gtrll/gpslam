/**
*  @file testRangeBearingFactor2DLinear.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gpslam/slam/RangeBearingFactor2DLinear.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpslam;


/* ************************************************************************** */
TEST(RangeBearingFactor2DLinear, Factor) {

  noiseModel::Gaussian::shared_ptr model = noiseModel::Isotropic::Sigma(2, 1.0);
  Key key_pose = Symbol('x', 1), key_lnd = Symbol('l', 1);
  Vector3 pose;
  Point2 land;
  RangeBearingFactor2DLinear factor;
  Matrix actualH1, actualH2;
  Matrix expectH1, expectH2;
  Vector actual, expect;

  // case 1
  pose = Vector3(3, 4, 0);
  land = Point2(7, 7);
  factor = RangeBearingFactor2DLinear(key_pose, key_lnd, 5.0, 0.643501108793284, model);
  actual = factor.evaluateError(pose, land, actualH1, actualH2);
  expect = (Vector(2) << 0, 0).finished();
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&RangeBearingFactor2DLinear::evaluateError, factor,
          _1, land, boost::none, boost::none)), pose, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Point2&)>(
      boost::bind(&RangeBearingFactor2DLinear::evaluateError, factor,
          pose, _1, boost::none, boost::none)), land, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));

  // random case
  pose = Vector3(13.1, -4.8, 1.5);
  land = Point2(-5.4, 6.6);
  factor = RangeBearingFactor2DLinear(key_pose, key_lnd, 13.1, 0.0, model);
  actual = factor.evaluateError(pose, land, actualH1, actualH2);
  expect = (Vector(2) << 1.089334716657378, 8.630393461693233).finished();
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&RangeBearingFactor2DLinear::evaluateError, factor,
          _1, land, boost::none, boost::none)), pose, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Point2&)>(
      boost::bind(&RangeBearingFactor2DLinear::evaluateError, factor,
          pose, _1, boost::none, boost::none)), land, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
}

/* ************************************************************************** */
TEST(RangeBearingFactor2DLinear, Optimization) {

  /**
   * A simple graph:
   *
   *     l
   *  p /|\
   *  |/ | \
   *  x1-x2-x3
   *
   * p is pose prior factor to fix the poses, poses are connected by between factor
   * calculated l correctly
   */

  noiseModel::Gaussian::shared_ptr meas_model = noiseModel::Isotropic::Sigma(2, 0.1);
  noiseModel::Gaussian::shared_ptr prior_model = noiseModel::Isotropic::Sigma(3, 1.0);
  noiseModel::Gaussian::shared_ptr between_model = noiseModel::Isotropic::Sigma(3, 0.1);
  Key key_p1 = Symbol('x', 1), key_p2 = Symbol('x', 2), key_p3 = Symbol('x', 3);
  Key key_lnd = Symbol('l', 1);

  // ground truth
  Vector3 p1 = Vector3(0, 0, 0);
  Vector3 p2 = Vector3(1, 0, 0);
  Vector3 p3 = Vector3(2, 1, 0);
  Point2 lnd = Point2(0, 2);

  // meas
  double dist1 = 2.0, dist2 = 2.2360679775, dist3 = 2.2360679775;
  Rot2 bear1(1.570796326794897), bear2(2.034443935795703), bear3(2.677945044588987);
  Vector3 btw12 = Vector3(1, 0, 0), btw23 = Vector3(1, 1, 0);

  // noisy init
  Vector3 p1_init = Vector3(0.2, -0.5, 0.3);
  Vector3 p2_init = Vector3(0.8, 0.2, 0.1);
  Vector3 p3_init = Vector3(2.4, 1.3, -0.4);
  Point2 lnd_init = Point2(0.1, 2.2);

  NonlinearFactorGraph graph;
  graph.add(PriorFactor<Vector3>(key_p1, p1, prior_model));
  graph.add(BetweenFactor<Vector3>(key_p1, key_p2, btw12, between_model));
  graph.add(BetweenFactor<Vector3>(key_p2, key_p3, btw23, between_model));
  graph.add(RangeBearingFactor2DLinear(key_p1, key_lnd, dist1, bear1, meas_model));
  graph.add(RangeBearingFactor2DLinear(key_p2, key_lnd, dist2, bear2, meas_model));
  graph.add(RangeBearingFactor2DLinear(key_p3, key_lnd, dist3, bear3, meas_model));

  Values init_values;
  init_values.insert(key_p1, p1_init);
  init_values.insert(key_p2, p2_init);
  init_values.insert(key_p3, p3_init);
  init_values.insert(key_lnd, lnd_init);

  GaussNewtonParams parameters;
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values values = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-6);
  EXPECT(assert_equal(p1, values.at<Vector3>(key_p1), 1e-6));
  EXPECT(assert_equal(p2, values.at<Vector3>(key_p2), 1e-6));
  EXPECT(assert_equal(p3, values.at<Vector3>(key_p3), 1e-6));
  EXPECT(assert_equal(lnd, values.at<Point2>(key_lnd), 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
