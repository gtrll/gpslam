/**
*  @file testRangeFactor2DLinear.cpp
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

#include <gpslam/slam/OdometryFactor2DLinear.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpslam;


/* ************************************************************************** */
TEST(OdometryFactor2DLinear, Factor) {

  noiseModel::Gaussian::shared_ptr model = noiseModel::Isotropic::Sigma(3, 1.0);
  Key key_pose1 = Symbol('x', 1), key_pose2 = Symbol('x', 2);
  Vector3 pose1, pose2, measured;
  OdometryFactor2DLinear factor;
  Matrix actualH1, actualH2;
  Matrix expectH1, expectH2;
  Vector actual, expect;

  // origin: zero case (not differentiable)
  pose1 = Vector3(0, 0, 0);
  pose2 = Vector3(0, 0, 0);
  measured = Vector3(0, 0, 0);
  factor = OdometryFactor2DLinear(key_pose1, key_pose2, measured, model);
  actual = factor.evaluateError(pose1, pose2);
  expect = (Vector(3) << 0, 0, 0).finished();
  EXPECT(assert_equal(expect, actual, 1e-6));

  // case 1
  pose1 = Vector3(0, 0, 0);
  pose2 = Vector3(1, 0, 0);
  measured = Vector3(1, 0, 0);
  factor = OdometryFactor2DLinear(key_pose1, key_pose2, measured, model);
  actual = factor.evaluateError(pose1, pose2, actualH1, actualH2);
  expect = (Vector(3) << 0, 0, 0).finished();
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&OdometryFactor2DLinear::evaluateError, factor,
          _1, pose2, boost::none, boost::none)), pose1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&OdometryFactor2DLinear::evaluateError, factor,
          pose1, _1, boost::none, boost::none)), pose2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));


  // case 2
  pose1 = Vector3(42, 24, 1.570796326794897);
  pose2 = Vector3(42, 25, 2.570796326794897);
  measured = Vector3(1, 0, 1.0);
  factor = OdometryFactor2DLinear(key_pose1, key_pose2, measured, model);
  actual = factor.evaluateError(pose1, pose2, actualH1, actualH2);
  expect = (Vector(3) << 0, 0, 0).finished();
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&OdometryFactor2DLinear::evaluateError, factor,
          _1, pose2, boost::none, boost::none)), pose1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&OdometryFactor2DLinear::evaluateError, factor,
          pose1, _1, boost::none, boost::none)), pose2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
