/**
*  @file testGaussianProcessPriorPose3VW.cpp
*  @author Jing Dong
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

#include <gpslam/gp/GaussianProcessPriorPose3VW.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpslam;


Vector errorWrapper(const GaussianProcessPriorPose3VW& factor,
    const Pose3& pose1, const Vector3& vel1, const Vector3& omega1,
    const Pose3& pose2, const Vector3& vel2, const Vector3& omega2) {
  return factor.evaluateError(pose1, vel1, omega1, pose2, vel2, omega2);
}

/* ************************************************************************** */
TEST(GaussianProcessPriorPose3VW, Factor) {

  const double delta_t = 0.1;
  Matrix Qc = 0.01 * Matrix::Identity(6,6);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);
  Key key_pose1 = Symbol('x', 1), key_pose2 = Symbol('x', 2);
  Key key_vel1 = Symbol('v', 1), key_vel2 = Symbol('v', 2);
  Key key_omega1 = Symbol('w', 1), key_omega2 = Symbol('w', 2);
  GaussianProcessPriorPose3VW factor(key_pose1, key_vel1, key_omega1, key_pose2, key_vel2, key_omega2, delta_t, Qc_model);
  Pose3 p1, p2;
  Vector3 v1, v2, w1, w2;
  Matrix actualH1, actualH2, actualH3, actualH4, actualH5, actualH6;
  Matrix expectH1, expectH2, expectH3, expectH4, expectH5, expectH6;
  Vector actual, expect;


  // test at origin
  p1 = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  p2 = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  v1 = Vector3(0, 0, 0); w1 = Vector3(0, 0, 0);
  v2 = Vector3(0, 0, 0); w2 = Vector3(0, 0, 0);
  actual = factor.evaluateError(p1, v1, w1, p2, v2, w2, actualH1, actualH2, actualH3, actualH4, actualH5, actualH6);
  expect = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
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


  // test at const forward velocity v1 = v2 = 1.0;
  p1 = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  p2 = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.1, 0.0, 0.0));
  v1 = Vector3(1, 0, 0); w1 = Vector3(0, 0, 0);
  v2 = Vector3(1, 0, 0); w2 = Vector3(0, 0, 0);
  actual = factor.evaluateError(p1, v1, w1, p2, v2, w2, actualH1, actualH2, actualH3, actualH4, actualH5, actualH6);
  expect = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
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


  // test at const rotation w1 = w2 = 1.0;
  p1 = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  p2 = Pose3(Rot3::Ypr(0.1, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  v1 = Vector3(0, 0, 0); w1 = Vector3(0, 0, 1);
  v2 = Vector3(0, 0, 0); w2 = Vector3(0, 0, 1);
  actual = factor.evaluateError(p1, v1, w1, p2, v2, w2, actualH1, actualH2, actualH3, actualH4, actualH5, actualH6);
  expect = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
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


  // some random stuff just for testing jacobian (error is not zero)
  p1 = Pose3(Rot3::Ypr(-0.1, 1.2, 0.3), Point3(-4.0, 2.0, 14.0));
  p2 = Pose3(Rot3::Ypr(2.4, -2.5, 3.7), Point3(9.0, -8.0, -7.0));
  v1 = Vector3(2, 3, 1); w1 = Vector3(5, 4, 9);
  v2 = Vector3(1, 3, 8); w1 = Vector3(0, 6, 4);
  actual = factor.evaluateError(p1, v1, w1, p2, v2, w2, actualH1, actualH2, actualH3, actualH4, actualH5, actualH6);
  expect = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
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
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));
  EXPECT(assert_equal(expectH6, actualH6, 1e-6));
}

/* ************************************************************************** */
TEST(GaussianProcessPriorPose3VW, Optimization) {
  /**
   * A simple graph:
   *
   * p1   p2
   * |    |
   * x1   x2
   *  \  /
   *   gp
   *  /  \
   * v1  v2
   *
   * p1 and p2 are pose prior factor to fix the poses, gp is the GP factor
   * that get correct velocity of v2
   */

  noiseModel::Isotropic::shared_ptr model_prior =
      noiseModel::Isotropic::Sigma(6, 0.001);
  double delta_t = 1;
  Matrix Qc = 0.01 * Matrix::Identity(6,6);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);

  Pose3 pose1(Rot3(), Point3(0,0,0)), pose2(Rot3(), Point3(1,0,0));
  Vector3 v1 = Vector3(1, 0, 0), w1 = Vector3(0, 0, 0);
  Vector3 v2 = Vector3(1, 0, 0), w2 = Vector3(0, 0, 0);

  NonlinearFactorGraph graph;
  graph.add(PriorFactor<Pose3>(Symbol('x', 1), pose1, model_prior));
  graph.add(PriorFactor<Pose3>(Symbol('x', 2), pose2, model_prior));

  graph.add(GaussianProcessPriorPose3VW(Symbol('x', 1), Symbol('v', 1), Symbol('w', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('w', 2), delta_t, Qc_model));

  Values init_values;
  init_values.insert(Symbol('x', 1), pose1);
  init_values.insert(Symbol('v', 1), v1);
  init_values.insert(Symbol('w', 1), w1);
  init_values.insert(Symbol('x', 2), pose2);
  init_values.insert(Symbol('v', 2), v2);
  init_values.insert(Symbol('w', 2), w2);

  GaussNewtonParams parameters;
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values values = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-6);
  EXPECT(assert_equal(pose1, values.at<Pose3>(Symbol('x', 1)), 1e-6));
  EXPECT(assert_equal(pose2, values.at<Pose3>(Symbol('x', 2)), 1e-6));
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
