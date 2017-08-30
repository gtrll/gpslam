/**
*  @file testGPInterpolatedProjectionFactorPose3.cpp
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

#include <gpslam/slam/GPInterpolatedProjectionFactorPose3.h>
#include <gpslam/gp/GaussianProcessPriorPose3.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpslam;


typedef GPInterpolatedProjectionFactorPose3<Cal3_S2> ProjectionFactor;

// error function wrapper for bind 10 params, by ignore default boost::none
inline Vector errorWrapper(const ProjectionFactor& factor,
    const Pose3& pose1, const Vector6& vel1,
    const Pose3& pose2, const Vector6& vel2, const Point3& point) {
  return factor.evaluateError(pose1, vel1, pose2, vel2, point);
}

/* ************************************************************************** */
TEST(GPInterpolatedProjectionFactorPose3, projection) {

  noiseModel::Isotropic::shared_ptr model_cam =
      noiseModel::Isotropic::Sigma(2, 0.1);
  Matrix Qc = 0.001 * Matrix::Identity(6,6);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);
  double dt = 0.1, tau = 0.04;
  boost::shared_ptr<Cal3_S2> K1(new Cal3_S2());
  boost::shared_ptr<Cal3_S2> K2(new Cal3_S2(50, 50, 0, 40, 30));
  Pose3 body_T_sensor = Pose3(Rot3::Ypr(1.0, 0.4, 0.5), Point3(0.3, 0.6, -0.7));
  Pose3 p1, p2;
  Vector6 v1, v2;
  Point3 land;
  Point2 meas;
  ProjectionFactor factor;
  Vector expect, actual;
  Matrix expectH1, expectH2, expectH3, expectH4, expectH5;
  Matrix actualH1, actualH2, actualH3, actualH4, actualH5;

  // test at origin, land is in front of z
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  p2 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  v1 = (Vector6() << 0, 0, 0, 0, 0, 0).finished();
  v2 = (Vector6() << 0, 0, 0, 0, 0, 0).finished();
  land = Point3(0, 0, 10);
  meas = Point2(0, 0);
  factor = ProjectionFactor(meas, model_cam, Qc_model, 0, 0, 0, 0, 0, dt, tau, K1);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expect = ((Vector(2) << 0, 0).finished());
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
  meas = Point2(0, 0);
  factor = ProjectionFactor(meas, model_cam, Qc_model, 0, 0, 0, 0, 0, dt, tau, K1);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expect = ((Vector(2) << 0, 0).finished());
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
  meas = Point2(0, 0);
  factor = ProjectionFactor(meas, model_cam, Qc_model, 0, 0, 0, 0, 0, dt, tau, K1);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expect = ((Vector(2) << 0, 0).finished());
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
  PinholeCamera<Cal3_S2> cam(ture_pose.compose(body_T_sensor), *K2);
  land = Point3(3.4, 1.2, 10);
  meas = cam.project(land);
  factor = ProjectionFactor(meas, model_cam, Qc_model, 0, 0, 0, 0, 0, dt, tau, K2, body_T_sensor);
  actual = factor.evaluateError(p1, v1, p2, v2, land, actualH1, actualH2,
      actualH3, actualH4, actualH5);
  expect = ((Vector(2) << 0, 0).finished());
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
  EXPECT(assert_equal(expectH4, actualH4, 1e-5));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));
}

/* ************************************************************************** */
TEST(GPInterpolatedProjectionFactorPose3, optimization) {
  /**
   * A simple graph:
   *
   *   l1
   *   |
   *  proj1,2,3
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
   * 3 measurements are given in different places
   */


  noiseModel::Isotropic::shared_ptr model_prior =
      noiseModel::Isotropic::Sigma(6, 0.01);
  noiseModel::Isotropic::shared_ptr model_cam =
      noiseModel::Isotropic::Sigma(2, 0.1);
  double delta_t = 0.1, tau1 = 0.02, tau2 = 0.06, tau3 = 0.09;
  Matrix Qc = 0.01 * Matrix::Identity(6,6);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);

  // ground truth
  Pose3 p1(Rot3(), Point3(0,0,0)), p2(Rot3(), Point3(1,0,0));
  Pose3 pcam1(Rot3(), Point3(0.2,0,0)), pcam2(Rot3(), Point3(0.6,0,0)),
      pcam3(Rot3(), Point3(0.9,0,0));
  Vector6 v1 = (Vector6() << 0, 0, 0, 10, 0, 0).finished();
  Vector6 v2 = (Vector6() << 0, 0, 0, 10, 0, 0).finished();

  // init values
  Pose3 p1i(Rot3::Ypr(0.1, 0.2, 0.4), Point3(0.2, 0.3, -0.2));
  Pose3 p2i(Rot3::Ypr(-0.1, -0.2, -0.4), Point3(1.2, -0.3, 0.2));
  Vector6 v1i = (Vector6() << -0.3, 0, 0, 0.7, 0, 0.2).finished();
  Vector6 v2i = (Vector6() << 0, 0, 0.4, 1.2, 0, -0.1).finished();

  // landmark and measurements
  boost::shared_ptr<Cal3_S2> K(new Cal3_S2(50, 50, 0, 40, 30));
  Point3 land = Point3(3.4, 1.2, 20);
  Point3 landi = Point3(3.3, 1.3, 18);
  PinholeCamera<Cal3_S2> cam1(pcam1, *K);
  Point2 meas1 = cam1.project(land);
  PinholeCamera<Cal3_S2> cam2(pcam2, *K);
  Point2 meas2 = cam2.project(land);
  PinholeCamera<Cal3_S2> cam3(pcam3, *K);
  Point2 meas3 = cam3.project(land);

  NonlinearFactorGraph graph;
  graph.add(PriorFactor<Pose3>(Symbol('x', 1), p1, model_prior));
  graph.add(PriorFactor<Pose3>(Symbol('x', 2), p2, model_prior));
  //graph.add(PriorFactor<Vector6>(Symbol('v', 1), v1, model_prior));
  //graph.add(PriorFactor<Vector6>(Symbol('v', 2), v2, model_prior));
  graph.add(GaussianProcessPriorPose3(Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), delta_t, Qc_model));
  graph.add(ProjectionFactor(meas1, model_cam, Qc_model, Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('l', 1), delta_t, tau1, K));
  graph.add(ProjectionFactor(meas2, model_cam, Qc_model, Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('l', 1), delta_t, tau2, K));
  graph.add(ProjectionFactor(meas3, model_cam, Qc_model, Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), Symbol('l', 1), delta_t, tau3, K));

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
