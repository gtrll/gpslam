/**
*  @file testGaussianProcessInterpolatorPose3VW.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <gpslam/gp/GaussianProcessInterpolatorPose3VW.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpslam;


Pose3 interWrapper(const GaussianProcessInterpolatorPose3VW& interpolater,
    const Pose3& pose1, const Vector3& v1, const Vector3& omega1,
    const Pose3& pose2, const Vector3& v2, const Vector3& omega2) {
  return interpolater.interpolatePose(pose1, v1, omega1, pose2, v2, omega2);
}

/* ************************************************************************** */
TEST(GaussianProcessInterpolatorPose3VW, interpolatePose) {
  Pose3 p1, p2, expect, actual;
  Vector3 v1, v2, w1, w2;
  Matrix actualH1, actualH2, actualH3, actualH4, actualH5, actualH6;
  Matrix expectH1, expectH2, expectH3, expectH4, expectH5, expectH6;
  Matrix6 Qc = 0.01 * Matrix::Identity(6,6);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);
  double dt = 0.1, tau = 0.03;
  GaussianProcessInterpolatorPose3VW base(Qc_model, dt, tau);

  // test at origin
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  p2 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  v1 = Vector3(0, 0, 0); w1 = Vector3(0, 0, 0);
  v2 = Vector3(0, 0, 0); w2 = Vector3(0, 0, 0);
  actual = base.interpolatePose(p1, v1, w1, p2, v2, w2, actualH1, actualH2,
      actualH3, actualH4, actualH5, actualH6);
  expect = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  expectH1 = numericalDerivative11(boost::function<Pose3(const Pose3&)>(
      boost::bind(interWrapper, base, _1, v1, w1, p2, v2, w2)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, _1, w1, p2, v2, w2)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, v1, _1, p2, v2, w2)), w1, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Pose3(const Pose3&)>(
      boost::bind(interWrapper, base, p1, v1, w1, _1, v2, w2)), p2, 1e-6);
  expectH5 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, v1, w1, p2, _1, w2)), v2, 1e-6);
  expectH6 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, v1, w1, p2, v2, _1)), w2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-8));
  EXPECT(assert_equal(expectH2, actualH2, 1e-8));
  EXPECT(assert_equal(expectH3, actualH3, 1e-8));
  EXPECT(assert_equal(expectH4, actualH4, 1e-8));
  EXPECT(assert_equal(expectH5, actualH5, 1e-8));
  EXPECT(assert_equal(expectH6, actualH6, 1e-8));

  // test forward
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  p2 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0.1, 0.2, 0));
  v1 = Vector3(1, 2, 0); w1 = Vector3(0, 0, 0);
  v2 = Vector3(1, 2, 0); w2 = Vector3(0, 0, 0);
  actual = base.interpolatePose(p1, v1, w1, p2, v2, w2, actualH1, actualH2,
      actualH3, actualH4, actualH5, actualH6);
  expect = Pose3(Rot3::Ypr(0, 0, 0), Point3(0.03, 0.06, 0));
  expectH1 = numericalDerivative11(boost::function<Pose3(const Pose3&)>(
      boost::bind(interWrapper, base, _1, v1, w1, p2, v2, w2)), p1, 1e-4);
  expectH2 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, _1, w1, p2, v2, w2)), v1, 1e-4);
  expectH3 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, v1, _1, p2, v2, w2)), w1, 1e-4);
  expectH4 = numericalDerivative11(boost::function<Pose3(const Pose3&)>(
      boost::bind(interWrapper, base, p1, v1, w1, _1, v2, w2)), p2, 1e-4);
  expectH5 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, v1, w1, p2, _1, w2)), v2, 1e-4);
  expectH6 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, v1, w1, p2, v2, _1)), w2, 1e-4);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));
  EXPECT(assert_equal(expectH6, actualH6, 1e-6));


  // test rotate
  p1 = Pose3(Rot3::Ypr(0, 0, 0), Point3(0, 0, 0));
  p2 = Pose3(Rot3::Ypr(0.1, 0, 0), Point3(0, 0, 0));
  v1 = Vector3(0, 0, 0); w1 = Vector3(0, 0, 1);
  v2 = Vector3(0, 0, 0); w2 = Vector3(0, 0, 1);
  actual = base.interpolatePose(p1, v1, w1, p2, v2, w2, actualH1, actualH2,
      actualH3, actualH4, actualH5, actualH6);
  expect = Pose3(Rot3::Ypr(0.03, 0, 0), Point3(0.0, 0, 0));
  expectH1 = numericalDerivative11(boost::function<Pose3(const Pose3&)>(
      boost::bind(interWrapper, base, _1, v1, w1, p2, v2, w2)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, _1, w1, p2, v2, w2)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, v1, _1, p2, v2, w2)), w1, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Pose3(const Pose3&)>(
      boost::bind(interWrapper, base, p1, v1, w1, _1, v2, w2)), p2, 1e-6);
  expectH5 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, v1, w1, p2, _1, w2)), v2, 1e-6);
  expectH6 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, v1, w1, p2, v2, _1)), w2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-8));
  EXPECT(assert_equal(expectH2, actualH2, 1e-8));
  EXPECT(assert_equal(expectH3, actualH3, 1e-8));
  EXPECT(assert_equal(expectH4, actualH4, 1e-8));
  EXPECT(assert_equal(expectH5, actualH5, 1e-8));
  EXPECT(assert_equal(expectH6, actualH6, 1e-8));


  // some random stuff, just test jacobians
  p1 = Pose3(Rot3::Ypr(0.4, -0.8, 0.2), Point3(3, -8, 2));
  p2 = Pose3(Rot3::Ypr(0.1, 0.3, -0.5), Point3(-9, 3, 4));
  v1 = Vector3(0.1, -0.2, -1.4); w1 = Vector3(0.5, 0.9, 0.7);
  v1 = Vector3(0.6, 0.3, -0.9); w1 = Vector3(0.4, -0.2, 0.8);
  actual = base.interpolatePose(p1, v1, w1, p2, v2, w2, actualH1, actualH2,
      actualH3, actualH4, actualH5, actualH6);
  expectH1 = numericalDerivative11(boost::function<Pose3(const Pose3&)>(
      boost::bind(interWrapper, base, _1, v1, w1, p2, v2, w2)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, _1, w1, p2, v2, w2)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, v1, _1, p2, v2, w2)), w1, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Pose3(const Pose3&)>(
      boost::bind(interWrapper, base, p1, v1, w1, _1, v2, w2)), p2, 1e-6);
  expectH5 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, v1, w1, p2, _1, w2)), v2, 1e-6);
  expectH6 = numericalDerivative11(boost::function<Pose3(const Vector3&)>(
      boost::bind(interWrapper, base, p1, v1, w1, p2, v2, _1)), w2, 1e-6);
  EXPECT(assert_equal(expectH1, actualH1, 1e-8));
  EXPECT(assert_equal(expectH2, actualH2, 1e-8));
  EXPECT(assert_equal(expectH3, actualH3, 1e-8));
  EXPECT(assert_equal(expectH4, actualH4, 1e-8));
  EXPECT(assert_equal(expectH5, actualH5, 1e-8));
  EXPECT(assert_equal(expectH6, actualH6, 1e-8));

}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
