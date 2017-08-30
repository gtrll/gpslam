/**
*  @file testGaussianProcessInterpolatorRot3.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <gpslam/gp/GaussianProcessInterpolatorRot3.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpslam;


/* ************************************************************************** */
TEST(GaussianProcessInterpolatorRot3, interpolatePose) {
  Rot3 p1, p2, expect, actual;
  Vector3 v1, v2;
  Matrix actualH1, actualH2, actualH3, actualH4;
  Matrix expectH1, expectH2, expectH3, expectH4;
  Matrix3 Qc = 0.01 * Matrix::Identity(3,3);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);
  double dt = 0.1, tau = 0.03;
  GaussianProcessInterpolatorRot3 base(Qc_model, dt, tau);

  // test at origin
  p1 = Rot3::Ypr(0, 0, 0);
  p2 = Rot3::Ypr(0, 0, 0);
  v1 = (Vector3() << 0, 0, 0).finished();
  v2 = (Vector3() << 0, 0, 0).finished();
  actual = base.interpolatePose(p1, v1, p2, v2, actualH1, actualH2,
      actualH3, actualH4);
  expect = Rot3::Ypr(0, 0, 0);
  expectH1 = numericalDerivative11(boost::function<Rot3(const Rot3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Rot3(const Vector3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Rot3(const Rot3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Rot3(const Vector3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-8));
  EXPECT(assert_equal(expectH2, actualH2, 1e-8));
  EXPECT(assert_equal(expectH3, actualH3, 1e-8));
  EXPECT(assert_equal(expectH4, actualH4, 1e-8));


  // test rotate x
  p1 = Rot3::Ypr(0, 0, 0);
  p2 = Rot3::Ypr(0, 0, 0.1);
  v1 = (Vector3() << 1, 0, 0).finished();
  v2 = (Vector3() << 1, 0, 0).finished();
  actual = base.interpolatePose(p1, v1, p2, v2, actualH1, actualH2,
      actualH3, actualH4);
  expect = Rot3::Ypr(0, 0, 0.03);
  expectH1 = numericalDerivative11(boost::function<Rot3(const Rot3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Rot3(const Vector3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Rot3(const Rot3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Rot3(const Vector3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));


  // test rotate z
  p1 = Rot3::Ypr(0, 0, 0);
  p2 = Rot3::Ypr(0.1, 0, 0);
  v1 = (Vector3() << 0, 0, 1).finished();
  v2 = (Vector3() << 0, 0, 1).finished();
  actual = base.interpolatePose(p1, v1, p2, v2, actualH1, actualH2,
      actualH3, actualH4);
  expect = Rot3::Ypr(0.03, 0, 0);
  expectH1 = numericalDerivative11(boost::function<Rot3(const Rot3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Rot3(const Vector3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Rot3(const Rot3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Rot3(const Vector3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-8));
  EXPECT(assert_equal(expectH2, actualH2, 1e-8));
  EXPECT(assert_equal(expectH3, actualH3, 1e-8));
  EXPECT(assert_equal(expectH4, actualH4, 1e-8));


  // some random stuff, just test jacobians
  p1 = Rot3::Ypr(0.4, -0.8, 0.2);
  p2 = Rot3::Ypr(0.1, 0.3, -0.5);
  v1 = (Vector3() << 0.1, -0.2, -1.4).finished();
  v2 = (Vector3() << 0.6, 0.3, -0.9).finished();
  actual = base.interpolatePose(p1, v1, p2, v2, actualH1, actualH2,
      actualH3, actualH4);
  expectH1 = numericalDerivative11(boost::function<Rot3(const Rot3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Rot3(const Vector3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Rot3(const Rot3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Rot3(const Vector3&)>(
      boost::bind(&GaussianProcessInterpolatorRot3::interpolatePose, base,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expectH1, actualH1, 1e-8));
  EXPECT(assert_equal(expectH2, actualH2, 1e-8));
  EXPECT(assert_equal(expectH3, actualH3, 1e-8));
  EXPECT(assert_equal(expectH4, actualH4, 1e-8));

}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
