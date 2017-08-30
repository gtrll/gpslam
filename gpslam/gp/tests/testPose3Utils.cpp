/**
*  @file testPose3Utils.cpp
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

#include <gpslam/gp/Pose3utils.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpslam;


/* ************************************************************************** */
// numerical method of jacobians
// see Forster15rss eq. (A.48)
template <class LIE_TYPE>
Matrix numericalLieLeftJacobian(const LIE_TYPE& lie, double dt) {
  const size_t dim = LIE_TYPE::dimension;
  Vector omega = LIE_TYPE::Logmap(lie);
  Matrix J_expect = Matrix(dim, dim);
  for (size_t i = 0; i < dim; i++) {
    Vector dlogmap = Vector::Zero(dim);
    dlogmap(i) = dt;
    LIE_TYPE r = LIE_TYPE::Expmap(omega + dlogmap);
    J_expect.block(0,i,dim,1) = LIE_TYPE::Logmap(r.compose(lie.inverse())) / dt;
  }
  return J_expect;
}

template <class LIE_TYPE>
Matrix numericalLieRightJacobian(const LIE_TYPE& lie, double dt) {
  const size_t dim = LIE_TYPE::dimension;
  Vector omega = LIE_TYPE::Logmap(lie);
  Matrix J_expect = Matrix(dim, dim);
  for (size_t i = 0; i < dim; i++) {
    Vector dlogmap = Vector::Zero(dim);
    dlogmap(i) = dt;
    LIE_TYPE r = LIE_TYPE::Expmap(omega + dlogmap);
    J_expect.block(0,i,dim,1) = LIE_TYPE::Logmap(lie.inverse().compose(r)) / dt;
  }
  return J_expect;
}

// TODO: why inverse version does not work??
#if 0
// see Barfoot14tro eq. (33), Forster15rss eq. (9)
template <class LIE_TYPE>
Matrix numericalLieLeftJacobian(const LIE_TYPE& lie, double dt) {
  const size_t dim = LIE_TYPE::dimension;
  Vector omega = LIE_TYPE::Logmap(lie);
  Matrix J_expect = Matrix(dim, dim);
  for (size_t i = 0; i < dim; i++) {
    Vector dlogmap = zero(dim);
    dlogmap(i) = dt;
    LIE_TYPE dr = LIE_TYPE::Expmap(dlogmap);
    J_expect.block(0,i,dim,1) = (LIE_TYPE::Logmap(lie.compose(dr)) - omega) / dt;
  }
  return J_expect.inverse();
}

template <class LIE_TYPE>
Matrix numericalLieRightJacobian(const LIE_TYPE& lie, double dt) {
  const size_t dim = LIE_TYPE::dimension;
  Vector omega = LIE_TYPE::Logmap(lie);
  Matrix J_expect = Matrix(dim, dim);
  for (size_t i = 0; i < dim; i++) {
    Vector dlogmap = zero(dim);
    dlogmap(i) = dt;
    LIE_TYPE dr = LIE_TYPE::Expmap(dlogmap);
    J_expect.block(0,i,dim,1) = (LIE_TYPE::Logmap(dr.compose(lie)) - omega) / dt;
  }
  return J_expect.inverse();
}
#endif

/* ************************************************************************** */
TEST(testPose3Utils, getBodyCentricVelocity) {
  Pose3 pose1, pose2;
  Vector expect, actual;
  double dt = 0.1;

  // test at origin
  pose1 = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  pose2 = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  expect = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  actual = getBodyCentricVb(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));
  expect = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  actual = getBodyCentricVs(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));

  // test forward x
  pose1 = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  pose2 = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.1, 0.0, 0.0));
  expect = (Vector(6) << 0, 0, 0, 1, 0, 0).finished();
  actual = getBodyCentricVb(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));
  expect = (Vector(6) << 0, 0, 0, 1, 0, 0).finished();
  actual = getBodyCentricVs(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));

  // test rotate y
  pose1 = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  pose2 = Pose3(Rot3::Ypr(0.1, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  expect = (Vector(6) << 0, 0, 1, 0, 0, 0).finished();
  actual = getBodyCentricVb(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));
  expect = (Vector(6) << 0, 0, 1, 0, 0, 0).finished();
  actual = getBodyCentricVs(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));

  // test forward x (body-y) at yaw=90 deg
  pose1 = Pose3(Rot3::Ypr(M_PI / 2.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  pose2 = Pose3(Rot3::Ypr(M_PI / 2.0, 0.0, 0.0), Point3(0.1, 0.0, 0.0));
  expect = (Vector(6) << 0, 0, 0, 0, -1, 0).finished();
  actual = getBodyCentricVb(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));
  expect = (Vector(6) << 0, 0, 0, 1, 0, 0).finished();
  actual = getBodyCentricVs(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));

  // test rotate body+z at yaw=90 deg
  pose1 = Pose3(Rot3::Ypr(M_PI / 2.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  pose2 = Pose3(Rot3::Ypr(M_PI / 2.0 + 0.1, 0.0, 0), Point3(0.0, 0.0, 0.0));
  expect = (Vector(6) << 0, 0, 1, 0, 0, 0).finished();
  actual = getBodyCentricVb(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));
  expect = (Vector(6) << 0, 0, 1, 0, 0, 0).finished();
  actual = getBodyCentricVs(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));

  // test rotate body+x at yaw=90 deg
  pose1 = Pose3(Rot3::Ypr(M_PI / 2.0, 0.0, 0.0), Point3(1.0, 0.0, 0.0));
  pose2 = Pose3(Rot3::Ypr(M_PI / 2.0, 0.0, 0.1), Point3(1.0, 0.0, 0.0));
  expect = (Vector(6) << 1, 0, 0, 0, 0, 0).finished();
  actual = getBodyCentricVb(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));
  expect = (Vector(6) << 0, 1, 0, 0, 0, 1).finished();
  actual = getBodyCentricVs(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));

  // circle motion
  pose1 = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, -1.0, 0.0));
  pose2 = Pose3(Rot3::Ypr(M_PI / 2.0, 0.0, 0.0), Point3(1.0, 0.0, 0.0));
  expect = (Vector(6) << 0, 0, M_PI/2*10, M_PI/2*10, 0, 0).finished();
  actual = getBodyCentricVb(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));
  expect = (Vector(6) << 0, 0, M_PI/2*10, 0, 0, 0).finished();
  actual = getBodyCentricVs(pose1, pose2, dt);
  EXPECT(assert_equal(expect, actual, 1e-6));
}

/* ************************************************************************** */
TEST(testPose3Utils, SO3Jacobian) {
  Rot3 rotbase;
  Matrix J_expect, J_actual;
  const double dt = 1e-6;

  // left jacobian
  rotbase = Rot3::Ypr(0.0, 0.0, 0.0);
  J_expect = numericalLieLeftJacobian(rotbase, dt);
  J_actual = leftJacobianRot3(Rot3::Logmap(rotbase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  rotbase = Rot3::Ypr(1.0, 2.0, 3.0);
  J_expect = numericalLieLeftJacobian(rotbase, dt);
  J_actual = leftJacobianRot3(Rot3::Logmap(rotbase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  // inverse left jacobian
  rotbase = Rot3::Ypr(0.0, 0.0, 0.0);
  J_expect = numericalLieLeftJacobian(rotbase, dt).inverse();
  J_actual = leftJacobianRot3inv(Rot3::Logmap(rotbase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  rotbase = Rot3::Ypr(1.0, 2.0, 3.0);
  J_expect = numericalLieLeftJacobian(rotbase, dt).inverse();
  J_actual = leftJacobianRot3inv(Rot3::Logmap(rotbase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  // right jacobian
  rotbase = Rot3::Ypr(0.0, 0.0, 0.0);
  J_expect = numericalLieRightJacobian(rotbase, dt);
  J_actual = rightJacobianRot3(Rot3::Logmap(rotbase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  rotbase = Rot3::Ypr(1.0, 2.0, 3.0);
  J_expect = numericalLieRightJacobian(rotbase, dt);
  J_actual = rightJacobianRot3(Rot3::Logmap(rotbase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  // inverse right jacobian
  rotbase = Rot3::Ypr(0.0, 0.0, 0.0);
  J_expect = numericalLieRightJacobian(rotbase, dt).inverse();
  J_actual = rightJacobianRot3inv(Rot3::Logmap(rotbase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  rotbase = Rot3::Ypr(1.0, 2.0, 3.0);
  J_expect = numericalLieRightJacobian(rotbase, dt).inverse();
  J_actual = rightJacobianRot3inv(Rot3::Logmap(rotbase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));
}

/* ************************************************************************** */
TEST(testPose3Utils, SE3Jacobian) {
  Pose3 posebase;
  Matrix J_expect, J_actual;
  const double dt = 1e-6;

  // left jacobian
  posebase = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  J_expect = numericalLieLeftJacobian(posebase, dt);
  J_actual = leftJacobianPose3(Pose3::Logmap(posebase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  posebase = Pose3(Rot3::Ypr(1e-5, 0.0, 1e-5), Point3(0.0, 2e-5, 0.0));
  J_expect = numericalLieLeftJacobian(posebase, dt);
  J_actual = leftJacobianPose3(Pose3::Logmap(posebase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  posebase = Pose3(Rot3::Ypr(1.0, 2.0, 3.0), Point3(4.0, 5.0, 6.0));
  J_expect = numericalLieLeftJacobian(posebase, dt);
  J_actual = leftJacobianPose3(Pose3::Logmap(posebase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  // left inverse jacobian
  posebase = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  J_expect = numericalLieLeftJacobian(posebase, dt).inverse();
  J_actual = leftJacobianPose3inv(Pose3::Logmap(posebase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  posebase = Pose3(Rot3::Ypr(1e-5, 0.0, 1e-5), Point3(0.0, 2e-5, 0.0));
  J_expect = numericalLieLeftJacobian(posebase, dt).inverse();
  J_actual = leftJacobianPose3inv(Pose3::Logmap(posebase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-8));

  posebase = Pose3(Rot3::Ypr(1.0, 2.0, 3.0), Point3(4.0, 5.0, 6.0));
  J_expect = numericalLieLeftJacobian(posebase, dt).inverse();
  J_actual = leftJacobianPose3inv(Pose3::Logmap(posebase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-5));

  // right jacobian
  posebase = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  J_expect = numericalLieRightJacobian(posebase, dt);
  J_actual = rightJacobianPose3(Pose3::Logmap(posebase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  posebase = Pose3(Rot3::Ypr(1e-5, 0.0, 1e-5), Point3(0.0, 2e-5, 0.0));
  J_expect = numericalLieRightJacobian(posebase, dt);
  J_actual = rightJacobianPose3(Pose3::Logmap(posebase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  posebase = Pose3(Rot3::Ypr(1.0, 2.0, 3.0), Point3(4.0, 5.0, 6.0));
  J_expect = numericalLieRightJacobian(posebase, dt);
  J_actual = rightJacobianPose3(Pose3::Logmap(posebase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  // inverse right jacobian
  posebase = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  J_expect = numericalLieRightJacobian(posebase, dt).inverse();
  J_actual = rightJacobianPose3inv(Pose3::Logmap(posebase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-6));

  posebase = Pose3(Rot3::Ypr(1e-5, 0.0, 1e-5), Point3(0.0, 2e-5, 0.0));
  J_expect = numericalLieRightJacobian(posebase, dt).inverse();
  J_actual = rightJacobianPose3inv(Pose3::Logmap(posebase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-8));

  posebase = Pose3(Rot3::Ypr(1.0, 2.0, 3.0), Point3(4.0, 5.0, 6.0));
  J_expect = numericalLieRightJacobian(posebase, dt).inverse();
  J_actual = rightJacobianPose3inv(Pose3::Logmap(posebase));
  EXPECT(assert_equal(J_expect, J_actual, 1e-5));
}

/* ************************************************************************** */
TEST(testPose3Utils, SE3velocity) {
  // check Anderson15iros eq. (8)
  Pose3 posebase, poseadd;
  Vector v_expect, v_actual;
  Vector dlogmap;
  const double dt = 0.01;

  // origin, zero vel
  posebase = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0, 0, 0));
  dlogmap = (Vector(6) << 0, 0, 0, 0, 0, 0).finished() * 1e-4;
  poseadd = Pose3::Expmap(Pose3::Logmap(posebase) + dlogmap);
  v_expect = getBodyCentricVb(posebase, poseadd, dt);
  v_actual = rightJacobianPose3(Pose3::Logmap(posebase)) * dlogmap / dt;
  EXPECT(assert_equal(v_expect, v_actual, 1e-6));
  v_expect = getBodyCentricVs(posebase, poseadd, dt);
  v_actual = leftJacobianPose3(Pose3::Logmap(posebase)) * dlogmap / dt;
  EXPECT(assert_equal(v_expect, v_actual, 1e-6));

  // origin, non zero vel
  posebase = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0, 0, 0));
  dlogmap = (Vector(6) << 1, 2, -4, 5, 2, 3).finished() * 1e-4;
  poseadd = Pose3::Expmap(Pose3::Logmap(posebase) + dlogmap);
  v_expect = getBodyCentricVb(posebase, poseadd, dt);
  v_actual = rightJacobianPose3(Pose3::Logmap(posebase)) * dlogmap / dt;
  EXPECT(assert_equal(v_expect, v_actual, 1e-6));
  v_expect = getBodyCentricVs(posebase, poseadd, dt);
  v_actual = leftJacobianPose3(Pose3::Logmap(posebase)) * dlogmap / dt;
  EXPECT(assert_equal(v_expect, v_actual, 1e-6));

  // rnd pose, non zero vel
  posebase = Pose3(Rot3::Ypr(2.4, 1.2, 3.9), Point3(43, -5, 12));
  dlogmap = (Vector(6) << 1, 2, -4, 5, 2, 3).finished() * 1e-4;
  poseadd = Pose3::Expmap(Pose3::Logmap(posebase) + dlogmap);
  v_expect = getBodyCentricVb(posebase, poseadd, dt);
  v_actual = rightJacobianPose3(Pose3::Logmap(posebase)) * dlogmap / dt;
  EXPECT(assert_equal(v_expect, v_actual, 1e-4));
  v_expect = getBodyCentricVs(posebase, poseadd, dt);
  v_actual = leftJacobianPose3(Pose3::Logmap(posebase)) * dlogmap / dt;
  EXPECT(assert_equal(v_expect, v_actual, 1e-4));
}

/* ************************************************************************** */
// wrap v from convertVbtoVW
Vector3 vWrapperVb(const Vector6& v6, const Pose3& pose) {
  Vector3 v,w;
  convertVbtoVW(v6, pose, v, w);
  return v;
}

// wrap w from convertVbtoVW
Vector3 wWrapperVb(const Vector6& v6, const Pose3& pose) {
  Vector3 v,w;
  convertVbtoVW(v6, pose, v, w);
  return w;
}

TEST(testPose3Utils, convertVelocityVW) {
  Pose3 pose;
  Vector6 v6exp, v6act;
  Vector3 vact, wact, vexp, wexp;
  Matrix36 H1vexp, H1vact, H1wexp, H1wact;
  Matrix63 H2vexp, H2vact, H2wexp, H2wact;
  Matrix6 H2pexp, H2pact;


  // zero pose
  pose = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  v6exp = (Vector6() << 1, 0, 0, 1, 0, 0).finished();
  vexp = Vector3(1, 0, 0), wexp = Vector3(1, 0, 0);
  convertVbtoVW(v6exp, pose, vact, wact, H1vact, H1wact);
  H1vexp = numericalDerivative11(boost::function<Vector3(const Vector6&)>(
      boost::bind(&vWrapperVb, _1, pose)), v6exp, 1e-6);
  H1wexp = numericalDerivative11(boost::function<Vector3(const Vector6&)>(
      boost::bind(&wWrapperVb, _1, pose)), v6exp, 1e-6);
  EXPECT(assert_equal(vexp, vact, 1e-9));
  EXPECT(assert_equal(wexp, wact, 1e-9));
  EXPECT(assert_equal(H1vexp, H1vact, 1e-6));
  EXPECT(assert_equal(H1wexp, H1wact, 1e-6));
  v6act = convertVWtoVb(vexp, wexp, pose, H2vact, H2wact, H2pact);
  H2vexp = numericalDerivative11(boost::function<Vector6(const Vector3&)>(
      boost::bind(&convertVWtoVb, _1, wexp, pose, boost::none, boost::none, boost::none)), vexp, 1e-6);
  H2wexp = numericalDerivative11(boost::function<Vector6(const Vector3&)>(
      boost::bind(&convertVWtoVb, vexp, _1, pose, boost::none, boost::none, boost::none)), wexp, 1e-6);
  H2pexp = numericalDerivative11(boost::function<Vector6(const Pose3&)>(
      boost::bind(&convertVWtoVb, vexp, wexp, _1, boost::none, boost::none, boost::none)), pose, 1e-6);
  EXPECT(assert_equal(v6exp, v6act, 1e-9));
  EXPECT(assert_equal(H2vexp, H2vact, 1e-6));
  EXPECT(assert_equal(H2wexp, H2wact, 1e-6));
  EXPECT(assert_equal(H2pexp, H2pact, 1e-6));


  pose = Pose3(Rot3::Ypr(0.0, 0.0, 0.0), Point3(3.0, 4.0, 5.0));
  v6exp = (Vector6() << 1, -2, -3, 1, 2, 3).finished();
  vexp = Vector3(1, 2, 3), wexp = Vector3(1, -2, -3);
  convertVbtoVW(v6exp, pose, vact, wact, H1vact, H1wact);
  H1vexp = numericalDerivative11(boost::function<Vector3(const Vector6&)>(
      boost::bind(&vWrapperVb, _1, pose)), v6exp, 1e-6);
  H1wexp = numericalDerivative11(boost::function<Vector3(const Vector6&)>(
      boost::bind(&wWrapperVb, _1, pose)), v6exp, 1e-6);
  EXPECT(assert_equal(vexp, vact, 1e-9));
  EXPECT(assert_equal(wexp, wact, 1e-9));
  EXPECT(assert_equal(H1vexp, H1vact, 1e-6));
  EXPECT(assert_equal(H1wexp, H1wact, 1e-6));
  v6act = convertVWtoVb(vexp, wexp, pose, H2vact, H2wact, H2pact);
  H2vexp = numericalDerivative11(boost::function<Vector6(const Vector3&)>(
      boost::bind(&convertVWtoVb, _1, wexp, pose, boost::none, boost::none, boost::none)), vexp, 1e-6);
  H2wexp = numericalDerivative11(boost::function<Vector6(const Vector3&)>(
      boost::bind(&convertVWtoVb, vexp, _1, pose, boost::none, boost::none, boost::none)), wexp, 1e-6);
  H2pexp = numericalDerivative11(boost::function<Vector6(const Pose3&)>(
      boost::bind(&convertVWtoVb, vexp, wexp, _1, boost::none, boost::none, boost::none)), pose, 1e-6);
  EXPECT(assert_equal(v6exp, v6act, 1e-9));
  EXPECT(assert_equal(H2vexp, H2vact, 1e-6));
  EXPECT(assert_equal(H2wexp, H2wact, 1e-6));
  EXPECT(assert_equal(H2pexp, H2pact, 1e-6));


  // yaw = 90 deg pose
  pose = Pose3(Rot3::Ypr(M_PI/2.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  v6exp = (Vector6() << 0, 0, 1, 0, -1, 0).finished();
  vexp = Vector3(1, 0, 0), wexp = Vector3(0, 0, 1);
  convertVbtoVW(v6exp, pose, vact, wact, H1vact, H1wact);
  H1vexp = numericalDerivative11(boost::function<Vector3(const Vector6&)>(
      boost::bind(&vWrapperVb, _1, pose)), v6exp, 1e-6);
  H1wexp = numericalDerivative11(boost::function<Vector3(const Vector6&)>(
      boost::bind(&wWrapperVb, _1, pose)), v6exp, 1e-6);
  EXPECT(assert_equal(vexp, vact, 1e-9));
  EXPECT(assert_equal(wexp, wact, 1e-9));
  EXPECT(assert_equal(H1vexp, H1vact, 1e-6));
  EXPECT(assert_equal(H1wexp, H1wact, 1e-6));
  v6act = convertVWtoVb(vexp, wexp, pose, H2vact, H2wact, H2pact);
  H2vexp = numericalDerivative11(boost::function<Vector6(const Vector3&)>(
      boost::bind(&convertVWtoVb, _1, wexp, pose, boost::none, boost::none, boost::none)), vexp, 1e-6);
  H2wexp = numericalDerivative11(boost::function<Vector6(const Vector3&)>(
      boost::bind(&convertVWtoVb, vexp, _1, pose, boost::none, boost::none, boost::none)), wexp, 1e-6);
  H2pexp = numericalDerivative11(boost::function<Vector6(const Pose3&)>(
      boost::bind(&convertVWtoVb, vexp, wexp, _1, boost::none, boost::none, boost::none)), pose, 1e-6);
  EXPECT(assert_equal(v6exp, v6act, 1e-9));
  EXPECT(assert_equal(H2vexp, H2vact, 1e-6));
  EXPECT(assert_equal(H2wexp, H2wact, 1e-6));
  EXPECT(assert_equal(H2pexp, H2pact, 1e-6));


  pose = Pose3(Rot3::Ypr(M_PI/2.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  v6exp = (Vector6() << 0, 0, 1, 4, 3, 0).finished();
  vexp = Vector3(-3, 4, 0), wexp = Vector3(0, 0, 1);
  convertVbtoVW(v6exp, pose, vact, wact, H1vact, H1wact);
  H1vexp = numericalDerivative11(boost::function<Vector3(const Vector6&)>(
      boost::bind(&vWrapperVb, _1, pose)), v6exp, 1e-6);
  H1wexp = numericalDerivative11(boost::function<Vector3(const Vector6&)>(
      boost::bind(&wWrapperVb, _1, pose)), v6exp, 1e-6);
  EXPECT(assert_equal(vexp, vact, 1e-9));
  EXPECT(assert_equal(wexp, wact, 1e-9));
  EXPECT(assert_equal(H1vexp, H1vact, 1e-6));
  EXPECT(assert_equal(H1wexp, H1wact, 1e-6));
  v6act = convertVWtoVb(vexp, wexp, pose, H2vact, H2wact, H2pact);
  H2vexp = numericalDerivative11(boost::function<Vector6(const Vector3&)>(
      boost::bind(&convertVWtoVb, _1, wexp, pose, boost::none, boost::none, boost::none)), vexp, 1e-6);
  H2wexp = numericalDerivative11(boost::function<Vector6(const Vector3&)>(
      boost::bind(&convertVWtoVb, vexp, _1, pose, boost::none, boost::none, boost::none)), wexp, 1e-6);
  H2pexp = numericalDerivative11(boost::function<Vector6(const Pose3&)>(
      boost::bind(&convertVWtoVb, vexp, wexp, _1, boost::none, boost::none, boost::none)), pose, 1e-6);
  EXPECT(assert_equal(v6exp, v6act, 1e-9));
  EXPECT(assert_equal(H2vexp, H2vact, 1e-6));
  EXPECT(assert_equal(H2wexp, H2wact, 1e-6));
  EXPECT(assert_equal(H2pexp, H2pact, 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
