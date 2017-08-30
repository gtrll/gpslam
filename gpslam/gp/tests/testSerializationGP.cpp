/**
*  @file testSerialization.cpp
*  @brief test serialization of all factors
*  @author Jing Dong
**/

#include <gpslam/gp/GaussianProcessInterpolatorLinear.h>
/*
#include <gpslam/gp/GaussianProcessFactorBasePose3.h>
#include <gpslam/gp/GaussianProcessFactorBasePose3VW.h>
#include <gpslam/gp/GaussianProcessFactorBasePose2.h>
#include <gpslam/gp/GaussianProcessFactorBaseRot3.h>

#include <gpslam/gp/GaussianProcessPriorLinear.h>
#include <gpslam/gp/GaussianProcessPriorPose3.h>
#include <gpslam/gp/GaussianProcessPriorPose3VW.h>
#include <gpslam/gp/GaussianProcessPriorPose2.h>
#include <gpslam/gp/GaussianProcessPriorRot3.h>
*/
#include <gtsam/base/serializationTestHelpers.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gpslam;
using namespace gtsam::serializationTestHelpers;


BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");

/* ************************************************************************** */
// data
static SharedNoiseModel Qcmodel_6 = noiseModel::Isotropic::Sigma(6, 0.1);
static SharedNoiseModel Qcmodel_3 = noiseModel::Isotropic::Sigma(3, 0.1);

// bases
static GaussianProcessInterpolatorLinear<6> gp_base_linear(Qcmodel_6, 0.1, 0.04);
/*
static GaussianProcessFactorBasePose3 gp_base_pose3(Qcmodel_6, 0.1, 0.04);
static GaussianProcessFactorBasePose3VW gp_base_pose3vw(Qcmodel_6, 0.1, 0.04);
static GaussianProcessFactorBasePose2 gp_base_pose2(Qcmodel_3, 0.1, 0.04);
static GaussianProcessFactorBaseRot3 gp_base_rot3(Qcmodel_3, 0.1, 0.04);

// factors
static GaussianProcessPriorLinear<6> gp_prior_linear(1, 2, 3, 4, 0.1, Qcmodel_6);
static GaussianProcessPriorPose3 gp_prior_pose3(1, 2, 3, 4, 0.1, Qcmodel_6);
static GaussianProcessPriorPose3VW gp_prior_pose3_vw(1, 2, 3, 4, 5, 6, 0.1, Qcmodel_6);
static GaussianProcessPriorPose2 gp_prior_pose2(1, 2, 3, 4, 0.1, Qcmodel_3);
static GaussianProcessPriorRot3 gp_prior_rot3(1, 2, 3, 4, 0.1, Qcmodel_3);
*/

/* ************************************************************************** */
TEST_UNSAFE(SerializationGP, text) {
  EXPECT(equalsObj(gp_base_linear));
  /*
  EXPECT(equalsObj(gp_base_pose3));
  EXPECT(equalsObj(gp_base_pose3vw));
  EXPECT(equalsObj(gp_base_pose2));
  EXPECT(equalsObj(gp_base_rot3));

  EXPECT(equalsObj(gp_prior_linear));
  EXPECT(equalsObj(gp_prior_pose3));
  EXPECT(equalsObj(gp_prior_pose3_vw));
  EXPECT(equalsObj(gp_prior_pose2));
  EXPECT(equalsObj(gp_prior_rot3));
  */
}

/* ************************************************************************** */
TEST_UNSAFE(SerializationGP, xml) {
  EXPECT(equalsXML(gp_base_linear));
  /*
  EXPECT(equalsXML(gp_base_pose3));
  EXPECT(equalsXML(gp_base_pose3vw));
  EXPECT(equalsXML(gp_base_pose2));
  EXPECT(equalsXML(gp_base_rot3));

  EXPECT(equalsXML(gp_prior_linear));
  EXPECT(equalsXML(gp_prior_pose3));
  EXPECT(equalsXML(gp_prior_pose3_vw));
  EXPECT(equalsXML(gp_prior_pose2));
  EXPECT(equalsXML(gp_prior_rot3));
  */
}

/* ************************************************************************** */
TEST_UNSAFE(SerializationGP, binary) {
  EXPECT(equalsBinary(gp_base_linear));
  /*
  EXPECT(equalsBinary(gp_base_pose3));
  EXPECT(equalsBinary(gp_base_pose3vw));
  EXPECT(equalsBinary(gp_base_pose2));
  EXPECT(equalsBinary(gp_base_rot3));

  EXPECT(equalsBinary(gp_prior_linear));
  EXPECT(equalsBinary(gp_prior_pose3));
  EXPECT(equalsBinary(gp_prior_pose3_vw));
  EXPECT(equalsBinary(gp_prior_pose2));
  EXPECT(equalsBinary(gp_prior_rot3));
  */
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
