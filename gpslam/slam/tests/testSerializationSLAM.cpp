/**
*  @file testSerialization.cpp
*  @brief test serialization of all factors
*  @author Jing Dong
**/

#include <gpslam/slam/GPInterpolatedGPSFactorPose3.h>
#include <gpslam/slam/GPInterpolatedGPSFactorPose3VW.h>
#include <gpslam/slam/GPInterpolatedProjectionFactorPose3.h>
#include <gpslam/slam/GPInterpolatedRangeFactorPose3.h>

#include <gpslam/slam/OdometryFactor2DLinear.h>
#include <gpslam/slam/RangeBearingFactor2DLinear.h>
#include <gpslam/slam/RangeFactor2DLinear.h>
#include <gpslam/slam/RangeBearingFactor2DLinear.h>

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
static SharedNoiseModel unit1 = noiseModel::Unit::Create(1);
static SharedNoiseModel unit2 = noiseModel::Unit::Create(2);
static SharedNoiseModel unit3 = noiseModel::Unit::Create(3);
static SharedNoiseModel Qcmodel_6 = noiseModel::Isotropic::Sigma(6, 0.1);
static Cal3_S2::shared_ptr K(new Cal3_S2());

// factors
static GPInterpolatedGPSFactorPose3 gp_gps_pose3(Point3(0.3, 0.6, 0.9), unit2, Qcmodel_6, 1, 2, 3, 4, 0.1, 0.04);
static GPInterpolatedGPSFactorPose3VW gp_gps_pose3vw(Point3(0.3, 0.6, 0.9), unit2,
    Qcmodel_6, 1, 2, 3, 4, 5, 6, 0.1, 0.04);
static GPInterpolatedProjectionFactorPose3<Cal3_S2> gp_proj(Point2(10, 20), unit2,
    Qcmodel_6, 1, 2, 3, 4, 5, 0.1, 0.04, K);
static GPInterpolatedRangeFactorPose3 gp_range(10.0, unit1, Qcmodel_6, 1, 2, 3, 4, 5, 0.1, 0.04);

static OdometryFactor2DLinear odom_linear2(1, 2, Vector3(0.1, 0.2, 3.0), unit3);
static RangeFactor2DLinear range_linear2(1, 2, 10.0, unit1);
static RangeBearingFactor2DLinear rangebearing_linear2(1, 2, 0.1, 10.0, unit2);

/* ************************************************************************** */
TEST(SerializationSLAM, text) {
  EXPECT(equalsObj(gp_gps_pose3));
  EXPECT(equalsObj(gp_gps_pose3vw));
  EXPECT(equalsObj(gp_proj));
  EXPECT(equalsObj(gp_range));

  EXPECT(equalsObj(odom_linear2));
  EXPECT(equalsObj(range_linear2));
  EXPECT(equalsObj(rangebearing_linear2));
}

/* ************************************************************************** */
TEST(SerializationSLAM, xml) {
  EXPECT(equalsXML(gp_gps_pose3));
  EXPECT(equalsXML(gp_gps_pose3vw));
  EXPECT(equalsXML(gp_proj));
  EXPECT(equalsXML(gp_range));

  EXPECT(equalsXML(odom_linear2));
  EXPECT(equalsXML(range_linear2));
  EXPECT(equalsXML(rangebearing_linear2));
}

/* ************************************************************************** */
TEST(SerializationSLAM, binary) {
  EXPECT(equalsBinary(gp_gps_pose3));
  EXPECT(equalsBinary(gp_gps_pose3vw));
  EXPECT(equalsBinary(gp_proj));
  EXPECT(equalsBinary(gp_range));

  EXPECT(equalsBinary(odom_linear2));
  EXPECT(equalsBinary(range_linear2));
  EXPECT(equalsBinary(rangebearing_linear2));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
