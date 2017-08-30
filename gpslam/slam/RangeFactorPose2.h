/**
 *  @file  RangeFactorPose2.h
 *  @brief range factor pose2
 *  @author Jing Dong
 **/

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/sam/RangeFactor.h>

namespace gpslam {

typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2, double> RangeFactorPose2;

} // namespace gpslam
