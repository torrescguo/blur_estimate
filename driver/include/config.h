#pragma once

#include "const_xslam.h"
#include "ccommon.h"

#define XSLAM_DATA_FOLDER "../data/"


  extern Float MAPPING_ADD_P3D_THRESHOLD;
  extern Float LOC_INLIER_THRESHOLD;
  extern int COMPUTE_POSE_MINIMUM_REQUIRED_FEATURES;

  extern int MATCH_RADIUS_LOC_1;
  extern int MATCH_RADIUS_LOC_2;
  extern int MAP_MATCHING_PREVIOUS_1;
  extern int MAP_MATCHING_PREVIOUS_2;
  extern int MAP_STEREO_MATCHING;
  extern Float COMPARE_DESCRIPTOR_THRESHOLD;
  extern int MINIMUM_INLIER_TO_BE_LOCALIZED;

  extern Float EPIPOLAR_DISTANCE;

  extern Float IMU_PERIOD;// 1.0f / IMU_FREQUENCY(125)
  extern Float CAMERA_PERIOD;
  // static const Float IMU_PERIOD = 0.0025;// 1.0f / IMU_FREQUENCY(400)

  extern int DETECTOR_TARGET_FEATURES;
  extern int DETECTOR_THRESHOLD_LOWER_LIMIT;
  extern int DETECTOR_THRESHOLD_UPPER_LIMIT;

void initConfigData(slam_config_t *slam_config_datas);