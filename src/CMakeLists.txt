#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

add_subdirectory(gaze-ctrl-lib)

option(BUILD_ARUKO_POSE_ESTIMATION "Build aruko-pose-estimation" OFF)
if (BUILD_ARUKO_POSE_ESTIMATION)
  add_subdirectory(aruko-pose-estimation)
endif()

option(BUILD_CAMERA_CALIBRATION_TEST "Build camera-calibration-test" OFF)
if (BUILD_CAMERA_CALIBRATION_TEST)
  add_subdirectory(camera-calibration-test)
endif()

option(BUILD_REACHING_TEST "Build reaching-test" OFF)
if (BUILD_REACHING_TEST)
  add_subdirectory(reaching-test)
endif()
