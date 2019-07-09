#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

add_subdirectory(gaze-ctrl-lib)

option(BUILD_ARUCO_POSE_ESTIMATION "Build aruco-pose-estimation" ON)
if (BUILD_ARUCO_POSE_ESTIMATION)
  add_subdirectory(aruco-pose-estimation)
endif()

option(BUILD_CAMERA_CALIBRATION_TEST "Build camera-calibration-test" ON)
if (BUILD_CAMERA_CALIBRATION_TEST)
  add_subdirectory(camera-calibration-test)
endif()

option(BUILD_REACHING_TEST "Build reaching-test" ON)
if (BUILD_REACHING_TEST)
  add_subdirectory(reaching-test)
endif()

option(BUILD_GRASP_AND_STABILITY "Build grasp-and-stability" ON)
if (BUILD_GRASP_AND_STABILITY)
  add_subdirectory(grasp-and-stability)
endif()
