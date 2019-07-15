#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME aruco-pose-estimation)

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             sig
             cv
             )

# OpenCV
find_package(OpenCV REQUIRED)

# iCub
find_package(ICUB REQUIRED)

# Application source files
set(${EXE_TARGET_NAME}_SRC
    src/main.cpp
    )

set(${EXE_TARGET_NAME}_THRIFT_HDR src/idl.thrift)
yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_THRIFT_SRC})

target_include_directories(${EXE_TARGET_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")


target_link_libraries(${EXE_TARGET_NAME} PRIVATE
                                         YARP::YARP_init
                                         YARP::YARP_OS
                                         YARP::YARP_sig
                                         YARP::YARP_cv
                                         iCubGazeCtrlLibrary
                                         ${OpenCV_LIBS}
                                         )

set(${EXE_TARGET_NAME}_CONF ${CMAKE_CURRENT_SOURCE_DIR}/conf/config_base.ini
                            ${CMAKE_CURRENT_SOURCE_DIR}/conf/config_hand.ini
                            ${CMAKE_CURRENT_SOURCE_DIR}/conf/sfm_config.ini
                            ${CMAKE_CURRENT_SOURCE_DIR}/conf/opencv_icub_intrinsics_left
                            ${CMAKE_CURRENT_SOURCE_DIR}/conf/opencv_icub_intrinsics_right
                            )

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/aruco-pose-estimation)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
