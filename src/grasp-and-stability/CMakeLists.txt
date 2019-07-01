#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================


set(EXE_TARGET_NAME grasp-and-stability)

# YARP
find_package(YARP CONFIG REQUIRED
             COMPONENTS
             OS
             sig
             )
# iCub
find_package(ICUB REQUIRED)

# Application source files
set(${EXE_TARGET_NAME}_SRC
    src/main.cpp
    src/pugixml.cpp
    )
set(${EXE_TARGET_NAME}_THRIFT_HDR src/idl.thrift)

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC}  ${${EXE_TARGET_NAME}_THRIFT_SRC})

target_include_directories(${EXE_TARGET_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")


target_link_libraries(${EXE_TARGET_NAME} PRIVATE
                                         YARP::YARP_init
                                         YARP::YARP_OS
                                         YARP::YARP_sig
                                         iKin
                                         )

 set(${EXE_TARGET_NAME}_CONF ${CMAKE_CURRENT_SOURCE_DIR}/conf/config.ini
                             )


 yarp_install(FILES ${${EXE_TARGET_NAME}_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/aruko-pose-estimation)

 install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
