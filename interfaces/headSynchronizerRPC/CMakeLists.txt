################################################################################
#                                                                              #
# Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# Create the headSynchronizerRPC C++ Library
add_library(headSynchronizerRPC STATIC)
yarp_add_idl(IDL_GEN_FILES headSynchronizerRPC.thrift)
target_sources(headSynchronizerRPC
  PRIVATE
    ${IDL_GEN_FILES})
target_include_directories(headSynchronizerRPC
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(headSynchronizerRPC
  PRIVATE
    YARP::YARP_os)
