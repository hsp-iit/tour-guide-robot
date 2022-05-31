################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# Create the BT_protocol C++ Library
add_library(google_synthesis STATIC)
yarp_add_idl(IDL_GEN_FILES google_synthesis.thrift)
target_sources(google_synthesis
  PRIVATE
    ${IDL_GEN_FILES})
target_include_directories(google_synthesis
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(google_synthesis
  PRIVATE
    YARP::YARP_os)
