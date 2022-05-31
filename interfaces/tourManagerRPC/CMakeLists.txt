################################################################################
#                                                                              #
# Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# Create the tourManagerRPC C++ Library
add_library(tourManagerRPC STATIC)
yarp_add_idl(IDL_GEN_FILES tourManagerRPC.thrift)
target_sources(tourManagerRPC
  PRIVATE
    ${IDL_GEN_FILES})
target_include_directories(tourManagerRPC
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(tourManagerRPC
  PRIVATE
    YARP::YARP_os)
