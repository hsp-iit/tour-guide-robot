################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# Create the BT_protocol C++ Library
add_library(skill_interface STATIC)
yarp_add_idl(IDL_GEN_FILES BT_wrappers.thrift)
target_sources(skill_interface
  PRIVATE
    ${IDL_GEN_FILES})
target_include_directories(skill_interface
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(skill_interface
  PRIVATE
    YARP::YARP_os)
