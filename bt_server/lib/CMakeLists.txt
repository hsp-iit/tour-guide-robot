################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# Create the yarp_bt_nodes library
add_library(yarp_bt_nodes STATIC)
target_sources(yarp_bt_nodes
  PRIVATE
    yarp_action.cpp
    yarp_action.h
    yarp_condition.cpp
    yarp_condition.h
    yarp_node.cpp
    yarp_node.h)
target_include_directories(yarp_bt_nodes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(yarp_bt_nodes
  PUBLIC
    YARP::YARP_os
    YARP::YARP_init
    ament_index_cpp::ament_index_cpp
    BT::behaviortree_cpp_v3
    skill_interface)
