################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

add_subdirectory(lib)

# Create the BT executable
add_executable(bt_server)
target_sources(bt_server
  PRIVATE
  main.cpp)
target_link_libraries(bt_server
  PRIVATE
    YARP::YARP_os
    YARP::YARP_init
    ament_index_cpp::ament_index_cpp
    BT::behaviortree_cpp_v3
    yarp_bt_nodes
    pthread)

install(TARGETS bt_server DESTINATION bin)
