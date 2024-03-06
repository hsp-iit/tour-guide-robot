################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

file(GLOB map_office ${CMAKE_CURRENT_SOURCE_DIR}/*.*)

yarp_install(FILES ${map_office}    DESTINATION ${TOUR-GUIDE-ROBOT_CONTEXTS_INSTALL_DIR}/maps/SIM_SMALL_OFFICE)