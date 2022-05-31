################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

set(appname openPoseStuff )

file(GLOB conf      ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB apps      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install(FILES ${conf}    DESTINATION ${BEHAVIOR-TOUR-ROBOT_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${apps}    DESTINATION ${BEHAVIOR-TOUR-ROBOT_APPLICATIONS_INSTALL_DIR})
