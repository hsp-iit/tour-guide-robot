################################################################################
#                                                                              #
# Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

set(appname headSynchronizer)

file(GLOB apps      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB conf      ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB robotInterface      ${CMAKE_CURRENT_SOURCE_DIR}/conf/robotInterface/*.xml)

yarp_install(FILES ${apps}    DESTINATION ${BEHAVIOR-TOUR-ROBOT_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES ${conf}    DESTINATION ${BEHAVIOR-TOUR-ROBOT_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${robotInterface}    DESTINATION ${BEHAVIOR-TOUR-ROBOT_CONTEXTS_INSTALL_DIR}/${appname}/robotInterface)
