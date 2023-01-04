################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

set(appname navigation2)

file(GLOB conf      ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB robotInterface      ${CMAKE_CURRENT_SOURCE_DIR}/conf/robotInterface/*.xml)
file(GLOB apps      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB launch      ${CMAKE_CURRENT_SOURCE_DIR}/launch/*.launch)

yarp_install(FILES ${conf}    DESTINATION ${TOUR-GUIDE-ROBOT_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${launch}    DESTINATION ${TOUR-GUIDE-ROBOT_CONTEXTS_INSTALL_DIR}/${appname}/launch)
yarp_install(FILES ${robotInterface}    DESTINATION ${TOUR-GUIDE-ROBOT_CONTEXTS_INSTALL_DIR}/${appname}/robotInterface)
yarp_install(FILES ${apps}    DESTINATION ${TOUR-GUIDE-ROBOT_APPLICATIONS_INSTALL_DIR})
