################################################################################
#                                                                              #
# Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

set(appname btStructure)

file(GLOB conf            ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB apps            ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB trees           ${CMAKE_CURRENT_SOURCE_DIR}/trees/*.xml)
file(GLOB robotInterface  ${CMAKE_CURRENT_SOURCE_DIR}/conf/robotInterface/*.xml)

yarp_install(FILES ${conf}    DESTINATION ${TOUR-GUIDE-ROBOT_CONTEXTS_INSTALL_DIR}/${appname})
# For trees xml's and help files it would be nice to have two different folders (like for Contexts and applications)
yarp_install(FILES ${trees}    DESTINATION ${TOUR-GUIDE-ROBOT_CONTEXTS_INSTALL_DIR}/${appname}/trees)
yarp_install(FILES ${apps}    DESTINATION ${TOUR-GUIDE-ROBOT_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES ${robotInterface}    DESTINATION ${TOUR-GUIDE-ROBOT_CONTEXTS_INSTALL_DIR}/${appname}/robotInterface)
