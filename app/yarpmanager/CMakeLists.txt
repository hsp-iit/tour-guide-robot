################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

set(appname yarpmanager_tour)

file(GLOB r1001 ${CMAKE_CURRENT_SOURCE_DIR}/conf/R1SN001/*.*)
file(GLOB r1003 ${CMAKE_CURRENT_SOURCE_DIR}/conf/R1SN003/*.*)
file(GLOB apps ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install(FILES ${r1001}    DESTINATION ${TOUR-GUIDE-ROBOT_CONTEXTS_INSTALL_DIR}/R1SN001)
yarp_install(FILES ${r1003}    DESTINATION ${TOUR-GUIDE-ROBOT_CONTEXTS_INSTALL_DIR}/R1SN003)
yarp_install(FILES ${apps}    DESTINATION ${TOUR-GUIDE-ROBOT_APPLICATIONS_INSTALL_DIR})