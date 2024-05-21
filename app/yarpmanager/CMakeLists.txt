################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

set(appname yarpmanager_tour)

file(GLOB r1001 ${CMAKE_CURRENT_SOURCE_DIR}/conf/R1SN001/*.*)

yarp_install(FILES ${r1001}    DESTINATION ${TOUR-GUIDE-ROBOT_CONTEXTS_INSTALL_DIR}/R1SN001)