################################################################################
#                                                                              #
# Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

set(appname headSynchronizer)

file(GLOB apps      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install(FILES ${apps}    DESTINATION ${BEHAVIOR-TOUR-ROBOT_APPLICATIONS_INSTALL_DIR})
