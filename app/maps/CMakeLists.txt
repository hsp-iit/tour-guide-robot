################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

set(appname maps)

file(GLOB maps ${CMAKE_CURRENT_SOURCE_DIR}/*.*)

yarp_install(FILES ${maps}    DESTINATION ${BEHAVIOR-TOUR-ROBOT_CONTEXTS_INSTALL_DIR}/maps)
