################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

add_subdirectory(headSynchronizer)
add_subdirectory(voiceActivationDetection)
add_subdirectory(maps)
add_subdirectory(btStructure)
add_subdirectory(google)
add_subdirectory(navigation2)
add_subdirectory(openPoseStuff)
add_subdirectory(tourManager)

file(GLOB apps      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install(FILES ${apps}    DESTINATION ${BEHAVIOR-TOUR-ROBOT_APPLICATIONS_INSTALL_DIR})