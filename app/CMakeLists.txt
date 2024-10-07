################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

add_subdirectory(headSynchronizer)
add_subdirectory(voiceActivationDetection)
add_subdirectory(maps)
add_subdirectory(maps_convince_cris)
add_subdirectory(btStructure)
add_subdirectory(google)
add_subdirectory(navigation2)
add_subdirectory(openPoseStuff)
add_subdirectory(tourManager)
add_subdirectory(speechPipeline)
add_subdirectory(llmTest)
add_subdirectory(VLNav)
add_subdirectory(yarpmanager)
add_subdirectory(Dialogue_demo)

file(GLOB apps      ${CMAKE_CURRENT_SOURCE_DIR}/*.xml)

yarp_install(FILES ${apps}    DESTINATION ${TOUR-GUIDE-ROBOT_APPLICATIONS_INSTALL_DIR})
