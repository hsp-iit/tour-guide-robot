################################################################################
#                                                                              #
# Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

add_subdirectory(headSynchronizer)
add_subdirectory(googleLogger)
add_subdirectory(tourManager)
add_subdirectory(eyeContactManager)
if(0)
    add_subdirectory(speechProcessing)
endif()
if(${OPENCV_ON})
    add_subdirectory(headObstaclesScanner)
    add_subdirectory(faceExpression)
    add_subdirectory(crowdDetector)
endif()
