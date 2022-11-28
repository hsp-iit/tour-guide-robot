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
find_package(PkgConfig REQUIRED)
pkg_check_modules(libfvad IMPORTED_TARGET libfvad)
if(${libfvad_FOUND})
    add_subdirectory(voiceActivationDetection)
endif()
if(${OPENCV_ON})
    add_subdirectory(headObstaclesScanner)
    add_subdirectory(faceExpression)
    add_subdirectory(crowdDetector)
endif()