################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################
project(voiceActivationDetection)
set(AUX_NAME voiceActivationDetection)
add_executable(${AUX_NAME})

find_package(YARP COMPONENTS os dev REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(libfvad IMPORTED_TARGET libfvad)

add_subdirectory(messages)

target_include_directories(${AUX_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)
target_sources(${AUX_NAME}
  PRIVATE
    main.cpp
    Detector.h
    Detector.cpp
    VADServer.h
    VADServer.cpp
    VoiceActivationDetectionModule.h
    VoiceActivationDetectionModule.cpp
)


target_sources(${AUX_NAME}
  PRIVATE
    ${STATECHARTS_FILES})

target_link_libraries(${AUX_NAME}
  PRIVATE
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_init
    PkgConfig::libfvad
    VADMsgs
    )
