################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################
project(voiceActivationDetection)
set(AUX_NAME voiceActivationDetection)
add_executable(${AUX_NAME})
target_include_directories(${AUX_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    $<TARGET_PROPERTY:WakeMsgs,INTERFACE_INCLUDE_DIRECTORIES>
)
target_sources(${AUX_NAME}
  PRIVATE
    main.cpp
    Detector.h
    Detector.cpp
    VoiceActivationDetectionModule.h
    VoiceActivationDetectionModule.cpp
)


target_sources(${AUX_NAME}
  PRIVATE
    ${STATECHARTS_FILES})

target_link_libraries(${AUX_NAME}
  PRIVATE
    ${YARP_LIBRARIES}
    PkgConfig::libfvad
    WakeMsgs)

install(TARGETS ${AUX_NAME} DESTINATION bin)
