################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################
project(speechMicController)
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(AUX_NAME speechMicController)
add_executable(${AUX_NAME})

target_include_directories(${AUX_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    $<TARGET_PROPERTY:WakeMsgs,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:SileroVADMsgs,INTERFACE_INCLUDE_DIRECTORIES>
)
target_sources(${AUX_NAME}
  PRIVATE
    main.cpp
    SpeechMicController.cpp
    SpeechMicController.h
)


target_sources(${AUX_NAME}
  PRIVATE
    ${STATECHARTS_FILES})

target_link_libraries(${AUX_NAME}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_init)


install(TARGETS ${AUX_NAME} DESTINATION bin)
