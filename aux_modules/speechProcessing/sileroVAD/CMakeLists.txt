################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################
project(sileroVAD)
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(AUX_NAME sileroVAD)
add_executable(${AUX_NAME})

set(ONNX_PATH $ENV{ONNX_PATH})

# Check if the environment variable is set
if (NOT ONNX_PATH)
    message(WARNING "ONNX_PATH is not set, using default value.")
    set(ONNX_PATH "/usr/local/src/robot/onnxruntime-linux-x64-1.20.1")
endif()

target_include_directories(${AUX_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    $<TARGET_PROPERTY:WakeMsgs,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:SileroVADMsgs,INTERFACE_INCLUDE_DIRECTORIES>
    ${ONNX_PATH}/include
)
target_sources(${AUX_NAME}
  PRIVATE
    main.cpp
    Detector.h
    Detector.cpp
    SileroVADServer.cpp
    VoiceActivationDetectionModule.h
    VoiceActivationDetectionModule.cpp
)


target_sources(${AUX_NAME}
  PRIVATE
    ${STATECHARTS_FILES})

target_link_libraries(${AUX_NAME}
  PRIVATE
    ${YARP_LIBRARIES}
    WakeMsgs
    SileroVADMsgs
    ${ONNX_PATH}/lib/libonnxruntime.so)


install(TARGETS ${AUX_NAME} DESTINATION bin)
