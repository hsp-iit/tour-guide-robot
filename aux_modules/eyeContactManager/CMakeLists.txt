################################################################################
#                                                                              #
# Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

set(AUX_NAME eyeContactManager)
add_executable(${AUX_NAME})
target_include_directories(${AUX_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)
target_sources(${AUX_NAME}
  PRIVATE
    ${AUX_NAME}.cpp
    ${AUX_NAME}.h
    main.cpp
)

target_sources(${AUX_NAME}
  PRIVATE
    ${STATECHARTS_FILES})

target_link_libraries(${AUX_NAME}
  PRIVATE
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_init
    headSynchronizerRPC
    )
install(TARGETS ${AUX_NAME} DESTINATION bin)
