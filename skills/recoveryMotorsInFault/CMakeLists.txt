################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

set(SKILL_NAME bt_recoveryMotorsInFault)
add_executable(${SKILL_NAME})
target_include_directories(${SKILL_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)
target_sources(${SKILL_NAME}
  PRIVATE
    ${SKILL_NAME}.cpp
    ${SKILL_NAME}.h
    main.cpp
)

target_sources(${SKILL_NAME}
  PRIVATE
    ${STATECHARTS_FILES})

target_link_libraries(${SKILL_NAME}
  PRIVATE
    YARP::YARP_os
    YARP::YARP_init
    skill_interface
    tourManagerRPC)
install(TARGETS ${SKILL_NAME} DESTINATION bin)
