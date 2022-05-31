################################################################################
#                                                                              #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

set(AUX_NAME googleLogger)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_executable(${AUX_NAME} ${folder_source} ${folder_header})

target_include_directories(${AUX_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_sources(${AUX_NAME}
  PRIVATE
    ${STATECHARTS_FILES})

target_link_libraries(${AUX_NAME}
  PRIVATE
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_init
    tourManagerRPC
)
install(TARGETS ${AUX_NAME} DESTINATION bin)
