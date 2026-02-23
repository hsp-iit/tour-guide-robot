################################################################################
#                                                                              #
# Copyright (C) 2024 Fondazione Istituto Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

file(GLOB ros2_launch ${CMAKE_CURRENT_SOURCE_DIR}/ros2_launch.sh)
file(GLOB ros2_run ${CMAKE_CURRENT_SOURCE_DIR}/ros2_run.sh)
file(GLOB roam ${CMAKE_CURRENT_SOURCE_DIR}/roam.sh)
file(GLOB uv_run ${CMAKE_CURRENT_SOURCE_DIR}/uv_run.sh)


yarp_install(FILES ${ros2_launch} DESTINATION bin)
yarp_install(FILES ${ros2_run} DESTINATION bin)
yarp_install(FILES ${roam} DESTINATION bin)
yarp_install(FILES ${uv_run} DESTINATION bin)