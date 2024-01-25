# *************************************************************************************************************************************************************
# This software and all the related documentation that are transmitted as a final report constitute Foregrund of the Research Agreement between IIT and Konica
# and their use and exploitation are subject to the limitations contained therein.
# *************************************************************************************************************************************************************

set(PROJECTNAME crowdDetector)
project(${PROJECTNAME})

# YARP needs CMake 3.0 or greater
cmake_minimum_required(VERSION 3.12)

# find YARP
find_package(YARP COMPONENTS os REQUIRED)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

find_package( OpenCV REQUIRED )
find_package(YARP REQUIRED COMPONENTS sig cv dev os math)
include_directories(${OpenCV_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})
#add_executable(${PROJECT_NAME} ${folder_source})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)