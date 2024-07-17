cmake_minimum_required(VERSION 3.12)

project(tour-guide-robot LANGUAGES CXX)

set(BTState OFF CACHE BOOL OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(BASE_INCOHERENT_TO_FAULT "if enabled, the bt_motorsNotInFault module will consider a fault every pair of different controle modes for the robot base wheels motors" OFF)
if(BASE_INCOHERENT_TO_FAULT)
	add_definitions(-DBASE_INCOHERENT_TO_FAULT=1)
endif()

option(WAKE_WORD "if enabled, the wakeWordDetection will be compiled" OFF)

if(${BTState})
	# BehaviorTree.CPP links Boost::coroutine, but does not look for it
	find_package(Boost COMPONENTS coroutine REQUIRED)
	find_package(ament_index_cpp)
	find_package(behaviortree_cpp_v3 REQUIRED)
endif()
find_package(YCM REQUIRED)

# Checking if opencv4 is installed (for python3 and ubuntu 20.04) and including the required directory for compatibility. Else, continue as it was.
find_package (OpenCV)
if (${OpenCV_FOUND})
	set(OPENCV_ON ON)
	if (IS_DIRECTORY /usr/include/opencv4/)
		include_directories ("/usr/include/opencv4/")
	endif ()
endif()

find_package(YARP COMPONENTS os idl_tools dev REQUIRED)
find_package(navigation)

yarp_configure_external_installation(${PROJECT_NAME} WITH_PLUGINS)

if(${BTState})
	add_subdirectory(bt_server)
	add_subdirectory(skills)
endif()
add_subdirectory(interfaces)
add_subdirectory(aux_modules)
add_subdirectory(app)
add_subdirectory(bin)
