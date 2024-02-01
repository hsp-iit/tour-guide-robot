cmake_minimum_required(VERSION 3.16)
project(thriftEditor_example)
 
# Needed to link the static library from the plugin, on 64 bit
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
 
find_package(YARP COMPONENTS os idl_tools REQUIRED)
 
add_library(VADMsgs STATIC)
 
yarp_idl_to_dir(
  INPUT_FILES VADMsgs.thrift
  OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/VADMsgs
  SOURCES_VAR IDL_GEN_SRCS
  HEADERS_VAR IDL_GEN_HDRS
  INCLUDE_DIRS_VAR IDL_INCLUDE_DIRS
  PLACEMENT MERGED
)
 
target_sources(VADMsgs
  PRIVATE
    ${IDL_GEN_SRCS}
    ${IDL_GEN_HDRS}
)
 
target_include_directories(VADMsgs
  PUBLIC
    $<BUILD_INTERFACE:${IDL_INCLUDE_DIRS}>
)
 
target_link_libraries(VADMsgs
  PRIVATE
    YARP::YARP_os)