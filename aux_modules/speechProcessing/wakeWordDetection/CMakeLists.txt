set(MODULE_NAME wakeWordDetection)
add_executable(${MODULE_NAME})

find_package(YARP COMPONENTS os dev REQUIRED)

target_include_directories(${MODULE_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    $<TARGET_PROPERTY:VADMsgs,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:WakeMsgs,INTERFACE_INCLUDE_DIRECTORIES>
)

target_sources(${MODULE_NAME}
  PRIVATE
    WakeWordModule.cpp
    WakeServer.cpp
    AudioCallback.cpp
    main.cpp
)

target_sources(${MODULE_NAME}
  PRIVATE
    ${STATECHARTS_FILES})

target_link_libraries(${MODULE_NAME}
  PRIVATE
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_init
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/x86_64/libpv_porcupine.so
    VADMsgs
    WakeMsgs
    )
    
install(TARGETS ${MODULE_NAME} DESTINATION bin)