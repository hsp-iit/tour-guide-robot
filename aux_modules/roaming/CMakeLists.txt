project(roaming)
set(ROAMING_SERVER roaming_server)
set(ROAMING_CLIENT roaming_client)

## Roaming server configuration
add_executable(${ROAMING_SERVER})
target_include_directories(${ROAMING_SERVER} PUBLIC include)
target_sources(${ROAMING_SERVER} PRIVATE 
    src/mainServer.cpp
    src/RoamingServer.cpp 
    src/LinuxNetworkInteraction.cpp
    include/LinuxNetworkInteraction.h
    include/RoamingServer.h)
target_link_libraries(${ROAMING_SERVER} PRIVATE ${YARP_LIBRARIES})
if(${ENABLE_TESTS})
    add_subdirectory(test)
endif()


## Roaming client configuration
find_library(wpa libwpa_client.so HINTS /opt/wpa_supplicant-2.10/wpa_supplicant)
if(${wpa} MATCHES wpa-NOTFOUND)
    message("-- Cannot build roaming client")
else()
    add_executable(${ROAMING_CLIENT})
    cmake_path(GET wpa PARENT_PATH WPA_PATH)
    cmake_path(GET WPA_PATH PARENT_PATH WPA_PARENT_PATH)
    target_include_directories(${ROAMING_CLIENT} PUBLIC
        include
        ${WPA_PARENT_PATH}/src/common
        ${WPA_PARENT_PATH}/src/utils
    )
    target_sources(${ROAMING_CLIENT} PRIVATE
        src/mainClient.cpp
        src/RoamingClient.cpp
        src/WpaSupplicantRoaming.cpp
        include/RoamingClient.h
    )
    target_link_libraries(${ROAMING_CLIENT} PUBLIC
        ${YARP_LIBRARIES}
        ${wpa}
    )
endif()
