add_subdirectory(messages)
if(WAKE_WORD)
    add_subdirectory(wakeWordDetection)
endif()

if(SILERO_VAD)
    add_subdirectory(sileroVAD)
endif()

add_subdirectory(openWakeWord)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libfvad IMPORTED_TARGET libfvad)
if(${libfvad_FOUND})
    add_subdirectory(voiceActivationDetection)
endif()