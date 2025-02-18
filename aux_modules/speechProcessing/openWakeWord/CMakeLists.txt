cmake_minimum_required(VERSION 3.1)
project(OpenWakeWordInstaller)

# Define the directory and file path for the Python script
set(OWW_DIR "${CMAKE_SOURCE_DIR}/aux_modules/speechProcessing/openWakeWord")
set(PYTHON_SCRIPT "${OWW_DIR}/oww.py")

# Verify that the Python script exists
if(NOT EXISTS ${PYTHON_SCRIPT})
  message(FATAL_ERROR "File not found: ${PYTHON_SCRIPT}")
endif()

# Define destination folder inside the build directory
set(BIN_DESTINATION "${CMAKE_BINARY_DIR}/bin")
file(MAKE_DIRECTORY ${BIN_DESTINATION})

# Custom target: Copy the Python script to build/bin after every build
add_custom_target(copy_openWakeWord ALL
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PYTHON_SCRIPT} ${BIN_DESTINATION}
  COMMENT "Copying oww.py to ${BIN_DESTINATION}"
)

install(PROGRAMS ${PYTHON_SCRIPT} DESTINATION bin)
