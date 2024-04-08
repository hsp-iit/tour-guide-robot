Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.4.0 # or a later release
)

FetchContent_MakeAvailable(Catch2)

project(roaming_test)
add_executable(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC ../include)
target_include_directories(${PROJECT_NAME} PUBLIC .)
target_sources(${PROJECT_NAME} PRIVATE 
  test.cpp 
  ../src/roaming.cpp 
  FakeNetworkInteraction.h
  ../include/roaming.h
  ../include/INetworkInteraction.h)
target_link_libraries(${PROJECT_NAME} PRIVATE ${YARP_LIBRARIES} Catch2::Catch2WithMain)