# Minimum Version Required Due to FetchContent
cmake_minimum_required(VERSION 3.11)

project(InvertedDoublePendulum)

include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        d37918b4286d8654dff320a250534bb901edfa8f
)

FetchContent_MakeAvailable(moteus)

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/opt/openrobots")

find_package(pinocchio)

add_executable(manualTorque_${PROJECT_NAME} driver/manualTorque.cpp)
target_link_libraries(manualTorque_${PROJECT_NAME} moteus::cpp pthread) 

add_executable(rtDriver_${PROJECT_NAME} driver/rtDriver.cpp)
target_link_libraries(rtDriver_${PROJECT_NAME} moteus::cpp pthread) 

add_executable(main_${PROJECT_NAME} main.cpp IDPDriver.cpp IDPControl.cpp)
target_link_libraries(main_${PROJECT_NAME} pinocchio::pinocchio moteus::cpp pthread) 
