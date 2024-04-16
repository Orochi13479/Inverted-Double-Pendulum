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

include(FetchContent)
FetchContent_Declare(
  cactus_rt
  GIT_REPOSITORY https://github.com/cactusdynamics/cactus-rt.git
  GIT_TAG        efb41a5f4c59223bded1ad367eb0908ac8e147e2
)

FetchContent_MakeAvailable(cactus_rt)

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/opt/openrobots")

find_package(pinocchio)

add_executable(manualTorque_${PROJECT_NAME} driver/manualTorque.cpp)
target_link_libraries(manualTorque_${PROJECT_NAME} moteus::cpp pthread) 

add_executable(rtDriver_${PROJECT_NAME} driver/rtDriver.cpp)
target_link_libraries(rtDriver_${PROJECT_NAME} moteus::cpp pthread cactus_rt) 

add_executable(driver_${PROJECT_NAME} driver/driver.cpp)
target_link_libraries(driver_${PROJECT_NAME} moteus::cpp pthread) 

add_executable(main_${PROJECT_NAME} main.cpp IDPDriver.cpp IDPControl.cpp)
target_link_libraries(main_${PROJECT_NAME} pinocchio::pinocchio moteus::cpp pthread)
