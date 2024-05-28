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

# Main Executable
add_executable(main_${PROJECT_NAME} main.cpp)
target_link_libraries(main_${PROJECT_NAME} pinocchio::pinocchio moteus::cpp pthread cactus_rt)

setup_cactus_rt_target_options(main_${PROJECT_NAME})


# Control Executables
add_executable(control_${PROJECT_NAME} control/IDPControlPosition.cpp)
target_link_libraries(control_${PROJECT_NAME} pinocchio::pinocchio moteus::cpp pthread)

add_executable(csvReaderTest_${PROJECT_NAME} control/csvReader.cpp)
target_link_libraries(csvReaderTest_${PROJECT_NAME} pinocchio::pinocchio moteus::cpp pthread)

add_executable(rnea_${PROJECT_NAME} control/rnea.cpp)
target_link_libraries(rnea_${PROJECT_NAME} pinocchio::pinocchio moteus::cpp pthread)

add_executable(torqueError_${PROJECT_NAME} control/torqueError.cpp)
target_link_libraries(torqueError_${PROJECT_NAME} pinocchio::pinocchio moteus::cpp pthread)

# Driver Executables
add_executable(manualTorque_${PROJECT_NAME} driver/manualTorque.cpp)
target_link_libraries(manualTorque_${PROJECT_NAME} moteus::cpp pthread) 

add_executable(driver_${PROJECT_NAME} driver/driver.cpp)
target_link_libraries(driver_${PROJECT_NAME} moteus::cpp pthread) 

add_executable(cactusTest_${PROJECT_NAME} driver/cactusTest.cpp)
target_link_libraries(cactusTest_${PROJECT_NAME} PRIVATE moteus::cpp pthread cactus_rt) 

setup_cactus_rt_target_options(cactusTest_${PROJECT_NAME})

add_executable(rtDriver_${PROJECT_NAME} driver/rtDriver.cpp)
target_link_libraries(rtDriver_${PROJECT_NAME} PRIVATE moteus::cpp pthread cactus_rt) 

setup_cactus_rt_target_options(rtDriver_${PROJECT_NAME})

add_executable(bandwidthTest_${PROJECT_NAME} driver/bandwidthTest.cpp)
target_link_libraries(bandwidthTest_${PROJECT_NAME} moteus::cpp pthread)

add_executable(bandwidthTestRT_${PROJECT_NAME} driver/bandwidthTestRT.cpp)
target_link_libraries(bandwidthTestRT_${PROJECT_NAME} moteus::cpp pthread cactus_rt)

setup_cactus_rt_target_options(bandwidthTestRT_${PROJECT_NAME})