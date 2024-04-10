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

add_executable(${PROJECT_NAME}_manualTorque driver/manualTorque.cpp)
target_link_libraries(${PROJECT_NAME}_manualTorque moteus::cpp pthread) 


add_executable(${PROJECT_NAME}_main main.cpp IDPDriver.cpp IDPControl.cpp)
# target_link_libraries(${PROJECT_NAME}_main moteus::cpp pthread)

# Uncomment if Pinocchio is needed
find_package(pinocchio)
target_link_libraries(${PROJECT_NAME}_main pinocchio::pinocchio moteus::cpp pthread) 
