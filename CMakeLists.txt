cmake_minimum_required(VERSION 3.25)

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

add_executable(IDPControl IDPControl.cc)
target_link_libraries(IDPControl moteus::cpp)
target_link_libraries(IDPControl pinocchio::pinocchio)

add_executable(IDPDriver IDPDriver.cc)
target_link_libraries(IDPDriver moteus::cpp)
target_link_libraries(IDPDriver pinocchio::pinocchio)
