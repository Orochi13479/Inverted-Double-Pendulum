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

# find_package(pinocchio)

add_executable(${PROJECT_NAME}_main main.cpp IDPControl.cpp IDPDriver.cpp)
target_link_libraries(${PROJECT_NAME}_main moteus::cpp)

# add_executable(IDPControl IDPControl.cpp)
# target_link_libraries(IDPControl moteus::cpp)
# target_link_libraries(IDPControl pinocchio::pinocchio)

# add_executable(IDPDriver IDPDriver.cpp)
# target_link_libraries(IDPDriver moteus::cpp)
# target_link_libraries(IDPDriver pinocchio::pinocchio)


