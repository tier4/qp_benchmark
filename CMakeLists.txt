cmake_minimum_required(VERSION 3.8)
project(trajectory_optimizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(thirdparty/matplotlibcpp17)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(benchmark_smoother
  src/smoother.cpp
  src/trajectory_utils.cpp
  src/resample.cpp
)

ament_auto_add_executable(benchmark
  src/benchmark.cpp
)
target_link_libraries(benchmark benchmark_smoother matplotlibcpp17::matplotlibcpp17)

ament_auto_package(
  INSTALL_TO_SHARE
  bags
  config
)
