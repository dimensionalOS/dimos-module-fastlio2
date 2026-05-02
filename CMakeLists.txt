cmake_minimum_required(VERSION 3.5)
project(fast_lio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  include
  src
  src/map_builder
)

add_executable(fastlio_mapping
  src/main.cpp
  src/utils.cpp
  src/map_builder/commons.cpp
  src/map_builder/ieskf.cpp
  src/map_builder/ikd_Tree.cpp
  src/map_builder/imu_processor.cpp
  src/map_builder/lidar_processor.cpp
  src/map_builder/map_builder.cpp
)

find_library(NL3_LIB nl-3 HINTS ENV LIBRARY_PATH)
find_library(NL_GENL3_LIB nl-genl-3 HINTS ENV LIBRARY_PATH)

target_link_libraries(fastlio_mapping
  ${PCL_LIBRARIES}
  yaml-cpp
  Sophus::Sophus
  ${NL3_LIB}
  ${NL_GENL3_LIB}
)
