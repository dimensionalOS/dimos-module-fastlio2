cmake_minimum_required(VERSION 3.5)
project(point_lio_nonros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -pthread -fexceptions")

# Point-LIO expects a ROOT_DIR macro (Log/ PCD/ paths) and the MP defines.
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
add_definitions(-DMP_EN -DMP_PROC_NUM=2)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
  include/IKFoM/IKFoM_toolkit
  src
)

add_executable(pointlio_smoke
  src/main.cpp
  src/Estimator.cpp
  src/parameters.cpp
  src/preprocess.cpp
  include/ikd-Tree/ikd_Tree.cpp
)
target_link_libraries(pointlio_smoke ${PCL_LIBRARIES} yaml-cpp)
if(OpenMP_CXX_FOUND)
  target_link_libraries(pointlio_smoke OpenMP::OpenMP_CXX)
endif()
