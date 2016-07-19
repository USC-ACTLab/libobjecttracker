cmake_minimum_required(VERSION 2.8.3)
project(libobjecttracker)

# Enable C++11
# This requires PCL to be compiled with C++11 enabled as well!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL REQUIRED)
# find_package(Eigen3 REQUIRED)

# Additional include folders
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  /usr/include/eigen3/
  # ${EIGEN3_INCLUDE_DIR}
)

# Declare a cpp library
add_library(libobjecttracker
  src/object_tracker.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(libobjecttracker
  ${PCL_LIBRARIES}
)
