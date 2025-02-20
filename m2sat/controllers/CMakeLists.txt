cmake_minimum_required(VERSION 3.10)
project(controllers)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add include directories
include_directories(include)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Add include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Create the library
add_library(controllers STATIC src/controller.cpp src/kalman.cpp)

# Specify include directories for users of this library
target_include_directories(controllers PUBLIC include)
# Link Eigen to the library
target_link_libraries(controllers PUBLIC Eigen3::Eigen)