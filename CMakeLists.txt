project(phobos_perloc)
cmake_minimum_required(VERSION 3.18)

# Find packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add link dirs
include_directories(${OpenCV_INCLUDE_DIRS})

# Set output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# For VSCode include path
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# C++ flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")

# Add the src dir
add_subdirectory(src)
