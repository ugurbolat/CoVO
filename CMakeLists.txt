cmake_minimum_required(VERSION 3.5)
project(covo)

# Creating compile_command.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable debugable executable binary
set(CMAKE_BUILD_TYPE Release)

# Seperating executables and libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Setting C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Adding modules
add_subdirectory(utils)
add_subdirectory(libcovo)
add_subdirectory(tests)
