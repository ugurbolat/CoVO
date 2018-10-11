set(libcovo_SRCS 
  Feature.cpp Feature.h 
  Matcher.cpp Matcher.h
  Uncertainty.cpp Uncertainty.h
  Optimizer.cpp Optimizer.h
  KeyframeTracker.cpp KeyframeTracker.h
  CeresEuclideanModel.h)

#include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
#conan_basic_setup()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/share/OpenCV)

find_package(OpenCV REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

# gflags
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Gflags REQUIRED)
include_directories( ${GFLAGS_INCLUDE_DIRS} )
add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

include_directories(${CONAN_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/third_party/args
  ${OpenCV_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIR})

# creating utils as a public static library
add_library(covo STATIC ${libcovo_SRCS})

# linking third party libraries
target_link_libraries(covo 
  ${OpenCV_LIBRARIES}
  nlohmann_json::nlohmann_json)

# declaring a path for utils so that executables know where to find
target_include_directories(covo 
  PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR} 
  ceres)
