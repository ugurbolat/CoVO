list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/share/OpenCV)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Gflags REQUIRED)

add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

include_directories(
  ${GFLAGS_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/third_party
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIR})


add_executable(covo_bin covo.cpp)
target_link_libraries(covo_bin 
  ${OpenCV_LIBRARIES} 
  covo
  utils
  ceres)