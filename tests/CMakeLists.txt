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


add_executable(test_read_args test_read_args.cpp)
target_link_libraries(test_read_args 
  ${OpenCV_LIBRARIES}
  utils)

add_executable(test_feature_extraction test_feature_extraction.cpp)
target_link_libraries(test_feature_extraction 
  ${OpenCV_LIBRARIES} 
  covo
  utils)

add_executable(test_feature_matching test_feature_matching.cpp)
target_link_libraries(test_feature_matching 
  ${OpenCV_LIBRARIES} 
  covo
  utils)

add_executable(test_covariance test_covariance.cpp)
target_link_libraries(test_covariance
  ${OpenCV_LIBRARIES} 
  covo
  utils)

add_executable(test_optimization test_optimization.cpp)
target_link_libraries(test_optimization
  ${OpenCV_LIBRARIES} 
  covo
  utils
  ceres)

