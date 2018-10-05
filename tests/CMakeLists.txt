#include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
#conan_basic_setup()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/share/OpenCV)

find_package(OpenCV REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${CONAN_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/third_party/args
  ${OpenCV_INCLUDE_DIRS})


add_executable(test_read_args test_read_args.cpp)
# linking internal utils library
target_link_libraries(test_read_args 
  ${CONAN_LIBS} 
  ${OpenCV_LIBRARIES}
  nlohmann_json::nlohmann_json 
  utils)

add_executable(test_feature_extraction test_feature_extraction.cpp)
# linking internal utils library
target_link_libraries(test_feature_extraction 
  ${CONAN_LIBS} 
  ${OpenCV_LIBRARIES} 
  nlohmann_json::nlohmann_json
  covo
  utils)

add_executable(test_feature_matching test_feature_matching.cpp)
# linking internal utils library
target_link_libraries(test_feature_matching 
  ${CONAN_LIBS} 
  ${OpenCV_LIBRARIES} 
  nlohmann_json::nlohmann_json
  covo
  utils)

