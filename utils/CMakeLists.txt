set(libutils_SRCS 
  ReadArgs.cpp ReadArgs.h)

#include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
#conan_basic_setup()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/share/OpenCV)

find_package(OpenCV REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${CONAN_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/third_party/args
  ${OpenCV_INCLUDE_DIRS})

# creating utils as a public static library
add_library(utils STATIC ${libutils_SRCS})

# linking third party libraries
target_link_libraries(utils ${CONAN_LIBS} ${OpenCV_LIBRARIES}
  nlohmann_json::nlohmann_json) 

# declaring a path for utils so that executables know where to find
target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
