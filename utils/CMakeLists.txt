set(libutils_SRCS 
  ReadArgs.cpp ReadArgs.h)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/share/OpenCV)

find_package(OpenCV REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/third_party
  ${OpenCV_INCLUDE_DIRS})

# creating utils as a public static library
add_library(utils STATIC ${libutils_SRCS})

# linking third party libraries
target_link_libraries(utils 
  ${OpenCV_LIBRARIES}) 

# declaring a path for utils so that executables know where to find
target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
