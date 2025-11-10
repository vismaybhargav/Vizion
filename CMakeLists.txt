cmake_minimum_required(VERSION 4.0)
project(Vizion)

set(CMAKE_CXX_STANDARD 23)

configure_file(${CMAKE_SOURCE_DIR}/singlemarkersoriginal.jpg ${CMAKE_BINARY_DIR}/singlemarkersoriginal.jpg COPYONLY)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Vizion src/main.cpp)
message(${OpenCV_LIBS})
target_link_libraries(Vizion ${OpenCV_LIBS})