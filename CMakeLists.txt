cmake_minimum_required(VERSION 2.8)
PROJECT(K4a_grabber)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}" ${CMAKE_MODULE_PATH})

add_executable(viewer
  Source_pcl.cpp k4a_grabber.cpp k4a_grabber.h)

find_package(k4a REQUIRED)
find_package(PCL 1.10 REQUIRED)

add_definitions(-DNDEBUG)
add_definitions(-D_USE_MATH_DEFINES)
add_definitions(-DNOMINMAX)
add_definitions(-DPCL_NO_PRECOMPILE)
add_definitions(-DEIGEN_MAKE_ALIGNED_OPERATION_NEW)
add_definitions(-DVTK_VISUALIZATION)

target_link_libraries(viewer ${PCL_LIBRARIES} k4a::k4a)
