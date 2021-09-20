cmake_minimum_required(VERSION 3.1.0)
project(rs_test VERSION 0.1.0)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 

 
include(CTest)
enable_testing()

set(DEPENDENCIES realsense2)

find_package(realsense2)
if(NOT realsense2_FOUND)
  message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()
find_library(realsense2_LIBRARIES realsense2)
find_path(realsense2_INCLUDE_DIRS librealsense2/rs.hpp)

FIND_PACKAGE( Boost 1.71.0 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_executable(rs_test main.cpp)
target_include_directories(rs_test PUBLIC ./librealsense-master/examples)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set_property(TARGET rs_test PROPERTY CXX_STANDARD 11)
target_link_libraries(rs_test ${DEPENDENCIES} ${Boost_LIBRARIES})

set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)




 

