cmake_minimum_required(VERSION 2.8.3)
project(orb2_slam_cuda_ros)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../orb2_slam_cuda/cmake_modules)

find_package(OpenCV 3.1.0 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED
  roscpp
  cv_bridge
  image_transport
  message_filters
  orb2_slam_cuda
  tf
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS orb2_slam_cuda roscpp cv_bridge image_transport message_filters tf
  DEPENDS EIGEN3 OpenCV Pangolin
)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/
  ${PROJECT_SOURCE_DIR}/include
  ${Pangolin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(LIBS 
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
)

# Node for RGB-D camera
add_executable(orb2_slam_cuda_node
src/ros_node.cc
src/image_grabber.cc
src/ros_mono.cc
src/ros_rgbd.cc
src/slam_data.cc
)

target_link_libraries(orb2_slam_cuda_node
  ${LIBS}
  ${catkin_LIBRARIES}
)

