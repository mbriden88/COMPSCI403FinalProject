PROJECT(final_project)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
find_package(OpenCV REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)
ROSBUILD_INIT()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

ROSBUILD_GENMSG()
ROSBUILD_GENSRV()

SET(libs roslib roscpp)

ROSBUILD_ADD_EXECUTABLE(final_project_main
    src/final_project_main.cpp)
TARGET_LINK_LIBRARIES(final_project_main ${libs} ${OpenCV_LIBS})