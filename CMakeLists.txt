cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#find_package(OpenCV 2.3.1 REQUIRED)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(analyzehands src/analyze_hands.cpp)
target_link_libraries(analyzehands pcl_utils)

rosbuild_add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration pcl_utils)

rosbuild_add_executable(pointing src/pointing_area.cpp)
target_link_libraries(pointing pcl_utils)

rosbuild_add_executable(visual src/visual.cpp)
target_link_libraries(visual pcl_utils)
rosbuild_add_executable(training src/training.cpp)
target_link_libraries(training pcl_utils)

rosbuild_add_executable(detectskelhands src/detect_hands_wskel.cpp)
target_link_libraries(detectskelhands pcl_utils)

rosbuild_add_executable(detecthands src/detect_hands.cpp)
target_link_libraries(detecthands pcl_utils)

rosbuild_add_executable(saver src/saveHandCloud.cpp)
target_link_libraries(saver pcl_utils)

#rosbuild_add_executable(predict src/predict.cpp src/svm.cpp)
#target_link_libraries(predict pcl_utils)

rosbuild_add_executable(predict src/predict2.cpp src/svm.cpp)
target_link_libraries(predict pcl_utils)

rosbuild_add_executable(yubisashi src/yubisashi.cpp src/svm.cpp)
target_link_libraries(yubisashi pcl_utils)




