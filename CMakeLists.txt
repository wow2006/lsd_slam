cmake_minimum_required(VERSION 2.8.9)
set(LsdSlam_PROJECT_NAME lsd_slam_and_apps)
project(${LsdSlam_PROJECT_NAME})

# Look for our CMake modules paths
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

find_package(CSparse REQUIRED)
find_package(G2O     REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(Sophus  REQUIRED)
find_package(Threads REQUIRED)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost   REQUIRED
    COMPONENTS date_time filesystem system thread program_options)

# Add lsd_slam library
add_subdirectory(lsd_slam)
add_subdirectory(apps/slam)
