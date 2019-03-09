cmake_minimum_required(VERSION 3.9)
set(LsdSlam_PROJECT_NAME lsd_slam_and_apps)
project(${LsdSlam_PROJECT_NAME} CXX)

# Look for our CMake modules paths
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

option(ENABLE_TESTING "Enable Testing" ON)
if(ENABLE_TESTING)
  enable_testing()

  find_package(GTest REQUIRED)
endif()

find_package(CSparse REQUIRED)
find_package(G2O     REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(Sophus  REQUIRED)
find_package(Glog    REQUIRED)

find_package(QGLViewer REQUIRED)

find_package(
  Boost
  REQUIRED
  COMPONENTS
  filesystem
  thread
  program_options
)

# Add lsd_slam library
add_subdirectory(lsd_slam)
add_subdirectory(apps/slam)


