set(
  lsd_slam_src
  live_slam_wrapper.cc
  slam_system.cpp
  # adding depth_estimation dir
  depth_estimation/depth_map.cc
  depth_estimation/depth_map_pixel_hypothesis.cc
  # adding global_mapping dir
  global_mapping/fab_map.cc
  global_mapping/g2o_type_sim3_sophus.cc
  global_mapping/key_frame_graph.cc
  global_mapping/trackable_key_frame_search.cc
  # adding io_wrapper dir
  io_wrapper/ImageDisplay_OpenCV.cc
  io_wrapper/OpenCVImageStreamThread.cpp
  io_wrapper/timestamp.cc
  # adding model dir
  model/frame.cc
  model/frame_memory.cc
  model/frame_pose_struct.cc
  # adding tracking dir
  tracking/least_squares.cc
  tracking/relocalizer.cc
  tracking/se3_tracker.cc
  tracking/sim3_tracker.cc
  tracking/tracking_reference.cpp
  # adding util dir
  util/GlobalFuncs.cpp
  util/settings.cc
  util/sophus_util.cc
  util/undistorter.cc
)

add_library(
  lsd_slam
  ${lsd_slam_src}
)

target_include_directories(
  lsd_slam
  PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIR}
)

target_link_libraries(
  lsd_slam
  PUBLIC
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_DENSE}
  ${G2O_SOLVER_EIGEN}
  ${G2O_TYPES_DATA}
  ${G2O_TYPES_ICP}
  ${G2O_STUFF_LIBRARY}
  ${G2O_CORE_LIBRARY}

  ${OpenCV_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
)

set_target_properties(
  lsd_slam
  PROPERTIES
  CXX_STANDARD 14
)

target_compile_options(
  lsd_slam
  PUBLIC
  -Wno-c++98-compat
)

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()
