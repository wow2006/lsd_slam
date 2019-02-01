add_executable(
  testSE3Tracker 
  testSE3Tracker.cpp
)

target_link_libraries(
  testSE3Tracker
  PUBLIC
  GTest::GTest
  GTest::Main
  lsd_slam
)

add_executable(
  testDepthTracking
  testDepthTracking.cpp
)

target_link_libraries(
  testDepthTracking 
  PUBLIC
  GTest::GTest
  GTest::Main
  lsd_slam
)

