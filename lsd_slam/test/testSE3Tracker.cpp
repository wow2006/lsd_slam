#include <gtest/gtest.h>
#include "tracking/se3_tracker.h"


TEST(test, test) {
  const int width  = 512;
  const int height = 512;
  const auto K = Eigen::Matrix3f::Identity();

  lsd_slam::SE3Tracker tracker(width, height, K);

}

