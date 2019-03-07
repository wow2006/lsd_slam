#include <memory>

#include <opencv2/opencv.hpp>

#include <sophus/geometry.hpp>

#include "slam_system.hpp"

#include "util/undistorter.h"
#include "util/GlobalFuncs.hpp"

#include <glog/logging.h>

int main(int argc, char **argv) {
  google::SetLogDestination(google::GLOG_INFO, ".");
  google::InitGoogleLogging(argv[0]);

  const auto inputFiles = lsd_slam::parseArgs(argc, argv);

  // Get images in dir
  const auto files = lsd_slam::getFiles(inputFiles[0]);

  const auto undistorter =
      lsd_slam::Undistorter::getUndistorterForFile(inputFiles[2].c_str());
  const float fx = undistorter->getK().at<double>(0, 0);
  const float fy = undistorter->getK().at<double>(1, 1);
  const float cx = undistorter->getK().at<double>(2, 0);
  const float cy = undistorter->getK().at<double>(2, 1);

  const int out_width = undistorter->getOutputWidth();
  const int out_height = undistorter->getOutputHeight();
  Sophus::Matrix3f K;
  K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
  {
    const auto firstImage = cv::imread(files.front(), cv::IMREAD_GRAYSCALE);
    LOG(INFO) << "Start frame : " << files.front() << '\n';
    // Read First Image
    if (firstImage.empty()) {
      LOG(FATAL) << "Can not found " << firstImage << '\n';
      return -1;
    }

    const int width = firstImage.cols, height = firstImage.rows;
    auto system = std::make_unique<lsd_slam::SlamSystem>(out_width, out_height, K, true);
    // make output wrapper. just set to zero if no output is required.
    int index = 0;
    double timeStamp = 0.0f;

    cv::Mat image = cv::Mat(height, width, CV_8U);
    undistorter->undistort(firstImage, image);
    system->randomInit(image.data, timeStamp, index);

    for (auto file : files) {
      auto currentImage = cv::imread(file, cv::IMREAD_GRAYSCALE);

      if (currentImage.empty()) {
        continue;
      }

      undistorter->undistort(currentImage, image);
      system->trackFrame(image.data, static_cast<uint>(index++), false,
                         timeStamp += 0.03);
    }
  }

  return 0;
}
