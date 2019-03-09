#include <thread>
#include <memory>

#include "ImageReader.hpp"
#include "slam_system.hpp"

#include "util/GlobalFuncs.hpp"
#include "util/undistorter.h"

#include <opencv2/opencv.hpp>

#include <glog/logging.h>

namespace Application {

ImageReader::ImageReader() = default;

ImageReader::~ImageReader() = default;

bool ImageReader::initialize(int argc, char **argv) {
  const auto vInputFiles = lsd_slam::parseArgs(argc, argv);

  // Get images in dir
  m_vFiles = lsd_slam::getFiles(vInputFiles[0]);

  m_pUndistorter =
      lsd_slam::Undistorter::getUndistorterForFile(vInputFiles[2].c_str());
  const float fx = m_pUndistorter->getK().at<double>(0, 0);
  const float fy = m_pUndistorter->getK().at<double>(1, 1);
  const float cx = m_pUndistorter->getK().at<double>(2, 0);
  const float cy = m_pUndistorter->getK().at<double>(2, 1);

  const int out_width = m_pUndistorter->getOutputWidth();
  const int out_height = m_pUndistorter->getOutputHeight();
  Sophus::Matrix3f K;
  K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
  const auto firstImage = cv::imread(m_vFiles.front(), cv::IMREAD_GRAYSCALE);
  LOG(INFO) << "Start frame : " << m_vFiles.front() << '\n';
  // Read First Image
  if (firstImage.empty()) {
    LOG(FATAL) << "Can not found " << firstImage << '\n';
    return false;
  }

  const int width = firstImage.cols, height = firstImage.rows;
  m_pSystem =
      std::make_unique<lsd_slam::SlamSystem>(out_width, out_height, K, true);

  cv::Mat image = cv::Mat(height, width, CV_8U);
  m_pUndistorter->undistort(firstImage, image);
  m_pSystem->randomInit(image.data, 0.0f, 0);

  mRunningThread = std::thread([](ImageReader* pImageReader){
      pImageReader->run();
  }, this);

  std::cout << "Start Thread!\n";

  return true;
}

void ImageReader::run() {
  // make output wrapper. just set to zero if no output is required.
  int index = 0;
  double timeStamp = 0.0f;

  cv::Mat image;

  for (const auto &file : m_vFiles) {
    if(m_bDone.load()) {
      return;
    }

    auto currentImage = cv::imread(file, cv::IMREAD_GRAYSCALE);

    if (currentImage.empty()) {
      continue;
    }

    m_pUndistorter->undistort(currentImage, image);
    m_pSystem->trackFrame(image.data, static_cast<uint>(index++), false,
                        timeStamp += 0.03);
  }
}

void ImageReader::cleanup() {
  m_bDone = true;
  mRunningThread.join();
}

} // namespace Application
