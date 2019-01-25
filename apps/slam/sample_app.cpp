#include <string>
#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <eigen3/Eigen/Geometry>
#include <sophus/geometry.hpp>

#include "slam_system.hpp"
#include "live_slam_wrapper.h"
#include "DebugOutput3DWrapper.h"
#include "util/settings.h"
#include "util/undistorter.h"
#include "util/global_funcs.h"
#include "io_wrapper/OpenCVImageStreamThread.h"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using namespace lsd_slam;

namespace fs = boost::filesystem;

auto getFiles(const std::string& _dir) {
    auto output = std::vector<std::string>();
    auto imagesPath = fs::path(_dir);
    try {
        if(fs::exists(imagesPath) &&  fs::is_directory(imagesPath)) {
            std::cout << _dir << " directory exist!\n";
            std::vector<fs::path> images;
            std::copy(fs::directory_iterator(imagesPath),
                      fs::directory_iterator(),
                      std::back_inserter(images));
            std::sort(std::begin(images), std::end(images));
            /*
                      [](fs::path& it1, fs::path& it2) {
                      return std::stoi(it1.stem().string()) < std::stoi(it2.stem().string());
                      });
                      */
            output.resize(images.size());
            std::transform(std::begin(images), std::end(images), std::begin(output),
                    [](const fs::path& _path) { return _path.string(); });
        }
    }
    catch(const fs::filesystem_error& exp) {
        std::cerr << exp.what() << '\n';
    }
  return output;
}

auto getIntrinsicMatrix(const std::string& _dir) {
  cv::FileStorage inputFile(_dir, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  cv::Mat opencvK;
  inputFile["IntrinsicMatrix"] >> opencvK;

  Eigen::Matrix3f K;
  K << opencvK.at<float>(0, 0), opencvK.at<float>(0, 1), opencvK.at<float>(0, 2),
       opencvK.at<float>(1, 0), opencvK.at<float>(1, 1), opencvK.at<float>(1, 2),
       opencvK.at<float>(2, 0), opencvK.at<float>(2, 1), opencvK.at<float>(2, 2);

  return K;
}

namespace po = boost::program_options;

auto parseArgs(int argc, char** argv) {
  std::string inputFile, outputFile, intrinsicFile;

  po::options_description desc("LSD_SLAM input");
  desc.add_options()
      ("help", "")
      ("input,i", po::value<std::string>(&inputFile)->required(), "")
      ("intrinsic,k", po::value<std::string>(&intrinsicFile)->required(), "");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if(vm.count("help")) {
        std::cerr << desc << '\n';
        std::exit(0);
    }
  } catch (...) {
    std::cerr << desc << '\n';
    std::exit(-1);
  }

  return std::vector<std::string> {inputFile, outputFile, intrinsicFile};
}

int main(int argc, char **argv) {
  const auto inputFiles = parseArgs(argc, argv);

  // Get images in dir
  const auto files = getFiles(inputFiles[0]);

  const auto undistorter = Undistorter::getUndistorterForFile(inputFiles[2].c_str());
  const float fx = undistorter->getK().at<double>(0, 0);
  const float fy = undistorter->getK().at<double>(1, 1);
  const float cx = undistorter->getK().at<double>(2, 0);
  const float cy = undistorter->getK().at<double>(2, 1);

  const int out_width  = undistorter->getOutputWidth();
  const int out_height = undistorter->getOutputHeight();
  Sophus::Matrix3f K;
  K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
  {
    const auto firstImage = cv::imread(files.front(), cv::IMREAD_GRAYSCALE);
    std::cout << "Start frame : " << files.front() << '\n';
    // Read First Image
    if(firstImage.empty()) {
        std::cerr << "Can not found " << firstImage << '\n';
        return -1;
    }

    const int width = firstImage.cols, height = firstImage.rows;
    auto system = std::make_unique<SlamSystem>(out_width, out_height, K, true);
    // make output wrapper. just set to zero if no output is required.
    int index = 0;
    double timeStamp = 0.0f;

    cv::Mat image = cv::Mat(height, width, CV_8U);
    undistorter->undistort(firstImage, image);
    system->randomInit(image.data, timeStamp, index);

    for(auto file : files) {
      auto currentImage = cv::imread(file, cv::IMREAD_GRAYSCALE);

      if(currentImage.empty()) {
          continue;
      }

      undistorter->undistort(currentImage, image);
      system->trackFrame(image.data, static_cast<uint>(index++),
                         false, timeStamp += 0.03);
    }
  }

  return 0;
}
