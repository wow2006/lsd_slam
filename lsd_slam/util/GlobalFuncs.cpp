/**
 * This file is part of LSD-SLAM.
 *
 * Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical
 * University of Munich) For more information see
 * <http://vision.in.tum.de/lsdslam>
 *
 * LSD-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LSD-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#include "opencv2/opencv.hpp"

#include "util/GlobalFuncs.hpp"
#include "util/sophus_util.h"

#include "model/frame.h"

#include <glog/logging.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

namespace lsd_slam {

SE3 SE3CV2Sophus(const cv::Mat &R, const cv::Mat &t) {
  Sophus::Matrix3f sR;
  Sophus::Vector3f st;

  for (int i = 0; i < 3; i++) {
    sR(0, i) = R.at<double>(0, i);
    sR(1, i) = R.at<double>(1, i);
    sR(2, i) = R.at<double>(2, i);
    st[i] = t.at<double>(i);
  }

  return SE3(toSophus(sR.inverse()), toSophus(st));
}

void printMessageOnCVImage(cv::Mat &image, std::string line1,
                           std::string line2) {
  for (int x = 0; x < image.cols; x++)
    for (int y = image.rows - 30; y < image.rows; y++)
      image.at<cv::Vec3b>(y, x) *= 0.5;

  cv::putText(image, line2, cv::Point(10, image.rows - 5),
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 250), 1, 8);

  cv::putText(image, line1, cv::Point(10, image.rows - 18),
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 250), 1, 8);
}

cv::Mat getDepthRainbowPlot(Frame *kf, int lvl) {
  return getDepthRainbowPlot(kf->idepth(lvl), kf->idepthVar(lvl),
                             kf->image(lvl), kf->width(lvl), kf->height(lvl));
}

cv::Mat getDepthRainbowPlot(const float *idepth, const float *idepthVar,
                            const float *gray, int width, int height) {
  cv::Mat res = cv::Mat(height, width, CV_8UC3);
  if (gray != 0) {
    cv::Mat keyFrameImage(height, width, CV_32F, const_cast<float *>(gray));
    cv::Mat keyFrameImage8u;
    keyFrameImage.convertTo(keyFrameImage8u, CV_8UC1);
    cv::cvtColor(keyFrameImage8u, res, cv::COLOR_GRAY2RGB);
  } else
    fillCvMat(&res, cv::Vec3b(255, 170, 168));

  for (int i = 0; i < width; i++)
    for (int j = 0; j < height; j++) {
      float id = idepth[i + j * width];

      if (id >= 0 && idepthVar[i + j * width] >= 0) {

        // rainbow between 0 and 4
        float r = (0 - id) * 255 / 1.0;
        if (r < 0)
          r = -r;
        float g = (1 - id) * 255 / 1.0;
        if (g < 0)
          g = -g;
        float b = (2 - id) * 255 / 1.0;
        if (b < 0)
          b = -b;

        uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
        uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
        uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);

        res.at<cv::Vec3b>(j, i) = cv::Vec3b(255 - rc, 255 - gc, 255 - bc);
      }
    }
  return res;
}
cv::Mat getVarRedGreenPlot(const float *idepthVar, const float *gray, int width,
                           int height) {
  float *idepthVarExt = new float[width * height];
  memcpy(idepthVarExt, idepthVar, sizeof(float) * width * height);

  for (int i = 2; i < width - 2; i++)
    for (int j = 2; j < height - 2; j++) {
      if (idepthVar[(i) + width * (j)] <= 0)
        idepthVarExt[(i) + width * (j)] = -1;
      else {
        float sumIvar = 0;
        float numIvar = 0;
        for (int dx = -2; dx <= 2; dx++)
          for (int dy = -2; dy <= 2; dy++) {
            if (idepthVar[(i + dx) + width * (j + dy)] > 0) {
              float distFac =
                  (float)(dx * dx + dy * dy) * (0.075 * 0.075) * 0.02;
              float ivar =
                  1.0f / (idepthVar[(i + dx) + width * (j + dy)] + distFac);
              sumIvar += ivar;
              numIvar += 1;
            }
          }
        idepthVarExt[(i) + width * (j)] = numIvar / sumIvar;
      }
    }

  cv::Mat res = cv::Mat(height, width, CV_8UC3);
  if (gray != 0) {
    cv::Mat keyFrameImage(height, width, CV_32F, const_cast<float *>(gray));
    cv::Mat keyFrameImage8u;
    keyFrameImage.convertTo(keyFrameImage8u, CV_8UC1);
    cv::cvtColor(keyFrameImage8u, res, cv::COLOR_GRAY2RGB);
  } else
    fillCvMat(&res, cv::Vec3b(255, 170, 168));

  for (int i = 0; i < width; i++)
    for (int j = 0; j < height; j++) {
      float idv = idepthVarExt[i + j * width];

      if (idv > 0) {
        float var = sqrt(idv);

        var = var * 60 * 255 * 0.5 - 20;
        if (var > 255)
          var = 255;
        if (var < 0)
          var = 0;

        res.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 255 - var, var);
      }
    }

  delete[] idepthVarExt;
  return res;
}

std::vector<std::string> getFiles(const std::string &_dir) {
  namespace fs = boost::filesystem;

  auto output = std::vector<std::string>();
  auto imagesPath = fs::path(_dir);
  try {
    if (fs::exists(imagesPath) && fs::is_directory(imagesPath)) {
      LOG(INFO) << _dir << " directory exist!\n";
      std::vector<fs::path> images;
      std::copy(fs::directory_iterator(imagesPath), fs::directory_iterator(),
                std::back_inserter(images));
      std::sort(std::begin(images), std::end(images));
      /*
                [](fs::path& it1, fs::path& it2) {
                return std::stoi(it1.stem().string()) <
         std::stoi(it2.stem().string());
                });
                */
      output.resize(images.size());
      std::transform(std::begin(images), std::end(images), std::begin(output),
                     [](const fs::path &_path) { return _path.string(); });
    }
  } catch (const fs::filesystem_error &exp) {
    LOG(ERROR) << exp.what() << '\n';
  }
  return output;
}

std::vector<std::string> parseArgs(int argc, char **argv) {
  namespace po = boost::program_options;

  std::string inputFile, outputFile, intrinsicFile;

  po::options_description desc("LSD_SLAM input");
  desc.add_options()("help", "")(
      "input,i", po::value<std::string>(&inputFile)->required(), "")(
      "intrinsic,k", po::value<std::string>(&intrinsicFile)->required(), "");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cerr << desc << '\n';
      std::exit(0);
    }
  } catch (...) {
    std::cerr << desc << '\n';
    std::exit(-1);
  }

  return std::vector<std::string>{inputFile, outputFile, intrinsicFile};
}

Eigen::Matrix3f getIntrinsicMatrix(const std::string &_dir) {
  cv::FileStorage inputFile(_dir, cv::FileStorage::READ |
                                      cv::FileStorage::FORMAT_JSON);
  cv::Mat opencvK;
  inputFile["IntrinsicMatrix"] >> opencvK;

  Eigen::Matrix3f K;
  K << opencvK.at<float>(0, 0), opencvK.at<float>(0, 1),
      opencvK.at<float>(0, 2), opencvK.at<float>(1, 0), opencvK.at<float>(1, 1),
      opencvK.at<float>(1, 2), opencvK.at<float>(2, 0), opencvK.at<float>(2, 1),
      opencvK.at<float>(2, 2);

  return K;
}

} // namespace lsd_slam
