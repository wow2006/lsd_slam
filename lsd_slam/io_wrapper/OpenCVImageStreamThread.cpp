/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University
* of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
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

#include "OpenCVImageStreamThread.h"
//#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "cv_bridge/cv_bridge.h"
#include "util/settings.h"

#include <ctime>
#include <fstream>
#include <iostream>

namespace lsd_slam {

using namespace cv;

OpenCVImageStreamThread::OpenCVImageStreamThread() {
  // wait for cam calib
  width_ = height_ = 0;

  // imagebuffer
  imageBuffer = new NotifyBuffer<TimestampedMat>(8);
  undistorter = 0;
  lastSEQ = 0;

  haveCalib = false;
  undistorter = 0;
}

OpenCVImageStreamThread::~OpenCVImageStreamThread() { delete imageBuffer; }

void OpenCVImageStreamThread::setCameraCapture(CvCapture *cap) {
  capture = cap;
}

void OpenCVImageStreamThread::setCalibration(std::string file) {
  if (file == "") {
    printf("NO camera calibration file!\n");
  } else {
    undistorter = Undistorter::getUndistorterForFile(file.c_str());

    if (!undistorter) {
      printf("Failed to read camera calibration from file... wrong syntax?\n");
      assert("Failed to read camera calibration from file... wrong syntax?");
    }

    fx_ = undistorter->getK().at<double>(0, 0);
    fy_ = undistorter->getK().at<double>(1, 1);
    cx_ = undistorter->getK().at<double>(2, 0);
    cy_ = undistorter->getK().at<double>(2, 1);

    width_ = undistorter->getOutputWidth();
    height_ = undistorter->getOutputHeight();
  }

  haveCalib = true;
}

void OpenCVImageStreamThread::run() { boost::thread thread(boost::ref(*this)); }

void OpenCVImageStreamThread::operator()() {
  // ros::spin();
  if (!haveCalib) {
    assert("no calibration");
    return;
  }
  if (!capture) {
    assert("NO valid camera capture pointer");
    return;
  }
  while (1) {
    TimestampedMat bufferItem;
    bufferItem.timestamp = Timestamp::now();
    auto frame = cv::cvarrToMat(cvQueryFrame(capture));
    if (undistorter != 0) {
      assert(undistorter->isValid());
      undistorter->undistort(frame, bufferItem.data);
    } else {
      bufferItem.data = frame;
    }
    // bufferItem.data = cv::Mat(frame, true);
    imageBuffer->pushBack(bufferItem);
  }
  exit(0);
}

} //! lsd_slam
