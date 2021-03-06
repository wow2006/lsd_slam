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
#pragma once

#include "io_wrapper/input_image_stream.h"
#include "io_wrapper/notify_buffer.h"
#include "io_wrapper/timestamped_object.h"

#include "util/undistorter.h"

#include <opencv2/opencv.hpp>

namespace lsd_slam {

/**
 * Image stream provider.
 */
class OpenCVImageStreamThread : public InputImageStream {
public:
  OpenCVImageStreamThread();

  ~OpenCVImageStreamThread();

  /**
   * Starts the thread.
   */
  void run();

  void setCalibration(std::string file);

  void setCameraCapture(CvCapture *cap);

  /**
   * Thread main function.
   */
  void operator()();

private:
  bool haveCalib;
  std::unique_ptr<Undistorter> undistorter;

  int lastSEQ;

  CvCapture *capture;
};
} // namespace lsd_slam
