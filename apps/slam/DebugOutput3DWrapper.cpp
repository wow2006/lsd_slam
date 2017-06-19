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

#include "DebugOutput3DWrapper.h"
#include "util/sophus_util.h"
#include "util/settings.h"

#include "model/frame.h"
#include "global_mapping/key_frame_graph.h"
#include "sophus/sim3.hpp"
#include "global_mapping/g2o_type_sim3_sophus.h"
#include <opencv/highgui.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace lsd_slam {

DebugOutput3DWrapper::DebugOutput3DWrapper(int width, int height) {
    cv::namedWindow("Tracking_output", cv::WINDOW_NORMAL);
    this->width = width;
    this->height = height;

    tracker_display = cv::Mat::zeros(640, 480, CV_8UC1);
    tracker_display.setTo(cv::Scalar(255));
    cv::imshow("Tracking_output", tracker_display);
    publishLvl = 0;
}

DebugOutput3DWrapper::~DebugOutput3DWrapper() {}

void DebugOutput3DWrapper::publishKeyframe(Frame* f) {
    KeyFrameMessage fMsg;

    boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();

    fMsg.id = f->id();
    fMsg.time = f->timestamp();
    fMsg.isKeyframe = true;

    int w = f->width(publishLvl);
    int h = f->height(publishLvl);

    memcpy(fMsg.camToWorld.data(),
           f->getScaledCamToWorld().cast<float>().data(), sizeof(float) * 7);
    fMsg.fx = f->fx(publishLvl);
    fMsg.fy = f->fy(publishLvl);
    fMsg.cx = f->cx(publishLvl);
    fMsg.cy = f->cy(publishLvl);
    fMsg.width = w;
    fMsg.height = h;

    std::cout << "PublishKeyframe" << std::endl;
}

void DebugOutput3DWrapper::publishTrackedFrame(Frame* kf) {
    KeyFrameMessage fMsg;

    fMsg.id = kf->id();
    fMsg.time = kf->timestamp();
    fMsg.isKeyframe = false;

    memcpy(fMsg.camToWorld.data(),
           kf->getScaledCamToWorld().cast<float>().data(), sizeof(float) * 7);
    fMsg.fx = kf->fx(publishLvl);
    fMsg.fy = kf->fy(publishLvl);
    fMsg.cx = kf->cx(publishLvl);
    fMsg.cy = kf->cy(publishLvl);
    fMsg.width = kf->width(publishLvl);
    fMsg.height = kf->height(publishLvl);

    SE3 camToWorld = se3FromSim3(kf->getScaledCamToWorld());

    /*geometry_msgs::PoseStamped pMsg;

    pMsg.pose.position.x = camToWorld.translation()[0];
    pMsg.pose.position.y = camToWorld.translation()[1];
    pMsg.pose.position.z = camToWorld.translation()[2];
    pMsg.pose.orientation.x = camToWorld.so3().unit_quaternion().x();
    pMsg.pose.orientation.y = camToWorld.so3().unit_quaternion().y();
    pMsg.pose.orientation.z = camToWorld.so3().unit_quaternion().z();
    pMsg.pose.orientation.w = camToWorld.so3().unit_quaternion().w();

    if (pMsg.pose.orientation.w < 0)
    {
            pMsg.pose.orientation.x *= -1;
            pMsg.pose.orientation.y *= -1;
            pMsg.pose.orientation.z *= -1;
            pMsg.pose.orientation.w *= -1;
    }

    pMsg.header.stamp = ros::Time(kf->timestamp());
    pMsg.header.frame_id = "world";
    pose_publisher.publish(pMsg);*/

    cv::circle(tracker_display,
               cv::Point(320 + camToWorld.translation()[0] * 640,
                         -240 + camToWorld.translation()[1] * 480),
               2, cv::Scalar(255, 0, 0), 4);
    cv::imshow("Tracking_output", tracker_display);
    std::cout << "PublishTrackedKeyframe: " << camToWorld.translation()[0]
              << " " << camToWorld.translation()[1] << "  "
              << camToWorld.translation()[2] << std::endl;
}

void DebugOutput3DWrapper::publishKeyframeGraph(KeyFrameGraph* graph) {
    /*lsd_slam_viewer::keyframeGraphMsg gMsg;

    graph->edgesListsMutex.lock();
    gMsg.numConstraints = graph->edgesAll.size();
    gMsg.constraintsData.resize(gMsg.numConstraints * sizeof(GraphConstraint));
    GraphConstraint* constraintData =
    (GraphConstraint*)gMsg.constraintsData.data();
    for(unsigned int i=0;i<graph->edgesAll.size();i++)
    {
            constraintData[i].from = graph->edgesAll[i]->firstFrame->id();
            constraintData[i].to = graph->edgesAll[i]->secondFrame->id();
            Sophus::Vector7d err = graph->edgesAll[i]->edge->error();
            constraintData[i].err = sqrt(err.dot(err));
    }
    graph->edgesListsMutex.unlock();

    graph->keyframesAllMutex.lock_shared();
    gMsg.numFrames = graph->keyframesAll.size();
    gMsg.frameData.resize(gMsg.numFrames * sizeof(GraphFramePose));
    GraphFramePose* framePoseData = (GraphFramePose*)gMsg.frameData.data();
    for(unsigned int i=0;i<graph->keyframesAll.size();i++)
    {
            framePoseData[i].id = graph->keyframesAll[i]->id();
            memcpy(framePoseData[i].camToWorld,
    graph->keyframesAll[i]->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
    }
    graph->keyframesAllMutex.unlock_shared();

    graph_publisher.publish(gMsg);*/
}

void DebugOutput3DWrapper::publishTrajectory(
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
        trajectory,
    std::string identifier) {
    // unimplemented ... do i need it?
}

void DebugOutput3DWrapper::publishTrajectoryIncrement(
    const Eigen::Matrix<float, 3, 1>& pt, std::string identifier) {
    // unimplemented ... do i need it?
}

void DebugOutput3DWrapper::publishDebugInfo(
    const Eigen::Matrix<float, 20, 1>& data) {
    // std_msgs::Float32MultiArray msg;
    for (int i = 0; i < 20; i++) std::cout << (float)(data[i]) << std::endl;

    // debugInfo_publisher.publish(msg);
}

}
