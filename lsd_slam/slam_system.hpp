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
#include "io_wrapper/timestamp.h"
#include "opencv2/core/core.hpp"
#include "util/settings.h"
#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <chrono>
#include <deque>
#include <vector>

#include "tracking/relocalizer.h"
#include "util/sophus_util.h"

namespace lsd_slam {

class TrackingReference;
class KeyFrameGraph;
class SE3Tracker;
class Sim3Tracker;
class DepthMap;
class Frame;
class DataSet;
class LiveSLAMWrapper;
class Output3DWrapper;
class TrackableKeyFrameSearch;
class FramePoseStruct;
struct KFConstraintStruct;

typedef Eigen::Matrix<float, 7, 7> Matrix7x7;

class SlamSystem {
  friend class IntegrationTest;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // settings. Constant from construction onward.
  int width;              /** < Image Width */
  int height;             /** < Image Height */
  Eigen::Matrix3f K;      /** < Camera Intrinsic */
  const bool SLAMEnabled; /** < */

  bool trackingIsGood; /** < */

  SlamSystem(int w, int h, Eigen::Matrix3f K, bool enableSLAM = true);

  SlamSystem(const SlamSystem &) = delete;

  SlamSystem &operator=(const SlamSystem &) = delete;

  ~SlamSystem();

  /**
   * @brief         Initialize SLAM system with image
   *
   * @param image      First frame
   * @param timeStamp  First frame time stamp
   * @param id         First frame ID unique for system
   */
  void randomInit(uchar *image, double timeStamp, int id);

  /**
   * @brief
   *
   * @param image
   * @param depth
   * @param timeStamp
   * @param id
   */
  void gtDepthInit(uchar *image, float *depth, double timeStamp, int id);

  /**
   * @brief                 tracks a frame.
   * first frame will return Identity = camToWord.
   * frameID needs to be monotonically increasing.
   * returns camToWord transformation of the tracked frame.
   *
   * @param image              pointer to new frame.
   * @param frameID            frame ID unique for system.
   * @param blockUntilMapped   Force to block this thread till mapping thread
   * finish.
   * @param timestamp          frame time stamp
   */
  void trackFrame(uchar *image, unsigned int frameID, bool blockUntilMapped,
                  double timestamp);

  /**
   * @brief finalizes the system, i.e. blocks and does all remaining
   * loop-closures etc.
   */
  void finalize();

  /**
   * @brief Does an offline optimization step.
   */
  void optimizeGraph();

  /**
   * @brief  get current frame in process
   * Not thread-safe
   *
   * @Returns pointer to current Frame
   */
  inline Frame *getCurrentKeyframe() { return currentKeyFrame.get(); }

  /**
   * @brief  get current pose estimate.
   * @note thread-safe
   *
   * @Returns   Returns the current pose estimate.
   */
  SE3 getCurrentPoseEstimate();

  /**
   * @brief  Sets the visualization where point clouds and camera poses will be
   * sent to.
   *
   * @param outputWrapper Custom Implementation of Output3DWrapper
   */
  void setVisualization(Output3DWrapper *outputWrapper);

  /**
   * @brief
   *
   * @param filename
   */
  void requestDepthMapScreenshot(const std::string &filename);

  /**
   * @brief
   *
   * @Returns
   */
  bool doMappingIteration();

  /**
   * @brief
   *
   * @param newKeyFrame
   * @param forceParent
   * @param useFABMAP
   * @param closeCandidatesTH
   *
   * @Returns
   */
  int findConstraintsForNewKeyFrames(Frame *newKeyFrame,
                                     bool forceParent = true,
                                     bool useFABMAP = true,
                                     float closeCandidatesTH = 1.0);

  bool optimizationIteration(int itsPerTry, float minChange);

  void publishKeyframeGraph();

  std::vector<FramePoseStruct *> getAllPoses();

  float msTrackFrame;
  float msOptimizationIteration;
  float msFindConstraintsItaration;
  float msFindReferences;

  int nTrackFrame;
  int nOptimizationIteration;
  int nFindConstraintsItaration;
  int nFindReferences;

  float nAvgTrackFrame;
  float nAvgOptimizationIteration;
  float nAvgFindConstraintsItaration;
  float nAvgFindReferences;

  std::chrono::high_resolution_clock::time_point lastHzUpdate;

private:
  // ============= EXCLUSIVELY TRACKING THREAD (+ init) ===============
  TrackingReference *trackingReference; // tracking reference for current
  // keyframe. only used by tracking.
  SE3Tracker *m_pTracker = nullptr;

  // ============= EXCLUSIVELY MAPPING THREAD (+ init) =============
  DepthMap *m_pMap = nullptr;
  TrackingReference *mappingTrackingReference;

  // during re-localization used
  std::vector<Frame *> KFForReloc;
  int nextRelocIdx;
  std::shared_ptr<Frame> latestFrameTriedForReloc;

  // ============= EXCLUSIVELY FIND-CONSTRAINT THREAD (+ init) =============
  TrackableKeyFrameSearch *trackableKeyFrameSearch;
  Sim3Tracker *constraintTracker;
  SE3Tracker *constraintSE3Tracker;
  TrackingReference *newKFTrackingReference;
  TrackingReference *candidateTrackingReference;

  // ============= SHARED ENTITIES =============
  float tracking_lastResidual;
  float tracking_lastUsage;
  float tracking_lastGoodPerBad;
  float tracking_lastGoodPerTotal;

  int lastNumConstraintsAddedOnFullRetrack;
  bool doFinalOptimization;
  float lastTrackingClosenessScore;

  // for sequential operation. Set in Mapping, read in Tracking.
  boost::condition_variable newFrameMappedSignal;
  boost::mutex newFrameMappedMutex;

  // USED DURING RE-LOCALIZATION ONLY
  Relocalizer relocalizer;

  // Individual / no locking
  Output3DWrapper *outputWrapper; // no lock required
  KeyFrameGraph *keyFrameGraph;   // has own locks

  // Tracking: if (!create) set candidate, set create.
  // Mapping: if (create) use candidate, reset create.
  // => no locking required.
  std::shared_ptr<Frame> latestTrackedFrame;
  bool createNewKeyFrame;

  // PUSHED in tracking, READ & CLEARED in mapping
  std::deque<std::shared_ptr<Frame>> unmappedTrackedFrames;
  boost::mutex unmappedTrackedFramesMutex;
  boost::condition_variable unmappedTrackedFramesSignal;

  // PUSHED by Mapping, READ & CLEARED by constraintFinder
  std::deque<Frame *> newKeyFrames;
  boost::mutex newKeyFrameMutex;
  boost::condition_variable newKeyFrameCreatedSignal;

  // SET & READ EVERYWHERE
  std::shared_ptr<Frame> currentKeyFrame; // changed (and, for VO, maybe
  // deleted)  only by Mapping thread
  // within exclusive lock.
  std::shared_ptr<Frame>
      trackingReferenceFrameSharedPT; // only used in odometry-mode, to keep
  // a keyframe alive until it is
  // deleted. ONLY accessed whithin
  // currentKeyFrameMutex lock.
  boost::mutex mCurrentKeyFrameMutex;

  // threads
  boost::thread thread_mapping;
  boost::thread thread_constraint_search;
  boost::thread thread_optimization;
  bool keepRunning; // used only on destruction to signal threads to finish.

  // optimization thread
  bool newConstraintAdded;
  boost::mutex newConstraintMutex;
  boost::condition_variable newConstraintCreatedSignal;
  boost::mutex g2oGraphAccessMutex;

  // optimization merging. SET in Optimization, merged in Mapping.
  bool haveUnmergedOptimizationOffset;

  // mutex to lock frame pose consistency. within a shared lock of this,
  // *->getScaledCamToWorld() is
  // GUARANTEED to give the same result each call, and to be compatible to
  // each other.
  // locked exclusively during the pose-update by Mapping.
  boost::shared_mutex poseConsistencyMutex;

  bool depthMapScreenshotFlag;
  std::string depthMapScreenshotFilename;

  /** Merges the current keyframe optimization offset to all working entities.
   */
  void mergeOptimizationOffset();

  void mappingThreadLoop();

  void finishCurrentKeyframe();
  void discardCurrentKeyframe();

  void changeKeyframe(bool noCreate, bool force, float maxScore);
  void createNewCurrentKeyframe(std::shared_ptr<Frame> newKeyframeCandidate);
  void loadNewCurrentKeyframe(Frame *keyframeToLoad);

  bool updateKeyframe();

  void addTimingSamples();

  void debugDisplayDepthMap();

  void takeRelocalizeResult();

  void constraintSearchThreadLoop();

  /**
   * Calculates a scale independent error norm for reciprocal tracking
   * results a and b with associated information matrices.
   * @param A
   * @param B
   * @param lvlStart
   * @param lvlEnd
   * @param useSSE
   * @param AtoB
   * @param BtoA
   * @param e1
   * @param e2
   *
   * @Returns
   */
  float tryTrackSim3(TrackingReference *A, TrackingReference *B, int lvlStart,
                     int lvlEnd, bool useSSE, Sim3 &AtoB, Sim3 &BtoA,
                     KFConstraintStruct *e1 = 0, KFConstraintStruct *e2 = 0);

  void testConstraint(Frame *candidate, KFConstraintStruct *&e1_out,
                      KFConstraintStruct *&e2_out,
                      const Sim3 &candidateToFrame_initialEstimate,
                      float strictness);

  /**
   * @brief Optimization Thread Loop
   */
  void optimizationThreadLoop();
};
} // namespace lsd_slam
