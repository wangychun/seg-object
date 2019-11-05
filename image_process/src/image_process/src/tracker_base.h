/*
 * tracker_base.h
 *
 *  Created on: Nov 3, 2019
 *      Author: sarah
 */

#ifndef SRC_IMAGE_PROCESS_SRC_TRACKER_BASE_H_
#define SRC_IMAGE_PROCESS_SRC_TRACKER_BASE_H_

#include <tf2_ros/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <deep_sort/typedef.h>

#include "ukf.h"

class TrackerBase;
typedef std::shared_ptr<TrackerBase> TrackerBaseP;

typedef struct MatchResult
{
  std::vector<std::pair<int, int> > matches;
  std::vector<int> unmatched_detections;
  std::vector<int> unmatched_tracks;
}MatchResult;


/**
 * @brief Multi object track class
 */
class TrackerBase
{
public:
  TrackerBase();
  virtual ~TrackerBase();

public:
  // Track objects vector
  std::vector<tracking::UnscentedKFTracker> kalmanTrackers_;

  void initiateTrack(const sensors_fusion::ObjectTrack& detection, double time_stamp);

  /**
   * @brief Using detection results to update tracking state, including predict, match and update
   * @param detections
   * @param time_stamp
   */
  void update(const sensors_fusion::ObjectTrackArrayPtr detections, double time_stamp);

  void getObjectTrackArray(sensors_fusion::ObjectTrackArray& track_array);

protected:
  MatchResult match(const sensors_fusion::ObjectTrackArrayPtr detections);

private:
  // Data Association functions
  MatchResult GlobalNearestNeighbor(const sensors_fusion::ObjectTrackArrayPtr & detected_objects);
  float CalculateDistance(const tracking::Track & track, const sensors_fusion::ObjectTrack & object);
  float CalculateBoxMismatch(const tracking::Track & track, const sensors_fusion::ObjectTrack & object);
  float CalculateEuclideanAndBoxOffset(const tracking::Track & track,
      const sensors_fusion::ObjectTrack & object);

protected:
  // Class member
  tracking::Parameter params_;//先不考虑ukf????
//  tf::TransformListener listener_;
//  tf2_ros::Buffer tfBuffer_;
//  tf2_ros::TransformListener tfListener_;

  int next_id_ = 0;

  double time_stamp_;

  // Frame counter
  int frame_counter_;
};





#endif /* SRC_IMAGE_PROCESS_SRC_TRACKER_BASE_H_ */
