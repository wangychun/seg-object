/*
 * image_track.h
 *
 *  Created on: Nov 3, 2019
 *      Author: sarah
 */

#ifndef SRC_IMAGE_PROCESS_SRC_IMAGE_TRACK_H_
#define SRC_IMAGE_PROCESS_SRC_IMAGE_TRACK_H_

#include "tracker_base.h"
#include <deep_sort/multi_object_track.h>

class TrackerFusion : public TrackerBase
{
public:
  // Constructors
  TrackerFusion();

  void initiateTrack(sensors_fusion::ObjectTrack& detection, long long track_id, double time_stamp);

  /**
   * @brief Using detection results to update tracking state, including predict, match and update
   * @param detections
   * @param time_stamp
   */
  void update(sensors_fusion::ObjectTrackArrayPtr detections, double time_stamp);

  void setImageRaw(cv::Mat& image_in) { image_raw_ = image_in; }

  /**
   * @brief Get valid output object tracking array
   * @param track_array
   */
  void getObjectTrackArray(sensors_fusion::ObjectTrackArray& track_array);

private:
  /**
   * @brief Draw object tracking results in camera raw image
   * @param detections[in]  Object detection results
   * @param ds_track_map[in] Deep sort tracking results, contain track information
   * @param ds_tracker_result All tracked object in deep sort
   */
  void drawMatchResult(sensors_fusion::ObjectTrackArrayPtr detections,
                       std::map<int, deep_sort::TrackResult>& ds_track_map,
                       deep_sort::TTrackerP ds_tracker_result);

  void printMatchResult(deep_sort::MatchResult match_result,
                        sensors_fusion::ObjectTrackArrayPtr detections);

private:
  //! Deep sort image 2D bboxes multi object track
  deep_sort::MultiObjectTrack ds_multi_track_;

  // Camera image
  cv::Mat image_raw_;
};

typedef std::shared_ptr<TrackerFusion> TrackerFusionPtr;



#endif /* SRC_IMAGE_PROCESS_SRC_IMAGE_TRACK_H_ */
