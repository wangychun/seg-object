/*
 * tracker_base.cpp
 *
 *  Created on: Nov 3, 2019
 *      Author: sarah
 */

#include "tracker_base.h"
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using tracking::UnscentedKFTracker;
using tracking::UnscentedKF;
using std::cout; using std::endl;
using std::vector;

TrackerBase::TrackerBase():
//  tfListener_(tfBuffer_),
  frame_counter_(0)
{
  params_.da_ped_dist_pos = 1.0;
  params_.da_ped_dist_form = 2.0;
  params_.da_car_dist_pos = 2.0;
  params_.da_car_dist_form = 5.0;
  time_stamp_ = 0;//
  next_id_ = 1;
  cout<<"*********************************frame*****************************"<<endl;
}

TrackerBase::~TrackerBase() {
  // TODO Auto-generated destructor stub
}

void TrackerBase::initiateTrack(const sensors_fusion::ObjectTrack& detection, double time_stamp)
{
  UnscentedKFTracker newt(new UnscentedKF());
  newt->initialize(next_id_, detection, time_stamp);
  kalmanTrackers_.push_back(newt);
  next_id_ += 1;
}

void TrackerBase::update(const sensors_fusion::ObjectTrackArrayPtr detections, double time_stamp)
{
  time_stamp_ = time_stamp;

  // =====================Predict first=========================
  for(UnscentedKFTracker kalmanTrack : kalmanTrackers_)
    kalmanTrack->predict(time_stamp);


  // =====================Objects matching======================
  MatchResult matchRes = this->match(detections);

  // =====================Update track set=====================
  // -matches
  for(size_t i = 0; i < matchRes.matches.size(); i++){
    std::pair<int, int> pa = matchRes.matches[i];
    int track_idx = pa.first;
    int detection_idx = pa.second;
    kalmanTrackers_[track_idx]->update(detections->at(detection_idx));
  }

  // -unmatched track
  for(size_t i = 0; i < matchRes.unmatched_tracks.size(); i++){
    int track_idx = matchRes.unmatched_tracks[i];
    kalmanTrackers_[track_idx]->mark_missed();
  }

  // -unmatched detect
  for(size_t i = 0; i < matchRes.unmatched_detections.size(); i++){
    int detection_idx = matchRes.unmatched_detections[i];
    this->initiateTrack(detections->at(detection_idx), time_stamp);
  }

  // Delete missed many times tracks
  for (auto it = kalmanTrackers_.begin(); it != kalmanTrackers_.end(); ) {
    UnscentedKFTracker p = *it;
    if (p->is_deleted()) {
      it = kalmanTrackers_.erase(it);
    }
    else
      ++it;
  }
}

void TrackerBase::getObjectTrackArray(sensors_fusion::ObjectTrackArray& track_array)
{
  // Loop over all tracks
  for(size_t i = 0; i < kalmanTrackers_.size(); ++i){

      // Grab track
      tracking::Track & track = kalmanTrackers_[i]->track_;

      // Create new message and fill it
      sensors_fusion::ObjectTrack track_msg;
      track_msg.id = track.id;

      track_msg.velocity = track.sta.x[2];
      track_msg.heading = track.sta.x[3];

      track_msg.width = track.geo.width;
      track_msg.length = track.geo.length;
      track_msg.height = track.geo.height;
      track_msg.orientation = track.geo.orientation;
      track_msg.object_type = track.sem.name;
      track_msg.confidence = track.sem.confidence;

      track_msg.is_valid = true;

      // Push back track message
      track_array.push_back(track_msg);
  }
}

MatchResult TrackerBase::match(const sensors_fusion::ObjectTrackArrayPtr detections)
{
  MatchResult re = GlobalNearestNeighbor(detections);
  return re;
}

MatchResult TrackerBase::GlobalNearestNeighbor(const sensors_fusion::ObjectTrackArrayPtr& detected_objects)
{
  // Get track and detection objects indices
  vector<int> track_indices(kalmanTrackers_.size()),
              detection_indices(detected_objects->size());

  // Define assoication result
  MatchResult mr;
  vector<int> matched_track_vec, matched_detect_vec;

  // Loop through tracks
  for(size_t i = 0; i < kalmanTrackers_.size(); ++i){

    // Buffer variables
    std::vector<float> distances;
    std::vector<int> matches;

    // Set data association parameters depending on if
    // the track is a car or a pedestrian
    float gate;
    float box_gate;

    // Pedestrian
    if(kalmanTrackers_[i]->track_.sem.name == "Pedestrian" ||
       kalmanTrackers_[i]->track_.sem.name == "Cyclist"){
      gate = params_.da_ped_dist_pos;
      box_gate = params_.da_ped_dist_form;
    }
    else{
      gate = params_.da_car_dist_pos;
      box_gate = params_.da_car_dist_form;
      ROS_WARN("Object type is not Pedetrian or Cyclist for track [%d]", kalmanTrackers_[i]->track_.id);
    }

    // Loop through detected objects
    for(size_t j = 0; j < detected_objects->size(); ++j){

      // Calculate distance between track and detected object
        float dist = CalculateDistance(kalmanTrackers_[i]->track_,
            detected_objects->at(j));

        if(dist < gate){
          distances.push_back(dist);
          matches.push_back(j);
        }
    }

    // If track exactly finds one match assign it
    if(matches.size() == 1){

      float box_dist = CalculateEuclideanAndBoxOffset(kalmanTrackers_[i]->track_,
          detected_objects->at(matches[0]));
      if(box_dist < box_gate){
        mr.matches.push_back(std::make_pair(i, matches[0]));
        matched_track_vec.push_back(i);
        matched_detect_vec.push_back(matches[0]);
      }
    }
    // If found more then take best match and block other measurements
    else if(matches.size() > 1){

      // Block other measurements to NOT be initialized
      ROS_WARN("Multiple associations for track [%d]", kalmanTrackers_[i]->track_.id);

      // Calculate all box distances and find minimum
      float min_box_dist = box_gate;
      int min_box_index = -1;

      for(size_t k = 0; k < matches.size(); ++k){

        float box_dist = CalculateEuclideanAndBoxOffset(kalmanTrackers_[i]->track_,
            detected_objects->at(matches[k]));

        if(box_dist < min_box_dist){
          min_box_index = k;
          min_box_dist = box_dist;
        }
      }

      for(size_t k = 0; k < matches.size(); ++k){
        if(k == min_box_index){
          mr.matches.push_back(std::make_pair(i, matches[k]));
          matched_track_vec.push_back(i);
          matched_detect_vec.push_back(matches[k]);
        }
        else{
//          mr.unmatched_detections.push_back(matches[k]);
          // TODO: Think these detect objects as matched, so don't consider them
          matched_detect_vec.push_back(matches[k]);
        }
      }
    }
    else{
      ROS_WARN("No measurement found for track [%d]", kalmanTrackers_[i]->track_.id);
    }
  }// end for(int i = 0; i < kalmanTrackers_.size(); ++i)

  // Get unmatched objects
  for (size_t i = 0; i < kalmanTrackers_.size(); i++) {
    const auto& it = std::find(matched_track_vec.begin(), matched_track_vec.end(), i);
    if (it == matched_track_vec.end()) {
      mr.unmatched_tracks.push_back(i);
    }
  }
  for (size_t i = 0; i < detected_objects->size(); i++) {
    const auto& it = std::find(matched_detect_vec.begin(), matched_detect_vec.end(), i);
    if (it == matched_detect_vec.end()) {
      mr.unmatched_detections.push_back(i);
    }
  }
  return mr;
}


float TrackerBase::CalculateDistance(const tracking::Track & track,
    const sensors_fusion::ObjectTrack & object){

    // Calculate euclidean distance in x,y,z coordinates of track and object
    return abs(track.sta.x(0) - object.world_pos.point.x) +
        abs(track.sta.x(1) - object.world_pos.point.y) +
        abs(track.sta.z - object.world_pos.point.z);
}

float TrackerBase::CalculateBoxMismatch(const tracking::Track & track,
    const sensors_fusion::ObjectTrack & object){

    // Calculate mismatch of both tracked cube and detected cube
    float box_wl_switched =  abs(track.geo.width - object.length) +
        abs(track.geo.length - object.width);
    float box_wl_ordered = abs(track.geo.width - object.width) +
        abs(track.geo.length - object.length);
    float box_mismatch = (box_wl_switched < box_wl_ordered) ?
        box_wl_switched : box_wl_ordered;
    box_mismatch += abs(track.geo.height - object.height);
    return box_mismatch;
}

float TrackerBase::CalculateEuclideanAndBoxOffset(const tracking::Track & track,
    const sensors_fusion::ObjectTrack & object){

    // Sum of euclidean offset and box mismatch
    return CalculateDistance(track, object) +
        CalculateBoxMismatch(track, object);
}



