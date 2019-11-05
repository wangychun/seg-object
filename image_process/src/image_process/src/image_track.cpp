/*
 * image_track.cpp
 *
 *  Created on: Nov 3, 2019
 *      Author: sarah
 */
#include "image_track.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using tracking::UnscentedKFTracker;
using tracking::UnscentedKF;
using std::cout; using std::endl;
using std::vector;
// Whether saving data for paper writing
static bool _is_save_data = true;

cv::Scalar generateColor(int unique_id)
{
  int hue_value = (unique_id * 5) % 180;
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue_value, 255, 255));
  cv::Mat rgb;
  cv::cvtColor(hsv, rgb, CV_HSV2BGR);

  // Return BGR color value
  cv::Scalar color((int)rgb.at<cv::Vec3b>(0,0)[0],
                   (int)rgb.at<cv::Vec3b>(0,0)[1],
                   (int)rgb.at<cv::Vec3b>(0,0)[2]);
  return color;
}

TrackerFusion::TrackerFusion() : TrackerBase()
{
  ds_multi_track_.Init();
}

void TrackerFusion::initiateTrack(sensors_fusion::ObjectTrack& detection, long long track_id,  double time_stamp)
{
  UnscentedKFTracker newt(new UnscentedKF());
  newt->initialize(track_id, detection, time_stamp);
  kalmanTrackers_.push_back(newt);
}

void TrackerFusion::update(sensors_fusion::ObjectTrackArrayPtr detections, double time_stamp)
{

  std::cout<< "frame_counter_ = " << frame_counter_ <<std::endl;

  ++ frame_counter_;
   time_stamp_ = time_stamp;

   ///// 1) Run deep sort multi track and get deep sort match results
   // Get detection 2D rectangles
   vector<cv::Rect> rcs, outRcs;
   for(auto it = detections->begin(); it != detections->end(); ++it) {
     rcs.push_back(it->bbox);
   }
   std::map<int, deep_sort::TrackResult> ds_track_map =  ds_multi_track_.UpdateAndGet(image_raw_, rcs, outRcs);
   // Get this frame match deep sort match result
   deep_sort::MatchResult ds_match_result = ds_multi_track_.getMatchResult();
   // Get the track objects vector in deep sort for this frame
   deep_sort::TTrackerP ds_tracker_objects_vec = ds_multi_track_.getTTrackerP();


   ///// 2) Predict all valid track objects(is valid condtidion in predict function)
   for (UnscentedKFTracker kalmanTrack : kalmanTrackers_) {
     kalmanTrack->predict(time_stamp);
   }

   printMatchResult(ds_match_result, detections);

   ///// 3) Using deep sort match result to update valid track objects
   // =====================Update track set=====================
   // Matches
   for(size_t i = 0; i < ds_match_result.matches.size(); i++){
     std::pair<int, int> pa = ds_match_result.matches[i];
     int track_idx = pa.first;
     int detection_idx = pa.second;

     // a) Only use valid detection result to update valid track objects
     if(kalmanTrackers_[track_idx]->is_valid_ &&
        detections->at(detection_idx).is_valid){
       kalmanTrackers_[track_idx]->update(detections->at(detection_idx));
     }
     // b) Valid track object and unvalid 3D detections
     else if(kalmanTrackers_[track_idx]->is_valid_ &&
             !detections->at(detection_idx).is_valid){
       kalmanTrackers_[track_idx]->mark_missed();
       if(kalmanTrackers_[track_idx]->is_deleted())
         kalmanTrackers_[track_idx]->reset();
     }
     // b) For unvalid track object, if has valid match detection, check deep sort tracking state
     else if(!kalmanTrackers_[track_idx]->is_valid_ &&
             detections->at(detection_idx).is_valid){
       if(ds_tracker_objects_vec->kalmanTrackers_match_[track_idx]->is_confirmed()){
         long long tr_id = kalmanTrackers_[track_idx]->track_.id;

         kalmanTrackers_[track_idx]->initialize(tr_id, detections->at(detection_idx), time_stamp);
         ROS_ERROR("Enter in this one %d, %f, %f", tr_id, kalmanTrackers_[track_idx]->track_.sta.x[2],
             kalmanTrackers_[track_idx]->track_.sta.x[3]);
 //        kalmanTrackers_[track_idx]->markConfirmed();
       }
     }
   }

   // Unmatched tracks -> mark_missed
   for(size_t i = 0; i < ds_match_result.unmatched_tracks.size(); i++){
     int track_idx = ds_match_result.unmatched_tracks[i];

     // Only mark valid objects missed
     if(kalmanTrackers_[track_idx]->is_valid_){
       kalmanTrackers_[track_idx]->mark_missed();
       // If this valid 3D track object is think should be deleted, we reset its all state
       if(kalmanTrackers_[track_idx]->is_deleted() ||
          !ds_tracker_objects_vec->kalmanTrackers_match_[track_idx]->isOutput())
         kalmanTrackers_[track_idx]->reset();
     }
   }

   /// Tracking managing
   // Get new created and deleted objects in deep sort in this frame
   deep_sort::NewAndDelete ds_new_delete = ds_multi_track_.getNewAndDelete();

   // Unmatched detect -> Create new track object
   for(size_t i = 0; i < ds_match_result.unmatched_detections.size(); ++i){
     int detection_idx = ds_match_result.unmatched_detections[i];
     long long track_id = ds_new_delete.news_[detection_idx];
     this->initiateTrack(detections->at(detection_idx),track_id, time_stamp);
   }

   // Delete tracks according to deep sort tracking manager
   for (auto it = kalmanTrackers_.begin(); it != kalmanTrackers_.end(); ) {
     UnscentedKFTracker p = *it;
     auto iter = std::find_if(ds_new_delete.deletes_.begin(), ds_new_delete.deletes_.end(),
         [&](int obj){
       return obj == p->track_.id;
     });
     if (iter != ds_new_delete.deletes_.end()) {
       it = kalmanTrackers_.erase(it);
     }
     else
       ++it;
   }

   if(kalmanTrackers_.size() != ds_tracker_objects_vec->kalmanTrackers_.size())
     throw std::logic_error("Lidar tracker object vector size is not equal to deep sorts");

   // Show deep sort match result
   drawMatchResult(detections, ds_track_map, ds_tracker_objects_vec);
}

void TrackerFusion::getObjectTrackArray(sensors_fusion::ObjectTrackArray& track_array)
{
  /// ---------------Only output valid 3D objects----------------
  // Loop over all tracks

  for(size_t i = 0; i < kalmanTrackers_.size(); ++i){
    UnscentedKFTracker& tt = kalmanTrackers_[i];
    // If is not valid track or not confirmed, don't output them
//    if(!tt->is_valid_ || !tt->is_confirmed())
    if(!tt->is_valid_)
      continue;

    // Grab track
    tracking::Track & track = kalmanTrackers_[i]->track_;

    // Create new message and fill it
    sensors_fusion::ObjectTrack track_msg;
    track_msg.id = track.id;
    track_msg.is_confirmed = tt->is_confirmed();

//    track_msg.world_pos.header.frame_id = "world";
//    track_msg.world_pos.header.stamp = ros::Time(time_stamp_);

    /// Using the UKF output
//    track_msg.world_pos.point.x = track.sta.x[0];
//    track_msg.world_pos.point.y = track.sta.x[1];
//    track_msg.world_pos.point.z = track.sta.z;
    /// Using the detection result
//    track_msg.world_pos.point = track.sta.world_origin_pos.point;
//
//    try{
//      tfBuffer_.transform(
//          track_msg.world_pos,
//          track_msg.velo_pos,
//          "velo_link");
//    }
//    catch(tf2::TransformException& ex){
//      ROS_ERROR("Received an exception in TrackerBase::getObjectTrackArray: %s", ex.what());
//    }

    track_msg.velocity = track.sta.x[2];
    track_msg.heading = track.sta.x[3];

//     Add velocity constraint
    if(track_msg.velocity <= 1.0)
      continue;

    // Assign unique color for this object
    track_msg.color = generateColor(track.id);

    track_msg.width = track.geo.width;
    track_msg.length = track.geo.length;
    track_msg.height = track.geo.height;
    track_msg.orientation = track.geo.orientation;
    track_msg.object_type = track.sem.name;
    track_msg.confidence = track.sem.confidence;

    // -----------Get track object history trajectory------------------
    track_msg.history_trajectory = track.hist.trajectory;
    track_msg.history_trajectory.header.frame_id = "world";
    track_msg.history_trajectory.header.stamp = ros::Time(time_stamp_);
    track_msg.history_trajectory.id = track.id;

    track_msg.is_valid = true;
    // Push back track message
    track_array.push_back(track_msg);
  }
}

void TrackerFusion::printMatchResult(deep_sort::MatchResult match_result,
                                     sensors_fusion::ObjectTrackArrayPtr detections)
{
  cout<<endl;
  ROS_WARN("Deep sort match result:");
  for(size_t i = 0; i < match_result.matches.size(); i++){
    std::pair<int, int> pa = match_result.matches[i];
    int track_idx = pa.first;
    int detection_idx = pa.second;
    ROS_WARN("Matches: track_idx:[%d],valid[%d] <====> det_idx[%d],valid[%d]",
        kalmanTrackers_[track_idx]->track_.id, kalmanTrackers_[track_idx]->is_valid_,
        detection_idx, detections->at(detection_idx).is_valid);
  }

  for(size_t i = 0; i < match_result.unmatched_tracks.size(); i++){
     int track_idx = match_result.unmatched_tracks[i];
     ROS_WARN("Unmatched Tracks: track_idx:[%d],valid[%d]",
         kalmanTrackers_[track_idx]->track_.id, kalmanTrackers_[track_idx]->is_valid_);
  }

  for(size_t i = 0; i < match_result.unmatched_detections.size(); ++i){
    int detection_idx = match_result.unmatched_detections[i];
    ROS_WARN("Unmatched Detections: detection_idx:[%d],valid[%d]",detection_idx, detections->at(detection_idx).is_valid);
  }
  cout<<endl;
}

void TrackerFusion::drawMatchResult(sensors_fusion::ObjectTrackArrayPtr detections,
                                    std::map<int, deep_sort::TrackResult>& ds_track_map,
                                    deep_sort::TTrackerP ds_tracker_result)
{
  // 1) Clone image raw and draw current detection result
  cv::Mat image_show = image_raw_.clone();
  for(auto it = detections->begin(); it != detections->end(); ++it) {
    cv::rectangle(image_show, it->bbox, cv::Scalar(255, 255, 255)); // white color
    // Display detection ID
//    int idx = it - detections->begin();
//    std::string disp = std::to_string(idx);
//    cv::putText(image_show,
//        disp,
//        it->bbox.br(),
//        CV_FONT_HERSHEY_SIMPLEX,
//        1,
//        cv::Scalar(0, 255, 255),
//        2);
  }

  // 2) Show deep sort tracking result
  for(auto it = ds_track_map.begin(); it != ds_track_map.end(); ++it){
    CvScalar clr = cvScalar(0, 255, 0);// Green color
    cv::Rect rc = it->second.rect;
    int track_num = it->second.track_num;// Tracking times
    cv::rectangle(image_show, rc, clr); // Draw track object rectangle
    // Draw Object track_id and track num
    std::string disp = std::to_string(it->first) + " " + std::to_string(track_num);
    cv::putText(image_show,
        disp,
        cvPoint(rc.x, rc.y),
        CV_FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 0, 255));

    // Draw track object history trajectory
    deep_sort::Trace trajectory = it->second.trajectory;
    cv::Point pre_point;
    for (size_t m = 0; m < trajectory.size() - 1; ++ m) {
      //      cv::circle(frame, trajectory[m], 1, cvScalar(20,225- m*7,100+ m*5),1,8,0);
      cv::line(image_show, trajectory[m], trajectory[m + 1], cvScalar(20,255- m*6,100+ m*5), 1, 8, 0);
    }
  }

  // 3) Show 3D lidar track objects
  bool show_lidar_track_info = false;
  if (show_lidar_track_info) {
    std::vector<deep_sort::KalmanTracker>& kalmanTrackers = ds_tracker_result->kalmanTrackers_;
    for (size_t i = 0; i < kalmanTrackers_.size(); ++i) {
      UnscentedKFTracker& tt = kalmanTrackers_[i];
      // Get track objects in image
      deep_sort::DSBOX box = kalmanTrackers[i]->to_tlwh();
      cv::Rect rc;
      rc.x = box(0);
      rc.y = box(1);
      rc.width = box(2);
      rc.height = box(3);

      //    if(tt->is_valid_) {
      //      cv::rectangle(image_show, rc, cv::Scalar(0,255,0)); // Draw track object rectangle
      //    }
      //    else {
      //      cv::rectangle(image_show, rc, cv::Scalar(0,0,255)); // Draw track object rectangle
      //    }
      // Draw Object ID
      std::string disp = std::to_string(tt->track_.id) + " " + std::to_string(tt->is_valid_)
      + " " + std::to_string(tt->track_.sta.x[2])    + " " + std::to_string(tt->is_confirmed());
      cv::putText(image_show,
          disp,
          cvPoint(rc.x, rc.y + rc.height/2),
          CV_FONT_HERSHEY_SIMPLEX,
          0.3,
          cv::Scalar(0, 0, 255));
    }
  }// end if (show_lidar_track_info)

  cv::namedWindow("tracker_fusion", CV_WINDOW_NORMAL);
  cv::imshow("tracker_fusion", image_show);
  cv::waitKey(5);
  /// ------------Saving results-----------
  if (_is_save_data) {
    std::stringstream file_name_str;
    std::string path = "/home/zhanghm/catkin_ws_perception/image/";
    // 1) Saving original raw camera image
    file_name_str << path << frame_counter_<<"_raw_camera_image.png";
    cv::imwrite(file_name_str.str(), image_raw_);
    file_name_str.str("");

    // 2) Saving match result image
    file_name_str << path << frame_counter_<<"_match_result_image.png";
    cv::imwrite(file_name_str.str(), image_show);
    file_name_str.str("");
  }
}




