/*
 * multi_object_tracking.cpp
 *
 *  Created on: Nov 3, 2019
 *      Author: sarah
 */
#include "multi_object_tracking.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

MultiObjectTracking::MultiObjectTracking():image_detect()
{
//  track_ptr_ = TrackerBaseP(new TrackerBase);
  tracker_fuion_ptr_ = TrackerFusionPtr(new TrackerFusion);

}


void MultiObjectTracking::process(darknet_ros_msgs::ImageWithBBoxes &img_with_boxes)
{

	 image_detect.processImage(img_with_boxes);
	 image_detect.getobjArray();

	 sensors_fusion::ObjectTrackArray object_array;
	 image_detect.getObjectDetectArray(object_array);
//	 ++ frame_count_;

    cv::Mat image_camera_raw;
    image_camera_raw = image_detect.getImageRaw();
    ROS_INFO_STREAM("Object detect array size is "<<object_array.size());

    // Get time stamp for tracking
    double time_stamp = img_with_boxes.header.stamp.toSec();

    tracker_fuion_ptr_->setImageRaw(image_camera_raw);

    tracker_fuion_ptr_->update(std::make_shared<sensors_fusion::ObjectTrackArray>(object_array), time_stamp);
    ///5) ------Visualize tracking---------
    sensors_fusion::ObjectTrackArray object_track;
//    track_ptr_->getObjectTrackArray(object_track);
    tracker_fuion_ptr_->getObjectTrackArray(object_track);
    ROS_WARN_STREAM("Object track array size is "<<object_track.size());


  }





