/*
 * multi_object_tracking.h
 *
 *  Created on: Nov 3, 2019
 *      Author: sarah
 */

#ifndef SRC_MULTI_OBJECT_TRACKING_H_
#define SRC_MULTI_OBJECT_TRACKING_H_

// C++
#include <array>
#include <vector>
// ROS
#include <ros/ros.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>

#include <cv_bridge/cv_bridge.h>

#include "img_detection.h"
#include "tracker_base.h"
#include "image_track.h"


class MultiObjectTracking
{
public:

	MultiObjectTracking();
  void process(darknet_ros_msgs::ImageWithBBoxes &img_with_boxes);
//  void process_test();

private:
  bool transformCoordinate(sensors_fusion::ObjectTrackArray& obj_array, double time_stamp);

private:
	sensors_fusion::FusionDetection image_detect;

//  TrackerBaseP track_ptr_;//! muliti object tracking class
  TrackerFusionPtr tracker_fuion_ptr_; //! tracker fusion class for combine deep sort track
//
////  visualization::Visualization rviz_vis_;
//
//  double time_pre_; //! For memory the time
};




#endif /* SRC_MULTI_OBJECT_TRACKING_H_ */
