/*
 * track_type.h
 *
 *  Created on: Nov 3, 2019
 *      Author: sarah
 */

#ifndef SRC_IMAGE_PROCESS_SRC_TRACK_TYPE_H_
#define SRC_IMAGE_PROCESS_SRC_TRACK_TYPE_H_

//C++
#include <string>

//OpenCV
#include <opencv2/opencv.hpp>
//ROS
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
namespace sensors_fusion {


struct ObjectTrack
{
  std::string id;
  std_msgs::Header header;

  //! Whether this object is a valid 3D object (means has been determined has valid points cloud)
  bool is_valid;
  //! Whether this track object has been confirmed as output track object, means they are mature
  bool is_confirmed;

  std::string object_type;
  float confidence;

  float length, width, height;

  // Color property for visualization
  cv::Scalar color;

  // Objects center Geometry position info in lidar(base) coordinate
  geometry_msgs::PointStamped velo_pos;

  // Geometry center position info in world coordinate
  geometry_msgs::PointStamped world_pos;

  float velocity, heading, orientation;


  cv::Rect bbox;

  // Rotated rectangle in occuppied grid map
  cv::RotatedRect rotated_rect;

  // History trajectory
  visualization_msgs::Marker history_trajectory;

};

typedef std::vector<ObjectTrack> ObjectTrackArray;
typedef std::shared_ptr<ObjectTrackArray> ObjectTrackArrayPtr;

}// end namespace sensors_fusion




#endif /* SRC_IMAGE_PROCESS_SRC_TRACK_TYPE_H_ */
