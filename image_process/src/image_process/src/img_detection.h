/*
 * img_detection.h
 *
 *  Created on: Oct 30, 2019
 *      Author: sarah
 *      传感器融合目标跟踪主功能类，与外界接口，使用其他功能类，共同完成
*                1、目标检测
*                2、目标跟踪
*                等不同任务
 */

#ifndef SRC_IMAGE_PROCESS_SRC_IMG_DETECTION_H_
#define SRC_IMAGE_PROCESS_SRC_IMG_DETECTION_H_

//Includes
//C++
#include <iostream>
#include <vector>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

//OpenCV
#include <opencv2/opencv.hpp>
//Eigen
#include <Eigen/Dense>
//Project

#include "bbox_2d.h"
#include <darknet_ros_msgs/ImageWithBBoxes.h>
#include "track_type.h"

#include<cv_bridge/cv_bridge.h>



namespace sensors_fusion {

static std::map<std::string, cv::Scalar> color ={ {"Car", cv::Scalar(142,0,0)},//blue
                                           {"Pedestrian", cv::Scalar(60,20,220)},//淡红色
                                           {"Cyclist", cv::Scalar(32, 11, 119)} };//深红色

class FusionDetection {
public:
  // Default constructor

  void processImage(darknet_ros_msgs::ImageWithBBoxes& darknetMsg);
  void getObjectDetectArray(ObjectTrackArray& object_array) { object_array = clusters_track_; }

  //! Get original color camera image
  cv::Mat getImageRaw() const { return image_raw_; }
  void getobjArray();


//  std::vector<sensors_fusion::BBox2DBase> darknetMsgToBBoxes(const darknet_ros_msgs::ImageWithBBoxesConstPtr& darknetMsg);
  std::vector<sensors_fusion::BBox2DBase> darknetMsgToBBoxes(darknet_ros_msgs::ImageWithBBoxes& darknetMsg);
//  cv::Mat drawBBoxes(const cv::Mat& image_in, const std::vector<sensors_fusion::BBox2DBase>& bbox);
  cv::Mat drawBBoxes(cv::Mat& image_in, std::vector<sensors_fusion::BBox2DBase>& bbox);

  void printObjectInfo(std::string infoSource, const ObjectTrackArray& object);
private:


  // Image
  cv::Mat image_raw_;


  // Class member
  ObjectTrackArray clusters_track_;//! Final detected objects vector
  std::vector<sensors_fusion::BBox2DBase> detectBBoxes_;
  // Init counter
  int frame_count_;

};

} /* namespace sensors_fusion */
//
//static std::map<std::string, cv::Scalar> color ={ {"car", cv::Scalar(142,0,0)},//blue
//                                           {"person", cv::Scalar(60,20,220)},//淡红色
//                                           {"bicycle", cv::Scalar(32, 11, 119)} };//深红色
//
//
//void processImage(darknet_ros_msgs::ImageWithBBoxes::ConstPtr& image);
//
////! From image detection results extract cloud clusters
////void processImage(darknet_ros_msgs::ImageWithBBoxes::ConstPtr& image)
//void boxFitting();
//void getObjectDetectArray(ObjectTrackArray& object_array) { object_array = clusters_track_; }
//
//std::vector<sensors_fusion::BBox2DBase> darknetMsgToBBoxes(darknet_ros_msgs::ImageWithBBoxes& darknetMsg);
//
//cv::Mat drawBBoxes(cv::Mat& image_in, std::vector<sensors_fusion::BBox2DBase>& bbox);
//
//


#endif /* SRC_IMAGE_PROCESS_SRC_IMG_DETECTION_H_ */
