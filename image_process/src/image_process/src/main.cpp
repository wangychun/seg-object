#include <iostream>
#include <string>
#include <vector>
//ros订阅
#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
//opencv
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

//darknet
#include <darknet_ros_msgs/ImageWithBBoxes.h>
#include "img_detection.h"
#include "multi_object_tracking.h"
//	float64 probability
//	int64 xmin
//	int64 ymin
//	int64 xmax
//	int64 ymax
//	int16 id
//	string Class
using namespace sensors_fusion;

MultiObjectTracking image_tracker;
void imageCallback(darknet_ros_msgs::ImageWithBBoxes img_with_boxes)
{
	ROS_INFO_STREAM("Received!");
	image_tracker.process(img_with_boxes);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_process");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("Wait image topic!");
	ros::Subscriber img_with_boxes_sub = nh.subscribe("darknet_ros/image_with_bboxes", 1, imageCallback);
	ros::spin();
	return 0;
}
