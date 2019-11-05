/*
 * img_detection.cpp
 *
 *  Created on: Oct 30, 2019
 *      Author: sarah
 */

#include "img_detection.h"

namespace sensors_fusion {

void FusionDetection::processImage(darknet_ros_msgs::ImageWithBBoxes& darknetMsg)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(darknetMsg.image, sensor_msgs::image_encodings::BGR8);
	image_raw_ = cv_ptr->image;
	detectBBoxes_ = darknetMsgToBBoxes(darknetMsg);
//	cv::Mat image_draw = drawBBoxes(image_raw, detectBBoxes_);
//		cv::imshow("detect", image_draw);
//		cv::waitKey(1);

	  // Draw bounding boxes
	  cv::Mat image_draw = drawBBoxes(image_raw_, detectBBoxes_);
}

void FusionDetection::getobjArray()
{
	clusters_track_ = std::vector<ObjectTrack>(detectBBoxes_.size());


	    //2) Loop every bbox in input bboxes and initialize clusters_track_
	    for(size_t i = 0; i < detectBBoxes_.size(); ++i)
	    {
	      clusters_track_[i].bbox = detectBBoxes_[i].bbox_;
	      clusters_track_[i].object_type = detectBBoxes_[i].className_;
	      // Initialize valid is true
	      clusters_track_[i].is_valid = true;
	      }
}

std::vector<BBox2DBase> FusionDetection::darknetMsgToBBoxes(darknet_ros_msgs::ImageWithBBoxes & darknetMsg)
{
  std::vector<sensors_fusion::BBox2DBase> res;
  std::vector<darknet_ros_msgs::BoundingBox> bboxVec = darknetMsg.bboxes.bounding_boxes;
  for(const auto& box : bboxVec) {
    if(box.Class == "person" || box.Class == "bicycle" || box.Class == "car") {
    	if(box.probability < 0.4) continue;
//    	std::cout << box.id <<std::endl;
      cv::Rect rect(cv::Point(box.xmin, box.ymin), cv::Point(box.xmax, box.ymax));
      sensors_fusion::BBox2DBase boxBase(rect, box.Class, box.probability, box.id);
      res.push_back(boxBase);
    }
  }
  return res;
}

cv::Mat FusionDetection::drawBBoxes(cv::Mat& image_in, std::vector<sensors_fusion::BBox2DBase>& bbox)
{
  cv::Mat image = image_in.clone();
  auto getRectCenter = [](const cv::Rect& rect)->cv::Point{
    return cv::Point(cvRound(rect.x + rect.width/2.0), cvRound(rect.y + rect.height/2));
  };

  for(const auto& box : bbox) {
    if(box.className_ == "car") {
      cv::rectangle(image, box.bbox_, sensors_fusion::color["car"], 2); // blue
//      cv::circle(image, getRectCenter(box.bbox_), 2, color["Car"], 2);
//      std::string id_name = "id:" + std::to_string(box.id_);
      cv::putText(image, box.className_, getRectCenter(box.bbox_), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(255,255,255),2);
    }
    else if(box.className_ == "person") {
      rectangle(image, box.bbox_, sensors_fusion::color["person"], 2); // green
//      circle(image, getRectCenter(box.bbox_), 2, color["person"], 2);
      cv::putText(image, box.className_, getRectCenter(box.bbox_), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(255,255,255),2);
    }
    else if(box.className_ == "bicycle") {
      cv::rectangle(image, box.bbox_, sensors_fusion::color["bicycle"], 2);
//      circle(image, getRectCenter(box.bbox_), 2, color["bicycle"], 2);
      cv::putText(image, box.className_, getRectCenter(box.bbox_), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(255,255,255),2);
    }
    else {
      cv::rectangle(image, box.bbox_, cv::Scalar(255, 255, 255), 2);
      circle(image, getRectCenter(box.bbox_), 2, cv::Scalar(255, 255, 255), 2);
    }
  }
  return image;
}
}



