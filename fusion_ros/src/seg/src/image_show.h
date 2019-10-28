/*
 * image_show.h
 *
 *  Created on: Oct 24, 2019
 *      Author: sarah
 */

#ifndef SRC_SEG_SRC_IMAGE_SHOW_H_
#define SRC_SEG_SRC_IMAGE_SHOW_H_

//C++
#include <chrono>
#include <vector>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//OpenCV
#include <opencv2/opencv.hpp>
//Eigen
#include <Eigen/Dense>
//#include "utils/bbox_2d.h"
namespace sensors_fusion {

/*!
 * @brief Given 3D point and 3x4 project matrix, this function calculate the projected point in image
 *        Internal just do matrix multiply
 * @param pointIn            3D point coordinate
 * @param projectmatrix      3x4 project matrix
 * @param point_project[out] what we want
 * @return
 */
template <class PointT>
bool ProjectPoint2Image(PointT& pointIn,  Eigen::MatrixXf& projectmatrix, cv::Point2f& point_project)
{
  //check project matrix size
  if(!(projectmatrix.rows()==3&&projectmatrix.cols()==4)) {
    throw std::logic_error("[WARN] project matrix size need be 3x4!");
  }

  if(pointIn.x < 0)  return false;

  //apply the projection operation
  Eigen::Vector4f point_3d(pointIn.x, pointIn.y, pointIn.z, 1.0);//define homogenious coordniate
  Eigen::Vector3f point_temp = projectmatrix*point_3d;

  //get image coordinates
  float x = static_cast<float>(point_temp[0]/point_temp[2]);
  float y = static_cast<float>(point_temp[1]/point_temp[2]);
  point_project = cv::Point2f(x,y);
  return true;
}

/*!
 * @brief project cloud points to image and use color to encode points distance
 * @param cloudIn
 * @param imageIn
 * @param projectmatrix
 * @return  final image with point cloud
 */
template <typename PointT>
cv::Mat ProjectCloud2Image(pcl::PointCloud<PointT>& cloudIn, cv::Mat& imageIn,  Eigen::MatrixXf& projectmatrix)
{
  cv::Mat hsv_image, res_image;
  cv::cvtColor(imageIn, hsv_image, CV_BGR2HSV);
  int scale = 120, min_dis = 1, max_dis = 70;

  //int scale = 120, min_dis = 1, max_dis = 70;

  /// --------------distance to color conversion-----------
  auto toColor = [&]( PointT& point)->int{
    //1)calculate point distance
    float distance = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    //2)calculate color value, normalize values to (0 - scale) & close distance value has low value.
    int res = ((distance - min_dis) / (max_dis - min_dis))*scale;
    return res;
  };
  // plot color points using distance encode HSV color
  for(int i = 0;i<cloudIn.size();++i) {
    if(cloudIn.points[i].x < 0)   //将车后的点滤掉
      continue;
    int color = toColor(cloudIn.points[i]);
    cv::Point2f image_point;
    if(!ProjectPoint2Image(cloudIn.points[i],projectmatrix, image_point))
      continue;

//    ProjectPoint2Image(cloudIn.points[i],projectmatrix, image_point);
    cv::circle(hsv_image, image_point, 2, cv::Scalar(color, 255, 255), -1);
  }
  cv::cvtColor(hsv_image, res_image,CV_HSV2BGR);
  return res_image;
}

/**
 * @brief Project RGB cloud points to camera image, every points in cloud with the color property
 * @param cloudIn[in]
 * @param imageIn[in]
 * @param projectMatrix[in]
 * @return
 */
//cv::Mat ProjectRGBCloud2Image( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn,  cv::Mat& imageIn,  Eigen::MatrixXf& projectMatrix);
cv::Mat ProjectRGBCloud2Image(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, cv::Mat& imageIn, Eigen::MatrixXf& projectMatrix);

/**
 * @brief given cloud vector and project matrix, create the bounding boxes contains cloud in image
 * @param cloudVecIn
 * @param projectMatrix
 * @param imageSize
 * @return
 */
//std::vector<BBox2D> Create2DBBox( std::vector<pcl::PointCloud<pcl::PointXYZI>> cloudVecIn,  Eigen::MatrixXf& projectMatrix,  cv::Size& imageSize);

/*!
 * Class singleton design for lidar point cloud to image projection
 */
class ProjectionSingleton
{
public:
  enum DatasetSource {
    KITTI,
    OnBoard
  };
  static ProjectionSingleton* getInstance();

  void init(Eigen::MatrixXf transform_matrix = Eigen::MatrixXf::Zero(3,4));
  Eigen::MatrixXf& getTransformMatrix() { return transform_matrix_; }
  void setTransformMatrix(Eigen::MatrixXf transform_matrix) { transform_matrix_ = transform_matrix; }
private:
  ProjectionSingleton(){};
  virtual ~ProjectionSingleton(){}
  ProjectionSingleton(ProjectionSingleton& rhs);
  ProjectionSingleton& operator=(ProjectionSingleton& rhs);

private:
  static ProjectionSingleton* uniqueInstance;
  DatasetSource dataset_source_;
  Eigen::MatrixXf transform_matrix_;
};

/*!
 * @brief from bounding boxes to extract all
 * @param bboxes
 * @param cloudIn
 * @param transform_matrix
 * @return
 */
//std::vector<pcl::PointCloud<pcl::PointXYZI> > ExtractCloudFromBBoxes( std::vector<BBox2DBase>& bboxes, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, Eigen::MatrixXf transform_matrix);

} /* namespace sensors_fusion */




#endif /* SRC_SEG_SRC_IMAGE_SHOW_H_ */
