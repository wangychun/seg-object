/*
 * image_show.cpp
 *
 *  Created on: Oct 24, 2019
 *      Author: sarah
 */

//header:frame_id: /kitti_tracking_player
//height:  375
//width:  1242
//encoding: bgr8

#include "image_show.h"
#include <cmath>
using namespace cv;
using std::cout;
using std::endl;
using std::vector;
namespace sensors_fusion {

//cv::Mat ProjectRGBCloud2Image(  pcl::PointCloud<pcl::PointXYZRGB>:: Ptr cloudIn,   cv::Mat& imageIn,   Eigen::MatrixXf& projectMatrix)
//{
//  Mat res_image = imageIn.clone();
//  for(int i = 0; i < cloudIn->size(); ++i) {
//    if(cloudIn->points[i].y < 0)
//      continue;
//    Scalar color(cloudIn->points[i].b, cloudIn->points[i].g, cloudIn->points[i].r);
//    cv::Point2f image_point;
//    ProjectPoint2Image(cloudIn->points[i],projectMatrix,image_point);
//    cv::circle(res_image, image_point, 2,color, -1);
//  }
//  return res_image;
//}
//wangyc注：不知道为什么用 Ptr
cv::Mat ProjectRGBCloud2Image(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, cv::Mat& imageIn, Eigen::MatrixXf& projectMatrix)
{
  Mat res_image = imageIn.clone();
  for(int i = 0; i < cloudIn->size(); ++i) {
    if(cloudIn->points[i].y < 0)
      continue;
    Scalar color(cloudIn->points[i].b, cloudIn->points[i].g, cloudIn->points[i].r);
    cv::Point2f image_point;
    ProjectPoint2Image(cloudIn->points[i],projectMatrix,image_point);
    cv::circle(res_image, image_point, 2,color, -1);
  }
  return res_image;
}

//std::vector<BBox2D> Create2DBBox(  std::vector<pcl::PointCloud<pcl::PointXYZI>> cloudVecIn,   Eigen::MatrixXf& projectMatrix,   cv::Size& imageSize)
//{
//  std::vector<BBox2D> bbox_vec_res;
//  for(int i = 0; i < cloudVecIn.size(); ++i) {
//    BBox2D bbox(cloudVecIn[i], projectMatrix, imageSize);
//    bbox_vec_res.push_back(bbox);
//  }
//  return bbox_vec_res;
//}

///////////////////////////////////////ProjectionSingleton/////////////////////////////////////////
ProjectionSingleton* ProjectionSingleton::uniqueInstance = new ProjectionSingleton();

ProjectionSingleton* ProjectionSingleton::getInstance()
{
  return uniqueInstance;
}

void ProjectionSingleton::init(Eigen::MatrixXf transform_matrix)
{
  if(!transform_matrix.isZero())
    transform_matrix_ = transform_matrix;
  else {

    Eigen::Matrix4f RT_velo_to_cam;
    Eigen::Matrix4f R_rect_00;
    Eigen::MatrixXf project_matrix(3,4);

    RT_velo_to_cam<<7.533745e-03,-9.999714e-01,-6.166020e-04,-4.069766e-03,
                      1.480249e-02,7.280733e-04,-9.998902e-01, -7.631618e-02,
                      9.998621e-01,7.523790e-03,1.480755e-02,  -2.717806e-01,
                      0.0, 0.0, 0.0, 1.0;
    R_rect_00<<9.999239e-01,9.837760e-03,-7.445048e-03, 0.0,
                -9.869795e-03,9.999421e-01,-4.278459e-03, 0.0,
                 7.402527e-03,4.351614e-03,9.999631e-01, 0.0,
                 0.0, 0.0, 0.0, 1.0;

    project_matrix<<7.215377e+02,0.000000e+00,6.095593e+02,4.485728e+01,
                    0.000000e+00,7.215377e+02,1.728540e+02,2.163791e-01,
                    0.000000e+00,0.000000e+00,1.000000e+00,2.745884e-03;

        transform_matrix_ = project_matrix*R_rect_00*RT_velo_to_cam;

  }
}

//std::vector<pcl::PointCloud<pcl::PointXYZI> > ExtractCloudFromBBoxes(  std::vector<BBox2DBase>& bboxes, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, Eigen::MatrixXf transform_matrix)
//{
//  std::vector<pcl::PointCloud<pcl::PointXYZI> > cloudVec(bboxes.size());
//  //1) loop for every point in cloud
//  for(  auto& point : *cloudIn) {
//    if(point.y < 0)
//      continue;
//    cv::Point2f imgPoint;
//    ProjectPoint2Image(point, transform_matrix, imgPoint);
//    //2) loop every bbox in input bboxes
//    for(int i = 0; i < bboxes.size(); ++i) {
//      if(bboxes[i].bbox_.contains(imgPoint)) {
//        cloudVec[i].push_back(point); // push back point
//      }
//    }
//  }
//  return cloudVec;
//}

} /* namespace sensors_fusion */
