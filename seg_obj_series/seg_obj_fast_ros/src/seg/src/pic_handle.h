/*
 * pic_handle.h
 *
 *  Created on: Sep 25, 2019
 *      Author: sarah
 */

#ifndef SRC_PIC_HANDLE_H_
#define SRC_PIC_HANDLE_H_

#include "gridmap.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>

#include <array>
#include <vector>
#include <random>

#include <ctime>
#include <ratio>
#include <chrono>

//小轿车：5m*2.5m(0.2栅格：25*13= 325)
//公交车：12m*2.5m(0.2栅格：60*13)
//最多扫到0.8:  0.8*60*13 = 624
const int area_grid_max_thre = 400;
const int area_grid_min_thre = 4;
//扫到车尾，也至少有2m，假设只有1m，也至少面积为5以上，此时设为6
const int area_lw_max_thre = 600;

class ImgBasic{
public:
	cv::Mat image;
	cv::Mat labels;
	cv::Mat stats;
	cv::Mat centroids;
	int num;
};

//class ObjBasic{
//public:
//	double center_x;
//	double center_y;
//	double area;
//	int mark_origin;
//	int mark_checked;
//	std::vector<int> obj_ids;
//};

cv::Mat to_pic(std::array<std::array<Cell, numY>, numX> & Grid);
cv::Mat pic_closed(std::array<std::array<Cell, numY>, numX> & Grid);

ImgBasic updatemark(std::array<std::array<Cell, numY>, numX> & Grid);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr classfiyAndSave(std::array<std::array<Cell, numY>, numX> & Grid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

#endif /* SRC_PIC_HANDLE_H_ */
