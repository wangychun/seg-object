/*
 * pic_handle.cpp
 *
 *  Created on: Sep 24, 2019
 *      Author: sarah
 */
#include "pic_handle.h"


cv::Mat to_pic(array<array<Cell, numY>, numX> & Grid){
	cv::Mat pic_origin = cv::Mat::zeros(300, 400, CV_8U);
	cv::Mat image = cv::Mat::zeros(239, 211, CV_8U);


	for(int rows = 0; rows < 300; rows++)
		for(int cols = 0; cols < 400; cols++)
		{
			if (Grid[rows][cols].mark == 1) pic_origin.at<uchar>(rows, cols) = 255;
		}

//	cv::imshow("pic origin", pic_origin);

//	cv::waitKey(0);//wait keyboard;then close pic;

	return pic_origin;
}

cv::Mat pic_closed(array<array<Cell, numY>, numX> & Grid){
	cv::Mat image = to_pic(Grid);
	cv::Mat image_blur;
//	cv::medianBlur(image, image_blur ,3);
	cv::medianBlur(image, image_blur ,1);
	cv::imshow("pic_blur",image_blur);

	cv::Mat element3(3,3,CV_8U,cv::Scalar(1));
	cv::Mat element5(5,5,CV_8U,cv::Scalar(1));
	cv::Mat element7(7,7,CV_8U,cv::Scalar(1));

	cv::Mat image_closed;
	cv::morphologyEx(image_blur, image_closed, cv::MORPH_CLOSE, element3);
//	cv::morphologyEx(image_blur, image_closed, cv::MORPH_OPEN, element5);
//	cv::dilate(image_blur, image_dilate, element3)

//	cv::imshow("pic_close",image_closed);

//	cv::waitKey(0);
	return image_closed;
}

ImgBasic updatemark(array<array<Cell, numY>, numX> & Grid){
	ImgBasic image_basic;
	image_basic.image = pic_closed(Grid);

	int num = cv::connectedComponentsWithStats(image_basic.image, image_basic.labels,
										image_basic.stats, image_basic.centroids);
	//包括背景的轮廓数
	image_basic.num = num;
	std::cout << "轮廓数" << num << std::endl;
//	std::cout << image_basic.labels << std::endl;
	std::cout << image_basic.labels.rows<<"*"<< image_basic.labels.cols <<std::endl;
	for(int rows = 0; rows < 300; rows++)
			for(int cols = 0; cols < 400; cols++)
			{
					int label;
					label = image_basic.labels.at<int>(rows, cols);
					if (label != 0)
					Grid[rows][cols].mark = label + 1;
//				if (Grid[rows][cols].mark == 1) cout<<rows<<","<<cols<<endl;
			}
//	cout << "stats:" << endl;
//	cout << image_basic.stats << endl;
//	cout << image_basic.centroids << endl;
//	cout << image_basic.centroids.at<double>(3,1)<<endl;
	return image_basic;
}

void classfiyAndSave(array<array<Cell, numY>, numX> & Grid){

		ImgBasic img;
		img = updatemark(Grid);
		pcl::PointCloud<pcl::PointXYZRGB> cloud_joint;
//		pcl::PointCloud<pcl::PointXYZ> cloud_joint;
		std::vector<ObjBasic> obj_basic;
		//这里对vector直接初始化个数是不对的，后面研究如何用push_back解决这个问题

		std::string color[]={"yellow", "red", "brown", "green", "violet", "pink", "azure", "blue", "orange", "white"};
	    default_random_engine e;
	    uniform_int_distribution<int> u(0, 9);
//	    std::string rand_color = color[u(e)];
	    int i = 0;
		for(int mark_bh = 2; mark_bh < img.num; mark_bh++)
		{
			ObjBasic obj_ba;
			int mark_index = mark_bh - 1; // 图像中给的mark

//			cout << "area" << img.stats.at<int>(mark_index, 4) << endl;
			if (img.stats.at<int>(mark_index, 4) > area_grid_max_thre) continue;
			if (img.stats.at<int>(mark_index, 4) < area_grid_min_thre) continue;
			if (img.stats.at<int>(mark_index, 2) < 2 || img.stats.at<int>(mark_index, 3) < 2 )continue;

			std::string rand_color = color[u(e)];
//			cout << "centx" << obj_basic[i].center_x << endl;

//			cout<< img.centroids.at<double>(mark_index,0)<< endl;
//
			obj_ba.center_x = img.centroids.at<double>(mark_index,0) * Resolution - 30; //转换到车体坐标系
			obj_ba.center_y = - img.centroids.at<double>(mark_index,1) * Resolution + 40;

//			obj_basic[i].center_x = img.centroids.at<double>(mark_index,0)  ;
//			obj_basic[i].center_y = img.centroids.at<double>(mark_index,1) ;

//			cout<< "chetizuobiaoxi"<< endl;

			obj_ba.area = img.stats.at<int>(mark_index, 4) * Resolution * Resolution ;
			obj_ba.mark_origin = mark_bh; //点云的原始标注
			obj_ba.mark_checked = i;

			obj_ba.cloud = classify_cloud(mark_bh , Grid );
			obj_ba.cloud_color = classify_color_cloud(mark_bh, Grid, rand_color);



			obj_basic.push_back(obj_ba);
			if (obj_basic[i].cloud.size() == 0)
				{
				cout << "something wrong in classfiyAndSave!!!" << endl;
				continue;
				}
//			std::cout << "轮廓数" << i << std::endl;
			save_mark_color_cloud(obj_basic[i].cloud_color,std::to_string(i));
//			save_mark_origin_cloud(obj_basic[i].cloud, std::to_string(i));

			cloud_joint += obj_basic[i].cloud_color;
			i++;
		}
		std::cout << "轮廓数" << i << std::endl;

		save_mark_color_cloud(cloud_joint,"joint");

}


