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

//	cv::imshow("pic",pic_origin);
//	cv::waitKey();
	return pic_origin;

}

cv::Mat pic_closed(array<array<Cell, numY>, numX> & Grid){
	cv::Mat image = to_pic(Grid);

	cv::Mat element3(3,3,CV_8U,cv::Scalar(1));
	cv::Mat element5(5,5,CV_8U,cv::Scalar(1));
	cv::Mat element7(7,7,CV_8U,cv::Scalar(1));

	cv::Mat image_closed;

	cv::morphologyEx(image, image_closed, cv::MORPH_CLOSE, element3);

//	cv::imshow("pic",image_closed);
//	cv::waitKey();

	return image_closed;
}

ImgBasic updatemark(array<array<Cell, numY>, numX> & Grid){
	ImgBasic image_basic;

	image_basic.image = pic_closed(Grid);

	int num = cv::connectedComponentsWithStats(image_basic.image, image_basic.labels,
										image_basic.stats, image_basic.centroids);
	//包括背景的轮廓数
	image_basic.num = num;

	for(int rows = 0; rows < 300; rows++)
			for(int cols = 0; cols < 400; cols++)
			{
					int label;
					label = image_basic.labels.at<int>(rows, cols);
					if (label != 0)
					Grid[rows][cols].mark = label + 1;
			}

	return image_basic;
}

void classfiyAndSave(array<array<Cell, numY>, numX> & Grid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

		ImgBasic img;
		img = updatemark(Grid);

		std::vector<ObjBasic> obj_basic;

		std::string color[]={"yellow", "red", "brown", "green", "violet", "pink", "azure", "blue", "orange", "white"};
	    default_random_engine e;
	    uniform_int_distribution<int> u(0, 9);
//	    int i = 0;
		int i = 2;
	    std::vector<int> mark_to_checked(img.num + 2, 0);
		for(int mark_bh = 2; mark_bh < img.num + 2; mark_bh++)
		{

			ObjBasic obj_ba;
			int mark_index = mark_bh - 1; // 图像中给的mark

			obj_ba.mark_origin = mark_bh; //点云的原始标注

			if (img.stats.at<int>(mark_index, 4) > area_grid_max_thre) continue;
			if (img.stats.at<int>(mark_index, 4) < area_grid_min_thre) continue;
			if (img.stats.at<int>(mark_index, 2) < 2 || img.stats.at<int>(mark_index, 3) < 2 )continue;

			if (img.stats.at<int>(mark_index, 2) * img.stats.at<int>(mark_index, 3) > area_lw_max_thre ) continue;

			obj_ba.mark_checked = i;


			obj_ba.center_x = img.centroids.at<double>(mark_index,0) * Resolution - 30; //转换到车体坐标系
			obj_ba.center_y = - img.centroids.at<double>(mark_index,1) * Resolution + 40;


			obj_ba.area = img.stats.at<int>(mark_index, 4) * Resolution * Resolution ;


			mark_to_checked[obj_ba.mark_origin] = obj_ba.mark_checked; //从label为2开始。其他全部为0


			obj_basic.push_back(obj_ba); //vector类本身从0开始，但是mark从2开始

			i++;
		}


		classify_cloud(Grid , obj_basic, mark_to_checked);

//		std::cout << "轮廓数:" << i - 2 << std::endl;
		//可视化看一下效果
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_marked_joint(new pcl::PointCloud<pcl::PointXYZRGB>);
//
		for(int mark = 0; mark < obj_basic.size(); mark++)
		{
////			cout <<"nark" << mark << ":" << obj_basic_new[mark].obj_ids.size() << endl;
//
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_marked(new pcl::PointCloud<pcl::PointXYZ>);
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_marked_color(new pcl::PointCloud<pcl::PointXYZRGB>);
//
			cloud_marked = Ids_to_cloud( obj_basic[mark].obj_ids, cloud);
//			cloud_marked_color = set_color_cloud(cloud_marked, color[u(e)]);
//			*cloud_marked_joint += *cloud_marked_color;
//
//			cout <<"nark" << mark << ":" << obj_basic[mark-2].obj_ids.size() << endl;
		}
//
//		save_mark_color_cloud(cloud_marked_joint, "joint");


}


