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
	cv::Mat image_blur;

//	std::cout << "blur_before"  << std::endl;
	cv::medianBlur(image, image_blur ,3);  //中值滤波

	cv::Mat element3(3,3,CV_8U,cv::Scalar(1));
	cv::Mat element5(5,5,CV_8U,cv::Scalar(1));
	cv::Mat element7(7,7,CV_8U,cv::Scalar(1));

	cv::Mat image_closed;

	cv::morphologyEx(image_blur, image_closed, cv::MORPH_CLOSE, element3);

//	cv::imshow("pic",image_closed);
//	cv::waitKey();

	return image_closed;
}

ImgBasic updatemark(array<array<Cell, numY>, numX> & Grid){
	ImgBasic image_basic;
//	std::cout << "轮廓数"  << std::endl;
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
//					if (label != 0)
					Grid[rows][cols].mark = label;
			}

	return image_basic;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr classfiyAndSave(array<array<Cell, numY>, numX> & Grid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

		ImgBasic img;
		img = updatemark(Grid);

		std::vector<ObjBasic> obj_basic;
//		std::vector<ObjBasic> obj_basic_new;

		std::string color[]={"yellow", "red", "brown", "green", "violet", "pink", "azure", "blue", "orange", "white"};
	    default_random_engine e;
	    uniform_int_distribution<int> u(0, 9);
//	    int i = 0;
		int i = 1;
	    std::vector<int> mark_to_checked(img.num , 0);
		for(int mark_bh = 0; mark_bh < img.num ; mark_bh++)
		{

			ObjBasic obj_ba;

			obj_ba.mark_origin = mark_bh; //点云的原始标注

			if(mark_bh == 0){
				obj_ba.center_x=0;
				obj_ba.center_y=0;
				obj_ba.area=0;
				obj_ba.mark_checked=0;
				obj_basic.push_back(obj_ba);
				continue;
			}

			if (img.stats.at<int>(mark_bh, 4) > area_grid_max_thre)
			 {
				obj_ba.mark_checked = -1;
				mark_to_checked[obj_ba.mark_origin] = obj_ba.mark_checked;
				continue;
			 }
			if (img.stats.at<int>(mark_bh, 4) < area_grid_min_thre)
			 {
				obj_ba.mark_checked = -1;
				mark_to_checked[obj_ba.mark_origin] = obj_ba.mark_checked;
				continue;
			 }
			if (img.stats.at<int>(mark_bh, 2) < 2 || img.stats.at<int>(mark_bh, 3) < 2 )
			 {
				obj_ba.mark_checked = -1;
				mark_to_checked[obj_ba.mark_origin] = obj_ba.mark_checked;
				continue;
			 }
			if (img.stats.at<int>(mark_bh, 2) * img.stats.at<int>(mark_bh, 3) > area_lw_max_thre )
			 {
				obj_ba.mark_checked = -1;
				mark_to_checked[obj_ba.mark_origin] = obj_ba.mark_checked;
				continue;
			 }

			obj_ba.mark_checked = i;


			obj_ba.center_x = img.centroids.at<double>(mark_bh,0) * Resolution - 30; //转换到车体坐标系
			obj_ba.center_y = - img.centroids.at<double>(mark_bh,1) * Resolution + 40;


			obj_ba.area = img.stats.at<int>(mark_bh, 4) * Resolution * Resolution ;


			mark_to_checked[obj_ba.mark_origin] = obj_ba.mark_checked; //从label为1开始。地面为0，不符合要求的为-1
			//存储新旧mark的转换；


//			obj_ba.obj_ids = classify_cloud( mark_bh , Grid );
//			obj_ba.cloud_color = classify_color_cloud(mark_bh, Grid, rand_color);


			obj_basic.push_back(obj_ba); //vector类本身从0开始，mark从1开始

			i++;
		}

		std::vector<int> others_id;
		others_id = classify_cloud(Grid , obj_basic, mark_to_checked);


		pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
		ground_cloud = Ids_to_cloud(others_id, cloud);
		ground_cloud_rgb = set_color_cloud(ground_cloud, "green");



		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_marked_joint(new pcl::PointCloud<pcl::PointXYZRGB>);

//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mark(new pcl::PointCloud<pcl::PointXYZ>);
		for(int mark = 0; mark < obj_basic.size() ; mark++)
		{
//			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_marked(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_marked(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_marked_color(new pcl::PointCloud<pcl::PointXYZRGB>);

			cloud_marked = Ids_to_cloud( obj_basic[mark].obj_ids, cloud);
			if(mark == 0) 	cloud_marked_color = set_color_cloud(cloud_marked, "white");
//			cloud_marked_color = set_color_cloud(cloud_marked, color[u(e)]);
			else
				cloud_marked_color = set_color_cloud(cloud_marked, "red");
			*cloud_marked_joint += *cloud_marked_color;
		}
		*cloud_marked_joint += *ground_cloud_rgb;
//		cloud_marked_joint = Ids_to_cloud(all_id, cloud);
//		save_mark_origin_cloud(cloud_marked_joint, "joint");
		return cloud_marked_joint;


}


