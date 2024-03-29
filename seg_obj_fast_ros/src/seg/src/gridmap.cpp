#include "gridmap.h"
#include "pic_handle.h"
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <array>
#include <string>

using namespace std;
using namespace Eigen;
using namespace pcl;
extern int i;

void createAndMapGrid(PointCloud<PointXYZ>:: Ptr &cloud,
                           array<array<Cell, numY>, numX>& GridData ){

	int low =0, middle =0, high = 0;
    for (int i = 0; i < cloud->size(); i++) {

    	//针对kitti数据集的修改
    	float x = -cloud->points[i].y + 20;
    	float y = -cloud->points[i].x + 50;
    	float z = cloud->points[i].z + 1.73;

    	cloud->points[i].x = x;
    	cloud->points[i].y = y;
    	cloud->points[i].z = z;

//看一下数据
//    	if (cloud->points[i].z < -0.5) low++;
//    	else  if (cloud->points[i].z > 0.5) high++;
//    	else middle++;

        if( cloud->points[i].x < 0 || cloud->points[i].y < 0)  continue;
        if( cloud->points[i].x > 40 || cloud->points[i].y > 100)  continue;
//        if(poin )  continue;
        if(cloud->points[i].z > 2.5) continue;

        int x_id, y_id;
    	x_id = int(cloud->points[i].x / Resolution);
    	y_id = int(cloud->points[i].y / Resolution);


        if(x_id < 0 || x_id >=numX || y_id < 0 || y_id >= numY) {continue;} // to prevent segentation fault
        if (cloud->points[i].z < GridData[x_id][y_id].minZ) {GridData[x_id][y_id].minZ = cloud->points[i].z;}
        if (cloud->points[i].z > GridData[x_id][y_id].maxZ) GridData[x_id][y_id].maxZ = cloud->points[i].z;
        GridData[x_id][y_id].indexs.push_back(i);
    }

//    std::cout << "low:" << low <<std::endl;
//    std::cout << "middle:" << middle <<std::endl;
//    std::cout << "high:" << high <<std::endl;
}

std::vector<int> classify_cloud(array<array<Cell, numY>, numX>& Grid ,
												std::vector<ObjBasic> &obj_basic,
												std::vector<int> mark_to_checked)
{
//	obj_basic(obj_basic.size());
//	cout << "size:" << obj_basic.size() << endl;
	std::vector<int> others_i;
	for(int x_id = 0; x_id<numX; x_id++)
		for (int y_id=0; y_id<numY; y_id++)
//			for (int label_ori = 0; label_ori < mark_to_checked.size(); label_ori++ )
			{
				int old_label = Grid[x_id][y_id].mark;
				Grid[x_id][y_id].mark = mark_to_checked[old_label];//mark为-1,0,1,2,...
				int new_index = Grid[x_id][y_id].mark;
				if (new_index == -1)
					{
					others_i.insert(others_i.end(),Grid[x_id][y_id].indexs.begin(),
																		Grid[x_id][y_id].indexs.end());
					continue;
					}


				else obj_basic[new_index].obj_ids.insert(obj_basic[new_index].obj_ids.end(),
													Grid[x_id][y_id].indexs.begin(),
													Grid[x_id][y_id].indexs.end());
//				obj_all_i.insert(obj_all_i.end(),Grid[x_id][y_id].indexs.begin(),
//						Grid[x_id][y_id].indexs.end());
//				cout << obj_basic[new_index].obj_ids.size() << endl;
			}

	return others_i;
}

PointCloud<PointXYZ>::Ptr Ids_to_cloud(std::vector<int> &indexs, PointCloud<PointXYZ>::Ptr &cloud){

	PointCloud<PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, indexs, *cloudOut);
	return cloudOut;
}

//void save_mark_origin_cloud(std::vector<int> &indexs, PointCloud<PointXYZ>::Ptr &cloud, string name)
void save_mark_origin_cloud(PointCloud<PointXYZ>::Ptr &cloud, string name)
{
	i++;
//	string filename = "mark_ori_"+ name +".pcd";
	string filename = "mark_ori_"+ name + std::to_string(i) +".pcd";
//	PointCloud<PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
//	cloudOut = Ids_to_cloud(indexs, cloud);  //减少耦合，存点云就只存点云
	pcl::io::savePCDFileASCII(filename, *cloud);
}

//void save_mark_color_cloud(std::vector<int> &indexs,PointCloud<PointXYZ>::Ptr cloud, string name, string color)
void save_mark_color_cloud(PointCloud<PointXYZ>::Ptr cloud, string name, string color)
{
	PointCloud<PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	string filename = "mark_color_"+ name +".pcd";
//	PointCloud<PointXYZ>::Ptr cloudOut;
//	cloudOut = Ids_to_cloud(indexs, cloud);
	color_cloud = set_color_cloud(cloud, color);
	pcl::io::savePCDFileASCII(filename, *color_cloud);
}

void save_mark_color_cloud(PointCloud<PointXYZRGB>::Ptr cloud, string name)
{
	i++;
	string filename = "mark_color_"+ name +std::to_string(i) +".pcd";
//	PointCloud<PointXYZ>::Ptr cloudOut;
//	cloudOut = Ids_to_cloud(indexs, cloud);
	pcl::io::savePCDFileASCII(filename, *cloud);
}

PointCloud<PointXYZRGB>::Ptr set_color_cloud(PointCloud<PointXYZ>::Ptr & cloud, string color)
{
	PointCloud<PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	PointXYZRGB color_point;
	for (int i = 0; i < cloud->size(); i++)
	{
		color_point.x = cloud->points[i].x;
		color_point.y = cloud->points[i].y;
		color_point.z = cloud->points[i].z;

		if (color == "black")//黑
			{
				color_point.r = 0;
				color_point.g = 0;
				color_point.b = 0;
			}


			if (color == "yellow")//黄
			{
				color_point.r = 255;
				color_point.g = 255;
				color_point.b = 0;
			}
			if (color == "red")//红
			{
				color_point.r = 255;
				color_point.g = 0;
				color_point.b = 0;
			}
			if (color == "brown")//棕色
			{
				color_point.r = 205;
				color_point.g = 133;
				color_point.b = 63;
			}


			if (color == "green")//绿
			{
				color_point.r = 0;
				color_point.g = 255;
				color_point.b = 0;
			}
			if (color == "violet")//紫
			{
				color_point.r = 128;
				color_point.g = 0;
				color_point.b = 128;
			}
			if (color == "pink")//粉（品红）
			{
				color_point.r = 255;
				color_point.g = 0;
				color_point.b = 255;
			}


			if (color == "azure")//天蓝
			{
				color_point.r = 135;
				color_point.g = 206;
				color_point.b = 250;
			}
			if (color == "blue")//蓝
			{
				color_point.r = 0;
				color_point.g = 0;
				color_point.b = 255;
			}

			if (color =="orange")//橙
			{
				color_point.r = 255;
				color_point.g = 165;
				color_point.b = 0;
			}
			if (color =="white")//白
			{
				color_point.r = 255;
				color_point.g = 255;
				color_point.b = 255;
			}
		color_cloud->push_back(color_point);
	}

	return color_cloud;

}

void getAllGrid(PointCloud<PointXYZ>::Ptr &cloud,
		array<array<Cell, numY>, numX>& Grid ){
	//初始化一个空Grid
	for(int x_id = 0; x_id<numX; x_id++)
		for (int y_id=0; y_id<numY; y_id++)
			Grid[x_id][y_id].init_cell();

	createAndMapGrid(cloud, Grid);
//	int hd = 0;
//	int nnum = 0;
//	int c_nnum = 0;
	//处理完栅格，获得每个栅格的高度属性，mark暂时还是只有0，-1
	//mark = 0:点云量少，或地面点云
	//mark = 1; 障碍物点云
	for(int x_id = 0; x_id<numX; x_id++)
		for (int y_id=0; y_id<numY; y_id++)
		{
			Grid[x_id][y_id].getheightdiff();
			Grid[x_id][y_id].num = Grid[x_id][y_id].indexs.size();

			if(Grid[x_id][y_id].num < Empty_thre)
			{
//				Grid[x_id][y_id].mark = 1;
				continue;
			}

//			cout<<Grid[x_id][y_id].heightdiff<<endl;

			if (Grid[x_id][y_id].heightdiff >= 0 )
			{
				if(Grid[x_id][y_id].maxZ < Ground_H && Grid[x_id][y_id].heightdiff < GroundThre)
					{
//					cout <<"someting right!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<x_id<<","<<y_id<<endl;
					Grid[x_id][y_id].mark = 0;

					}
				else
					Grid[x_id][y_id].mark = 1;
			}
			else if(Grid[x_id][y_id].heightdiff == -1)
			{

				Grid[x_id][y_id].mark = 0;
				cout << "something wrong in heightdiff!" << Grid[x_id][y_id].heightdiff << ":"<< Grid[x_id][y_id].minZ << endl;
			}
			else ;//后期加一下报错

		}
}


