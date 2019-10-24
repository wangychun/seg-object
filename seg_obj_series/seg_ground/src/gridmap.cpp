#include "gridmap.h"

#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <array>
#include <string>

using namespace std;
using namespace Eigen;
using namespace pcl;


void filterCloud(PointCloud<PointXYZ> cloud, PointCloud<PointXYZ> & filteredCloud){
    for (int i = 0; i < cloud.size(); i++) {
      if(abs(cloud.points[i].x) > 40 || abs(cloud.points[i].y) > 30)  continue;
      if(abs(cloud.points[i].x) < 1.5 && cloud.points[i].y < 4 && cloud.points[i].y > -0.5 )  continue;

    	float x = cloud.points[i].x + 30;
        float y = -cloud.points[i].y + 40;
        float z = cloud.points[i].z;

            pcl::PointXYZ o;
            o.x = x;
            o.y = y;
            o.z = z;
            filteredCloud.push_back(o);
    }
    cout << "filter:"<<cloud.size()<<endl;
}


void createAndMapGrid(PointCloud<PointXYZ> cloud,
                           array<array<Cell, numY>, numX>& GridData ){
    for (int i = 0; i < cloud.size(); i++) {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;
        float z = cloud.points[i].z;
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;

        int x_id, y_id;
    	x_id = int(x/0.2);
    	y_id = int(y/0.2);
        // TODO; modify abobe function so that below code would not need
        if(x_id < 0 || x_id >=numX || y_id < 0 || y_id >= numY) continue; // to prevent segentation fault
        if (z < GridData[x_id][y_id].minZ) GridData[x_id][y_id].minZ = z;
        if (z > GridData[x_id][y_id].maxZ) GridData[x_id][y_id].maxZ = z;
    	GridData[x_id][y_id].updatecloud(point);
    }

	cout<<"create:"<<cloud.size()<<endl;
}
/////////////////////////////////////////////////////////////////////
//classify by mark
PointCloud<PointXYZ> classify_cloud(int& mark, array<array<Cell, numY>, numX>& Grid )
{
	PointCloud<PointXYZ> marksamecloud;
		for(int x_id = 0; x_id<numX; x_id++)
			for (int y_id=0; y_id<numY; y_id++)
			{
				if(Grid[x_id][y_id].mark == mark)
				{
					marksamecloud += Grid[x_id][y_id].cloud;
				}
			}
		return marksamecloud;
}

PointCloud<PointXYZRGB> classify_color_cloud(int& mark, array<array<Cell, numY>, numX>& Grid ,string color)
{
	PointCloud<PointXYZ> cloud;
	PointCloud<PointXYZRGB> markcolorcloud;
		for(int x_id = 0; x_id<numX; x_id++)
			for (int y_id=0; y_id<numY; y_id++)
			{
				if(Grid[x_id][y_id].mark == mark)
				{
					cloud += Grid[x_id][y_id].cloud;
				}
			}
		 markcolorcloud = set_color_cloud(cloud, color);
		return markcolorcloud;
}
//save grid cloud
void save_mark_origin_cloud(PointCloud<PointXYZ> uncolor_cloud, string name )
{
	string filename = "mark_ori_"+ name +".pcd";
	pcl::io::savePCDFileASCII(filename, uncolor_cloud);
}

void save_mark_color_cloud(PointCloud<PointXYZRGB> color_cloud, string name)
{
	string filename = "mark_color_"+ name +".pcd";
	pcl::io::savePCDFileASCII(filename, color_cloud);
}

PointCloud<PointXYZRGB> set_color_cloud(PointCloud<PointXYZ>& cloud, string color)
{
	PointCloud<PointXYZRGB> color_cloud;
	PointXYZRGB color_point;
	for (int i = 0; i < cloud.size(); i++)
	{
		color_point.x = cloud[i].x;
		color_point.y = cloud[i].y;
		color_point.z = cloud[i].z;

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
		color_cloud.push_back(color_point);
	}

	return color_cloud;

}

void getAllGrid(PointCloud<PointXYZ> cloud,
		array<array<Cell, numY>, numX>& Grid ){
	//初始化一个空Grid
	for(int x_id = 0; x_id<numX; x_id++)
		for (int y_id=0; y_id<numY; y_id++)
	Grid[x_id][y_id].init_cell();

	PointCloud<PointXYZ>  filteredCloud;
	filterCloud(cloud, filteredCloud);
	createAndMapGrid(filteredCloud, Grid);
	int hd = 0;
	int nnum = 0;
	int c_nnum = 0;
	//处理完栅格，获得每个栅格的高度属性，mark暂时还是只有0，-1
	//mark = 0:点云量少，或地面点云
	//mark = 1; 障碍物点云
	for(int x_id = 0; x_id<numX; x_id++)
		for (int y_id=0; y_id<numY; y_id++)
		{
			Grid[x_id][y_id].getheightdiff();
			Grid[x_id][y_id].num = Grid[x_id][y_id].cloud.size();

			nnum ++;
			if(Grid[x_id][y_id].num < Empty_thre) continue;


//			if (Grid[x_id][y_id].heightdiff > 0 || Grid[x_id][y_id].heightdiff == 0)
			if (Grid[x_id][y_id].heightdiff >= 0 )
			{
				if(Grid[x_id][y_id].minZ < Ground_H && Grid[x_id][y_id].heightdiff < GroundThre)
//				if(Grid[x_id][y_id].heightdiff < GroundThre)
					{
					c_nnum ++;
					//cout<<"minZ<0:" <<Grid[x_id][y_id].minZ << endl;
					Grid[x_id][y_id].mark = 0;

					}
				else
					Grid[x_id][y_id].mark = 1;
			}
			else if(Grid[x_id][y_id].heightdiff == -1)
			{
				cout<<"hd=-1:"<<Grid[x_id][y_id].minZ << "  x_id:" << y_id<< "  y_id:" << y_id << endl;
				hd++;
				Grid[x_id][y_id].mark = 0;
			}
			else ;//后期加一下报错

		}
	cout << "hd:" << hd << endl;
	cout << "nnum:" << nnum << endl;
	cout << "c_nnum:" << c_nnum << endl;
}


