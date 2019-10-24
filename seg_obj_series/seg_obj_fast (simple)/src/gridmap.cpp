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

void filterCloud(PointCloud<PointXYZ>::Ptr &cloud, PointCloud<PointXYZ>::Ptr & filteredCloud){

    for (int i = 0; i < cloud->size(); i++) {

    	float x = cloud->points[i].x + 30;
        float y = -cloud->points[i].y + 40;
        float z = cloud->points[i].z;

            pcl::PointXYZ o;
            o.x = x;
            o.y = y;
            o.z = z;
            filteredCloud->push_back(o);
    }
}
void createAndMapGrid(PointCloud<PointXYZ>:: Ptr &cloud,
                           array<array<Cell, numY>, numX>& GridData ){
    for (int i = 0; i < cloud->size(); i++) {

        if( cloud->points[i].x < 0 || cloud->points[i].y < 0)  continue;
        if( cloud->points[i].x > 60 || cloud->points[i].y > 80)  continue;
        if(abs(cloud->points[i].x - 30) < 1.5 && (-cloud->points[i].y+40) < 4 && (cloud->points[i].y + 40) > -0.5 )  continue;
        if(cloud->points[i].z > 2.5) continue;

    	float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;

        int x_id, y_id;
    	x_id = int(x / Resolution);
    	y_id = int(y / Resolution);

        if(x_id < 0 || x_id >=numX || y_id < 0 || y_id >= numY) continue; // to prevent segentation fault
        if (z < GridData[x_id][y_id].minZ) GridData[x_id][y_id].minZ = z;
        if (z > GridData[x_id][y_id].maxZ) GridData[x_id][y_id].maxZ = z;
        GridData[x_id][y_id].indexs.push_back(i);
    }

}


void classify_cloud(array<array<Cell, numY>, numX>& Grid ,
												std::vector<ObjBasic> &obj_basic,
												std::vector<int> mark_to_checked)
{

	for(int x_id = 0; x_id<numX; x_id++)
		for (int y_id=0; y_id<numY; y_id++)
			{
				int old_label = Grid[x_id][y_id].mark;
				Grid[x_id][y_id].mark = mark_to_checked[old_label];//mark为0 和 1 的， 现在都是0了。
				int new_index = Grid[x_id][y_id].mark - 2;
				if (Grid[x_id][y_id].mark == 0) continue;

				if (Grid[x_id][y_id].mark == 1) cout << "error" <<endl;

				obj_basic[new_index].obj_ids.insert(obj_basic[new_index].obj_ids.end(),
													Grid[x_id][y_id].indexs.begin(),
													Grid[x_id][y_id].indexs.end());
			}
}

PointCloud<PointXYZ>::Ptr Ids_to_cloud(std::vector<int> &indexs, PointCloud<PointXYZ>::Ptr &cloud){

	PointCloud<PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, indexs, *cloudOut);
	return cloudOut;
}

void save_mark_origin_cloud(PointCloud<PointXYZ>::Ptr &cloud, string name)
{
	string filename = "mark_ori_"+ name +".pcd";
	pcl::io::savePCDFileASCII(filename, *cloud);
}

void save_mark_color_cloud(PointCloud<PointXYZ>::Ptr cloud, string name, string color)
{
	PointCloud<PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	string filename = "mark_color_"+ name +".pcd";
	color_cloud = set_color_cloud(cloud, color);
	pcl::io::savePCDFileASCII(filename, *color_cloud);
}

void save_mark_color_cloud(PointCloud<PointXYZRGB>::Ptr cloud, string name)
{
	string filename = "mark_color_"+ name +".pcd";
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

	PointCloud<PointXYZ>::Ptr  filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	filterCloud(cloud, filteredCloud);
	createAndMapGrid(filteredCloud, Grid);
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

//			nnum ++;
			if(Grid[x_id][y_id].num < Empty_thre) continue;

			if (Grid[x_id][y_id].heightdiff >= 0 )
			{
				if(Grid[x_id][y_id].minZ < Ground_H && Grid[x_id][y_id].heightdiff < GroundThre)
					{
//					c_nnum ++;
					Grid[x_id][y_id].mark = 0;

					}
				else
					Grid[x_id][y_id].mark = 1;
			}
			else if(Grid[x_id][y_id].heightdiff == -1)
			{
//				hd++;
				Grid[x_id][y_id].mark = 0;
				cout << "something wrong in heightdiff!" << endl;
			}
			else ;//后期加一下报错

		}
}


