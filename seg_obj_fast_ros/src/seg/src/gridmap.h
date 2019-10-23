/*
 * ground_remove.h
 *
 *  Created on: Sep 22, 2019
 *      Author: sarah
 */

#ifndef GRIDMAP_H_
#define GRIDMAP_H_

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/impl/io.hpp>
#include <vector>
#include <array>

#include <opencv2/core.hpp>

//#include "pic_handle.h"

using namespace std;
using namespace pcl;

const double Resolution = 0.2;
const int numX = 300;//分辨率0.2, 60*80
const int numY = 400;
const float GroundThre = 0.05;
const int NumThre = 0;
const float Ground_H = 1.0;
const int Empty_thre = 3; //点云数量小于3 认为是空
//const float GroundZ;
class ObjBasic{
public:
	double center_x;
	double center_y;
	double area;
	int mark_origin;
	int mark_checked;
	std::vector<int> obj_ids;
};

class Cell{
public:
    float maxZ;
    float minZ;
    int num;
    float heightdiff;
    int mark;
//    PointCloud<PointXYZ> cloud;//wyc
    std::vector<int> indexs;
//    void updatecloud(PointXYZ point){cloud.push_back(point);}
    void getheightdiff(){if(maxZ < minZ) heightdiff = -1; else heightdiff = maxZ - minZ;}
//    void getheightdiff(){heightdiff = maxZ - minZ;}
    void init_cell(){ maxZ = -100; minZ = 100; mark = -1; num = 0;indexs = {};heightdiff = 0;}
};

void filterCloud(PointCloud<PointXYZ>::Ptr &cloud, PointCloud<PointXYZ>::Ptr & filteredCloud);

void createAndMapGrid(PointCloud<PointXYZ>:: Ptr &cloud,
                           array<array<Cell, numY>, numX>& GridData );

//std::vector<ObjBasic> classify_cloud(array<array<Cell, numY>, numX>& Grid ,
//												std::vector<ObjBasic> obj_basic,
//												std::vector<int> mark_to_checked);

std::vector<int> classify_cloud(array<array<Cell, numY>, numX>& Grid ,
												std::vector<ObjBasic> &obj_basic,
												std::vector<int> mark_to_checked);


PointCloud<PointXYZ>::Ptr Ids_to_cloud(std::vector<int> &indexs, PointCloud<PointXYZ>::Ptr &cloud);

void save_mark_origin_cloud( PointCloud<PointXYZ>::Ptr &cloud, string name);

void save_mark_color_cloud(PointCloud<PointXYZ>::Ptr cloud, string name, string color);
void save_mark_color_cloud(PointCloud<PointXYZRGB>::Ptr cloud, string name);

PointCloud<PointXYZRGB>::Ptr set_color_cloud(PointCloud<PointXYZ>::Ptr & cloud, string color);
void getAllGrid(PointCloud<PointXYZ>::Ptr &cloud,
		array<array<Cell, numY>, numX>& Grid );

#endif /* GRIDMAP_H_ */
