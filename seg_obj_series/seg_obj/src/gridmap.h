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

class Cell{
public:
    float maxZ;
    float minZ;
    int num;
    float heightdiff;
    int mark;
    PointCloud<PointXYZ> cloud;//wyc
    void updatecloud(PointXYZ point){cloud.push_back(point);}
    void getheightdiff(){if(maxZ < minZ) heightdiff = -1; else heightdiff = maxZ - minZ;}
    void init_cell(){ maxZ = -100; minZ = 100; mark = -1; num = 0;}
};

void filterCloud(PointCloud<PointXYZ> cloud, PointCloud<PointXYZ> & filteredCloud);
void createAndMapGrid(PointCloud<PointXYZ> cloud,
                           array<array<Cell, numY>, numX>& GridData );
void getAllGrid(PointCloud<PointXYZ> cloud,
		array<array<Cell, numY>, numX>& Grid );

PointCloud<PointXYZRGB> set_color_cloud(PointCloud<PointXYZ>& cloud, string color);

PointCloud<PointXYZ> classify_cloud(int& mark, array<array<Cell, numY>, numX>& Grid );
PointCloud<PointXYZRGB> classify_color_cloud(int& mark, array<array<Cell, numY>, numX>& Grid ,string color);
void save_mark_origin_cloud(PointCloud<PointXYZ> uncolor_cloud, string name);
void save_mark_color_cloud(PointCloud<PointXYZRGB> color_cloud, string name);


#endif /* GRIDMAP_H_ */
