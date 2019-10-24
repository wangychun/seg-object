#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include "gridmap.h"
#include <vector>
#include <array>

#include <string>

using namespace std;
array < array <class Cell, numY >, numX > Grid ;

int main(int argc, char **argv)
{
	cout<<"1211"<<endl;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if(pcl::io::loadPCDFile("/home/sarah/program/mine/seg_obj/data/0000000001.pcd", cloud)==-1)
		{
			PCL_ERROR("Couldn't read file test_pcd.pcd\n");
			return 0;
		}
	std::cout<<"Loaded "
	<<cloud.width*cloud.height
	<<" data points from test_pcd.pcd with the following fields: "
	<<std::endl;

  	getAllGrid(cloud, Grid);
	cout<<"3"<<endl;
  	int mark = -1;
	cout<<"4"<<endl;
  	//save_grid_cloud(mark, Grid);

    return 0;
}
