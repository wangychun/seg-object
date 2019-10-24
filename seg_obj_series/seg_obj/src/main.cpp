#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <math.h>

#include "gridmap.h"
#include <vector>
#include <array>

#include <string>
#include <ctime>
#include <ratio>
#include <chrono>

#include "pic_handle.h"


using namespace std;

array<array<Cell, numY>, numX> Grid ;

//PointCloud<PointXYZRGB> classify_color_cloud(int& mark, array<array<Cell, numY>, numX>& Grid ,string& color);

void Seg_obj(string in_file)
{

    pcl::PointCloud<pcl::PointXYZ> cloud;

	if(pcl::io::loadPCDFile<pcl::PointXYZ>(in_file, cloud)==-1)
	{
	PCL_ERROR("Couldn't read file test_pcd.pcd\n");
	}


	std::cout<<"Loaded "
	<<cloud.width*cloud.height
	<<" data points from test_pcd.pcd with the following fields: "
	<<std::endl;

  	getAllGrid(cloud, Grid);

  	classfiyAndSave(Grid);
//	cv::Mat image = pic_handle(Grid);

//  	int mark0 = 0;
//  	pcl::PointCloud<pcl::PointXYZRGB> cloud_0;
//  	cloud_0 = classify_color_cloud(mark0, Grid ,"green");
//
//  	int mark1 = 1;
//  	pcl::PointCloud<pcl::PointXYZRGB> cloud_1;
//  	cloud_1 = classify_color_cloud(mark1, Grid ,"pink");
//
//  	pcl::PointCloud<pcl::PointXYZRGB> cloud_joint = cloud_0 + cloud_1;
//  	save_mark_color_cloud(cloud_joint,"joint");
//  	pcl::PointCloud<pcl::PointXYZ> cloud_0;
//  	cloud_0 = classify_cloud(mark0,Grid);
//  	save_mark_origin_cloud(cloud_0,"0");

}

static std::vector<std::string> file_lists;

//Read the file lists
void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

bool computePairNum(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}

void sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;

    std::sort(filists.begin(),filists.end(),computePairNum);
}



int main(int argc, char **argv)
{
	std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now(); //返回时间戳

    std::string pcd_path = "/home/sarah/program/mine/seg_obj/data/";

    read_filelists( pcd_path, file_lists, "pcd" );
    sort_filelists( file_lists, "pcd" );


	for (int i = 0; i < 1; ++i)
    {

      std::string pcd_file = pcd_path + file_lists[i];
      Seg_obj(pcd_file);
    }


	std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

	std::chrono::duration<double, std::milli> time_span = t2 - t1;

    std::cout << time_span.count()  << std::endl;

    return 0;
}
