#include <iostream>
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
//ROS
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "gridmap.h"
#include "pic_handle.h"
#include <pcl/visualization/pcl_visualizer.h>

//#include "get_data.h"


//int main(int argc, char **argv)
//{
//	  std::string pcd_path = "../data/";
//    read_filelists( pcd_path, file_lists, "pcd" );
//    sort_filelists( file_lists, "pcd" );

//	for (int i = 0; i < 1; ++i)
//    {
//      std::string pcd_file = pcd_path + file_lists[i];
//      Seg_obj(pcd_file);
//    }
//   return 0;
//}
//
std::array<std::array<Cell, numY>, numX> Grid;

int i = 0;
class SubscribeAndPublish {
public:
	SubscribeAndPublish(ros::NodeHandle nh, std::string lidar_topic_name);

	void callback(const sensor_msgs::PointCloud2ConstPtr& cloudmsg) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_mark(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(*cloudmsg, *cloud);

		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

		getAllGrid(cloud, Grid);
		cloud_mark = classfiyAndSave(Grid, cloud);
//
//		std::cout<<cloud_mark->size()<<std::endl;
		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> time_span1 = t2 - t1;
	    std::cout << "total time:" << time_span1.count()  << std::endl;

		sensor_msgs::PointCloud2 ros_cloud;
		pcl::toROSMsg(*cloud_mark, ros_cloud);
		ros_cloud.header.frame_id = "global_init_frame";
		pub_.publish(ros_cloud);



	}
private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> Sub_Lidar;


};

SubscribeAndPublish::SubscribeAndPublish(ros::NodeHandle nh,
		std::string lidar_topic_name)
{
	//Topic you want to publish
	pub_ = nh.advertise < sensor_msgs::PointCloud2 > ("/seg_obj", 10);
	sub_ = n_.subscribe("lidar_cloud_calibrated", 10, &SubscribeAndPublish::callback, this);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "seg_obj");
	SubscribeAndPublish SAPObject(ros::NodeHandle(), "lidar_cloud_calibrated");
	ROS_INFO("waiting for data!");
	ros::spin();

    return 0;
}
