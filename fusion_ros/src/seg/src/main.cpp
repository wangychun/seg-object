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

//ros下和opencv数据转换
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>

std::array<std::array<Cell, numY>, numX> Grid;

int i = 0;
class SubscribeAndPublish {
public:
	SubscribeAndPublish(ros::NodeHandle nh, std::string lidar_topic_name, std::string image_topic_name);

	void callback(const sensor_msgs::PointCloud2ConstPtr& cloudmsg,
			const sensor_msgs::ImageConstPtr &imagemsg)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_mark(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(*cloudmsg, *cloud);

		cv_bridge::CvImagePtr cv_ptr;
		try                 //对错误异常进行捕获，检查数据的有效性
		    {
		      cv_ptr = cv_bridge::toCvCopy(imagemsg, sensor_msgs::image_encodings::BGR8);
		    }
		    catch (cv_bridge::Exception& e)
		    {
		      ROS_ERROR("cv_bridge exception: %s", e.what());
		      return;
		    }

		    cv::Mat image_raw = cv_ptr->image;
//		    cv::imshow("view", cv_ptr->image);
//		    cv::waitKey(1);//用于图片的更新

		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

		getAllGrid(cloud, Grid);
		cloud_mark = classfiyAndSave(Grid, cloud, image_raw);
//
//		std::cout<<cloud_mark->size()<<std::endl;
		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> time_span1 = t2 - t1;
	    std::cout << "total time:" << time_span1.count()  << std::endl;



		sensor_msgs::PointCloud2 ros_cloud;
		pcl::toROSMsg(*cloud_mark, ros_cloud);
//		ros_cloud.header.frame_id = "global_init_frame";

		ros_cloud.header.frame_id = "velo_link";
		pub_lidar_.publish(ros_cloud);



	}
private:
	ros::NodeHandle n_;
	ros::Publisher pub_lidar_;
	//图像发布
//	ros::Publisher pub_image_;
//	ros::Subscriber sub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> Sub_Lidar;
	message_filters::Subscriber<sensor_msgs::Image> Sub_image;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
			sensor_msgs::Image> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync;

};

SubscribeAndPublish::SubscribeAndPublish(ros::NodeHandle nh,
		std::string lidar_topic_name, std::string image_topic_name) :
				n_(nh), Sub_Lidar(nh, lidar_topic_name, 10), Sub_image(nh, image_topic_name,
						20), sync(MySyncPolicy(10), Sub_Lidar, Sub_image)
{
	//Topic you want to publish

	pub_lidar_ = nh.advertise < sensor_msgs::PointCloud2 > ("/seg_obj", 10);
	//先不发布图像
//	pub_image_ = nh.advertise < sensor_msgs::Image> ("/image_show", 10);
//	sub_ = n_.subscribe("lidar_cloud_calibrated", 10, &SubscribeAndPublish::callback, this);

//	sub_ = n_.subscribe("kitti/velo/pointcloud", 10, &SubscribeAndPublish::callback, this);
	sync.registerCallback(
			boost::bind(&SubscribeAndPublish::callback, this, _1, _2));
}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "seg_obj");
	SubscribeAndPublish SAPObject(ros::NodeHandle(), "kitti/velo/pointcloud","kitti/camera_color_left/image_raw");
	ROS_INFO("waiting for data!");
	ros::spin();

    return 0;
}
