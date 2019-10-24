#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
//#include "opencv2/imgcodecs.hpp"

#include<opencv2/opencv.hpp>
#include<algorithm>

int main(int argc, char **argv) {
	cv::Mat image_test;
	
	image_test = cv::imread("images.png",CV_LOAD_IMAGE_GRAYSCALE);//239*211
//	cv::namedWindow("show");
//	cv::imshow("show_ori",image_test);
//	cv::waitKey(10000);//这里是多个wait，wait后，再出下一张
	
	cv::Mat image(239, 211, CV_8U, 255);//最后那个数字就是初始化像素值:255白；不能为0，会报错，如果想初始化为0，如下操作：
//	cv::Mat image = cv::Mat::zeros(239, 211, CV_8U);
//	std::cout<< "channel: "<< image.channels()<<std::endl;
	image = image_test.clone();
//	std::cout<< "channel: "<< image.channels()<<std::endl;

	cv::imshow("show_copy",image);

	cv::Mat image_blur;
	cv::medianBlur(image, image_blur ,3);
//	cv::imshow("show_blur",image_blur);

	cv::Mat element5(5,5,CV_8U,cv::Scalar(1));
//	cv::Mat element3(3,3,CV_8U,cv::Scalar(1));
	cv::Mat image_closed;
	cv::morphologyEx(image_blur, image_closed,cv::MORPH_CLOSE,element5);
	cv::imshow("show_close",image_closed);

//	cv::Mat bin;
	cv::Mat labels;
	cv::Mat stats;
	cv::Mat centroids;
	cv::Mat show;

//	std::cout<< "channel: "<< image_closed.channels()<<std::endl;
//	threshold(image_closed, bin, 100, 255, THRESH_BINARY_INV);
//
	int num = cv::connectedComponentsWithStats(image_closed, labels, stats, centroids);
//
	std::cout << "轮廓数" << num << std::endl;
//	cv::imshow("labels", labels);
//	std::cout << "labels"<< labels << std::endl;
	std::cout << stats << std::endl;
	std::cout << "stats(3,4):" << stats.at<int>(3,4) << std::endl;
	std::cout << centroids << std::endl;
	std::cout << "cent(3):" << centroids.at<double>(3,0) << "*" << centroids.at<double>(3,1) << std::endl;
	std::cout << "cent(2):" << centroids.at<double>(2,0) << "*" << centroids.at<double>(2,1) << std::endl;
//	cv::imshow("stats", stats);
//	分别对应各个轮廓的x,y,width,height和面积。注意0的区域标识的是background
//	https://blog.csdn.net/qq_30815237/article/details/86898686
//
//	int x = stats.at<uchar>(1,0);
//	int y = stats.at<uchar>(1,1);
//	cv::imshow("centroids", centroids);
//	std::cout<<stats<<std::endl;
//	std::cout<<"x:"<< x <<"  y:"<< y << std::endl;
//	std::cout<<stats.channels()<<std::endl;
//	std::cout<<stats.type()<<std::endl;

	cv::Mat_<uchar> img;
	img = stats;
	int unb = img(3,2);
	std::cout<<unb<<std::endl;
	int ss = img(3);
	std::cout<<ss<<std::endl;
//	其中connectedComponents()仅仅创建了一个标记图（图中不同连通域使用不同的标记，和原图宽高一致），connectedComponentsWithStats()可以完成上面任务，除此之外，还可以返回每个连通区域的重要信息--bounding box, area, andcentroid
//	参数说明：
//	该函数有返回值，返回一个int整型 nccomps，函数返回值为连通区域的总数N，范围为[0,N-1]，其中0代表背景。
//	image：输入8位单通道二值图像；
//	label：输出，和原图image一样大的标记图，label对应于表示是当前像素是第几个轮廓，背景置0；
//	centroids：对应的是轮廓的中心点。nccomps×2的矩阵 表示每个连通区域的质心
//	stats：输出，nccomps×5的矩阵 ，表示每个连通区域的外接矩形和面积（pixel）

//	cv::Mat image_closed_b3;
//	cv::morphologyEx(image_blur, image_closed_b3,cv::MORPH_CLOSE,element3);
//	cv::imshow("show_b3",image_closed_b3);
//
//	cv::Mat image_closed_o5;
//	cv::morphologyEx(image, image_closed_o5,cv::MORPH_CLOSE,element5);
//	cv::imshow("show_o5",image_closed_o5);
//
//	cv::Mat image_closed_o3;
//	cv::morphologyEx(image, image_closed_o3,cv::MORPH_CLOSE,element3);
//	cv::imshow("show_o3",image_closed_o3);



	cv::waitKey(0);//wait keyboard;then close pic;
	return 0;
}
