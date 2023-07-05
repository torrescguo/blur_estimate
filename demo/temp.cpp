#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

void showPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud)
{
	pcl::visualization::PCLVisualizer visualizer("showcloud");
	visualizer.addPointCloud(pointcloud);
	visualizer.spin();
}

void showPointCloud(const std::vector<cv::Vec3f> &cloudpos, const std::vector<cv::Vec3b> &cloudcol)
{
	
	cv::viz::Viz3d window("showcloud");
	cv::viz::WCloud cloud_widget(cloudpos, cloudcol);
	window.showWidget("pointcloud", cloud_widget);
	window.spin();
}

int main()
{
	double f = 718.856, cx = 607.1928, cy = 185.2157; //相机内参
	double b = 0.573; //基线长度

	cv::Mat left_img = cv::imread("left.png");
	cv::Mat right_img = cv::imread("right.png");
	
	//下面是书上说的来自网络上的神奇参数
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32); 
	cv::Mat disparity_sgbm, disparity;
	sgbm->compute(left_img, right_img, disparity_sgbm); //计算两帧图像的视差
	disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0);

	/*std::vector<cv::Vec3f> pointcloud;
	std::vector<cv::Vec3b> pointcolor;
	for (int v = 0; v < left_img.rows; v++)
	{
		for (int u = 0; u < left_img.cols; u++)
		{
			if (disparity.at<float>(v, u) <= 10 || disparity.at<float>(v, u) >= 96) continue;
			cv::Vec3f pos;
			cv::Vec3b col;

			double x = (u - cx) / f;
			double y = (v - cy) / f;
			double depth = f * b / (disparity.at<float>(v, u));
			pos[0] = x * depth;
			pos[1] = y * depth;
			pos[2] = depth;
			col = left_img.at<cv::Vec3b>(v, u) / 255;
			pointcloud.emplace_back(pos);
			pointcolor.emplace_back(col);
		}
	}
	showPointCloud(pointcloud, pointcolor);*/

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int v = 0; v < left_img.rows; v++)
	{
		for (int u = 0; u < right_img.cols; u++)
		{
			if (disparity.at<float>(v, u) <= 10 || disparity.at<float>(v, u) >= 96) continue;
			pcl::PointXYZRGB point;
			double x = (u - cx) / f;
			double y = (v - cy) / f;
			double depth = f * b / (disparity.at<float>(v, u));
			point.x = x * depth;
			point.y = y * depth;
			point.z = depth;
			point.b = left_img.at<cv::Vec3b>(v, u)[0];
			point.g = left_img.at<cv::Vec3b>(v, u)[1];
			point.r = left_img.at<cv::Vec3b>(v, u)[2];
			pointcloud->push_back(point);
		}
	}
	showPointCloud(pointcloud);
	
	cv::imshow("disparity", disparity / 96);
	cv::waitKey(0);
}

