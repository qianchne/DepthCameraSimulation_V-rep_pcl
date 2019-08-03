//�ο����ӣ�https://github.com/qianchne/slambook/tree/master/ch13/dense_RGBD
//�ο����ӣ�https://blog.csdn.net/u014801811/article/details/79711773
#pragma comment(lib, "User32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma warning(disable: 4996)
#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

int show()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\Q-chen\\Desktop\\SLAM_C\\SLAM\\map6.pcd", *cloud);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\Q-chen\\Desktop\\SLAM_C\\SLAM\\map6.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	//std::cout << cloud->width << std::endl;
	//std::cout << cloud->height;

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}
	system("pause");
	return (0);
}

int main(int argc, char** argv)
{
	//show();
	//system("pause");
	vector<cv::Mat> colorImgs, depthImgs;    // ��ɫͼ�����ͼ
	vector<Eigen::Isometry3d> poses;         // ���λ��

	ifstream fin("C:\\Users\\Q-chen\\Desktop\\SLAM_C\\data\\pose6.txt");
	if (!fin)
	{
		cerr << "cannot find pose file" << endl;
		return 1;
	}
		
	for (int i = 0; i < 3; i++)
	{
		boost::format fmt("C:\\Users\\Q-chen\\Desktop\\SLAM_C\\data\\%s\\%d.%s"); //ͼ���ļ���ʽ
		colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
		depthImgs.push_back(cv::imread((fmt % "depth6" % (i + 1) % "pgm").str(), -1)); // ʹ��-1��ȡԭʼͼ��

		double data[7] = { 0 };
		for (int i = 0; i < 7; i++)
		{
			fin >> data[i];
		}
		Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
		Eigen::Isometry3d T(q);
		T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
		poses.push_back(T);
	}

	// ������Ʋ�ƴ��
	// ����ڲ� 
	double cx = 336.0;
	double cy = 276.0;
	double fx = 632.0;
	double fy = 650.3;
	double depthScale = 1000.0;

	cout << "���ڽ�ͼ��ת��Ϊ����..." << endl;

	// �������ʹ�õĸ�ʽ�������õ���XYZ
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloud;

	// �½�һ������
	PointCloud::Ptr pointCloud(new PointCloud);
	for (int i = 0; i < 3; i++)
	{
		PointCloud::Ptr current(new PointCloud);
		cout << "ת��ͼ����: " << i + 1 << endl;
		cv::Mat color = colorImgs[i];
		cv::Mat depth = depthImgs[i];
		Eigen::Isometry3d T = poses[i];
		for (int v = 0; v < color.rows; v++)
			for (int u = 0; u < color.cols; u++)
			{
				unsigned int d = depth.ptr<unsigned short>(v)[u]; // ���ֵ
				//if (d == 0) continue; // Ϊ0��ʾû�в�����
				//if (d >= 7000) continue; // ���̫��ʱ���ȶ���ȥ��
				Eigen::Vector3d point;
				point[2] = double(d) / depthScale;
				point[0] = (u - cx) * point[2] / fx;
				point[1] = (v - cy) * point[2] / fy;
				Eigen::Vector3d pointWorld = T * point;

				PointT p;
				p.x = pointWorld[0];
				p.y = pointWorld[1];
				p.z = pointWorld[2];
				//p.b = color.data[v * color.step + u * color.channels()];
				//p.g = color.data[v * color.step + u * color.channels() + 1];
				//p.r = color.data[v * color.step + u * color.channels() + 2];
				current->points.push_back(p);
			}
		// depth filter and statistical removal 
		PointCloud::Ptr tmp(new PointCloud);
		pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
		statistical_filter.setMeanK(50);
		statistical_filter.setStddevMulThresh(1.0);
		statistical_filter.setInputCloud(current);
		statistical_filter.filter(*tmp);
		(*pointCloud) += *tmp;
	}

	pointCloud->is_dense = false;
	cout << "���ƹ���" << pointCloud->size() << "����." << endl;

	// voxel filter 
	pcl::VoxelGrid<PointT> voxel_filter;
	voxel_filter.setLeafSize(0.02, 0.02, 0.02);       // resolution 
	PointCloud::Ptr tmp(new PointCloud);
	voxel_filter.setInputCloud(pointCloud);
	voxel_filter.filter(*tmp);
	tmp->swap(*pointCloud);

	cout << "�˲�֮�󣬵��ƹ���" << pointCloud->size() << "����." << endl;

	pcl::io::savePCDFileBinary("map6.pcd", *pointCloud);

	// ��ʾ����
	show();
	system("pause");
	return 0;
}