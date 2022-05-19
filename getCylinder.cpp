//这是第一个执行的文件
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

int main4585(int argc, char** argv)
{
	//------------------------------读取点云数据---------------------------------
	//string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\肖智华师兄的EI论文\\分类论文数据汇总\\0初步去噪之后数据\\4Cloud去噪.pcd";
	string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\公路隧道数据-秀山移动\\road_tunnel_4.pcd";
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ >);
	pcl::PCDReader reader;
	reader.read(readPath, *cloud);
	cout << "PointCloud has: " << cloud->points.size() << " data points." << endl;
	//--------------------------------直通滤波-----------------------------------
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ >);
	pcl::PassThrough<pcl::PointXYZ > pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");//将Z轴不在（-1.60, 4.22）范围内的点过滤掉
	pass.setFilterLimits(0.00, 9.22);
	pass.filter(*cloud_filtered);//剩余的点储存在cloud_filtered中后续使用
	cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << endl;
	//--------------------------------计算法线-----------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ >::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ >);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	n.setSearchMethod(tree);
	n.setInputCloud(cloud_filtered);
	n.setKSearch(50);
	n.compute(*normals);
	//------------------------------创建分割对象---------------------------------
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
	// ------------------------点云分割，提取平面上的点--------------------------
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(normals);
	seg.segment(*inliers_plane, *coefficients_plane);//获取平面模型系数和平面上的点
	cout << "Plane coefficients: " << *coefficients_plane << endl;
	//----------------------------------提取平面---------------------------------
	pcl::ExtractIndices<pcl::PointXYZ > extract;
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ >());
	extract.filter(*cloud_plane);
	cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << endl;
	//-----------------------------存储点云到输出文件----------------------------
	string outFile_plane = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\公路隧道数据-秀山移动\\中间处理结果\\4planeResult.pcd";

	pcl::PCDWriter writer;
	writer.write(outFile_plane, *cloud_plane, false);
	//-------------------------------提取圆柱体模型------------------------------
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ >);
	pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
	//获取平面以外的点和点的法线
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(normals);
	extract_normals.setIndices(inliers_plane);
	extract_normals.filter(*normals2);
	//为圆柱体分割创建分割对象，并设置参数
	seg.setOptimizeCoefficients(true);        //设置对估计的模型系数需要进行优化
	seg.setModelType(pcl::SACMODEL_CYLINDER); //设置分割模型为圆柱型
	seg.setMethodType(pcl::SAC_RANSAC);       //设置采用RANSAC作为算法的参数估计方法
	seg.setNormalDistanceWeight(0.2);         //设置表面法线权重系数
	seg.setMaxIterations(5000);               //设置迭代的最大次数
	seg.setDistanceThreshold(0.1);           //设置内点到模型的距离允许最大值 
	seg.setRadiusLimits(6.0, 7.0);              //设置估计出圆柱模型的半径范围,原来是2.3-2.8
	seg.setInputCloud(cloud_filtered2);
	seg.setInputNormals(normals2);
	//获取圆柱模型系数和圆柱上的点
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	cout << "Cylinder coefficients: " << *coefficients_cylinder << endl;
	//-----------------------------存储点云到输出文件----------------------------
	string outFile_cylinder = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\公路隧道数据-秀山移动\\中间处理结果\\4cylinderResult.pcd";
	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ >);
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty())
		cout << "Can't find the cylindrical component." << endl;
	else
	{
		cout << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << endl;
		writer.write(outFile_cylinder, *cloud_cylinder, false);
	}
	//---------------可视化，从左到右依次是原始点云，平面，圆柱------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("segment display"));
	//原始点云
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.33, 1, v1);
	viewer->setBackgroundColor(0, 255, 0, v1);
	viewer->addPointCloud(cloud, "cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	//平面
	int v2(0);
	viewer->createViewPort(0.33, 0.0, 0.66, 1, v2);
	viewer->setBackgroundColor(0, 0, 255, v2);
	viewer->addPointCloud(cloud_plane, "cloud_plane", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_plane");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_plane");
	//圆柱
	int v3(0);
	viewer->createViewPort(0.66, 0.0, 1, 1, v3);
	viewer->setBackgroundColor(0, 255, 0, v3);
	viewer->addPointCloud(cloud_cylinder, "cloud_cylinder", v3);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_cylinder");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_cylinder");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
	return 0;
}


