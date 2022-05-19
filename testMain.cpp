#include <vtkAutoInit.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>
#include"myKMeans.h"
#include<pcl/io/obj_io.h>

using namespace std;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;



int main10()
{

	//pcl::io::saveOBJFile
	
	//-------------------------------------------------------------------------------------------
	//这部分程序主要使用K均值聚类方法进行分类
	//------------------------------- 加载点云 -------------------------------
	string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb格式\\1Cloud.pcd";

	KMeans test(4, 10);//这里设置4个中心点（即4个聚类）
	//设置初始值,，中心点的坐标，如果设置的不合理，分割结果会有很大的影响
	test.centre_points_->points.push_back(pcl::PointXYZ(0.091200, 102.106018, -0.891000));
	test.centre_points_->points.push_back(pcl::PointXYZ(-2.055200, 100.929199, 0.246300));
	test.centre_points_->points.push_back(pcl::PointXYZ(-2.169900, 101.010071, 2.744600));
	test.centre_points_->points.push_back(pcl::PointXYZ(2.220500, 100.792908, 2.791700));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointCloud<pcl::PointXYZ>> output_cloud;
	pcl::io::loadPCDFile(readPath, *cloud);
	std::cerr << "raw cloud size" << cloud->points.size() << std::endl;
	test.kMeans(cloud, output_cloud);

	for (int i = 0; i < 4; ++i)//代码第三行，设置几个中心点，这里就循环几次
	{
		//pcl::PointCloud<pcl::PointXYZ> cloud1;
		//cloud1 = output_cloud[i];
		std::cerr << "output_cloud[i].points.size()" << output_cloud[i].points.size() << std::endl;
		output_cloud[i].width = output_cloud[i].points.size();
		output_cloud[i].height = 1;
		output_cloud[i].resize(output_cloud[i].width * output_cloud[i].height);
		pcl::io::savePCDFile("C:/Users/LYY/Desktop/TurnnelPaper/MyData/中间数据/" + std::to_string(i) + ".pcd", output_cloud[i]);
		//pcl::io::savePCDFile()
	}


	//std::cerr << "output_cloud[i].points.size()" << output_cloud[0].points.size() << std::endl;
	//output_cloud[0].width = output_cloud[0].points.size();
	//output_cloud[0].height = 1;
	//output_cloud[0].resize(output_cloud[0].width * output_cloud[0].height);
	//pcl::io::savePCDFile("G:/VS_TEST/Kmeans/Kmeans/results/kmeans" + std::to_string(0) + ".pcd", output_cloud[0]);
	std::cout << "Finish!!!\n";

	//---------------------------------------------------------------------------------------------------------------

	return 0;
}
