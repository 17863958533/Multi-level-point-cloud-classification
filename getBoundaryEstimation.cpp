
#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\features\normal_3d.h>
#include <pcl\features\boundary.h>
#include <pcl\console\time.h>

using namespace std;

typedef pcl::PointXYZ PointT;

//该函数使用投影边界提取（Boundary Estimation）的方法提取点云再某一投影面上的边界
int main2()
{
	//这里使用的是投影边界提取（Boundary Estimation）的方法
	//----------------------------- 加载点云 ----------------------------
	cout << "->正在加载点云..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb格式\\1Cloud.pcd";

	if (pcl::io::loadPCDFile(readPath, *cloud) < 0)
	{
		PCL_ERROR("->点云文件不存在！\a\n");
		return -1;
	}
	cout << "->共加载 " << cloud->points.size() << " 个数据点" << endl;
	//==================================================================


	//----------------------------- 点云投影 ----------------------------
	cout << "->正在水平面投影..." << endl;
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		cloud->points[i].z = 0.0f;
	}
	//==================================================================


	//-------------------------- 投影边界下采样 --------------------------
	pcl::PointCloud<PointT>::Ptr cloud_sub(new pcl::PointCloud<PointT>);	//投影下采样点云
	pcl::VoxelGrid<PointT> vg;			//创建体素下采样对象
	vg.setInputCloud(cloud);			//设置下采样输入点云
	vg.setLeafSize(0.05f, 0.05f, 0.05f);//设置体素栅格边长
	vg.filter(*cloud_sub);				//执行体素下采样
	//==================================================================


	//--------------------------- 投影边界提取 ---------------------------
	pcl::console::TicToc time;
	time.tic();
	cout << "->正在提取投影边界..." << endl;

	///法线估计
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;						//创建法线对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);	//创建法线估计对象
	normEst.setInputCloud(cloud_sub);	//设置法线估计输入点云
	normEst.setRadiusSearch(0.15);		//法线估计邻域半径
	normEst.compute(*normals);			//执行法线估计

	///边界估计
	pcl::PointCloud<pcl::Boundary> boundaries;										//创建边界对象
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;	//创建边界估计对象
	boundEst.setInputCloud(cloud_sub);			//设置边界估计输入点云
	boundEst.setInputNormals(normals);			//设置法向量
	boundEst.setRadiusSearch(0.15);				//边界搜索半径
	boundEst.setAngleThreshold(M_PI * 0.89);	//边界点角度阈值
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));	//设置搜索方式
	boundEst.compute(boundaries);				//执行边界估计

	///提取边界点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>); //边界点点云
	for (size_t i = 0; i < cloud_sub->points.size(); i++)
	{
		if (boundaries[i].boundary_point > 0)
		{
			cloud_boundary->push_back(cloud_sub->points[i]);
		}
	}
	cout << "->边界点提取用时：" << time.toc() / 1000 << " s" << endl;
	//==================================================================

	//---------------------------- 保存边界点云 --------------------------
	cout << "->正在保存边界点云..." << endl;
	string outPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb格式\\myalpha_estimation.pcd";

	if (!cloud_boundary->empty())
	{
		pcl::io::savePCDFileBinary(outPath, *cloud_boundary);
		cout << "->共提取到 " << cloud_boundary->points.size() << " 个边界点" << endl;
	}
	else
	{
		PCL_ERROR("->边界点云为空！\a\n");
		system("pause");
		return -1;
	}
	//==================================================================

	return 0;
}


