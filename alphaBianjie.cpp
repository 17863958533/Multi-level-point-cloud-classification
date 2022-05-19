#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\surface\concave_hull.h>
#include <pcl\console\time.h>

using namespace std;
//该程序主要使用Alpha Shapes算法进行点云再某一投影面上的边界提取
int main4()
{
	//----------------------------- 加载点云 ----------------------------
	cout << "->正在加载点云..." << endl;
	string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb格式\\1Cloud.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
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
		//cloud->points[i].z = -1.0f;
		cloud->points[i].y = 99.0f;
		
	}
	//==================================================================

	//-------------------------- 投影边界下采样 --------------------------
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_sub(new pcl::PointCloud<pcl::PointXYZ>);	//投影下采样点云
	pcl::VoxelGrid<pcl::PointXYZ> vg;			//创建体素下采样对象
	vg.setInputCloud(cloud);			//设置下采样输入点云
	vg.setLeafSize(0.05f, 0.05f, 0.05f);//设置体素栅格边长
	vg.filter(*cloud_sub);				//执行体素下采样
	//==================================================================

	//-------------------- Alpha Shapes 提取投影边界 --------------------
	pcl::console::TicToc time;
	time.tic();
	cout << "->正在提取边界..." << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> ch;
	ch.setInputCloud(cloud_sub);
	ch.setAlpha(0.15);
	ch.reconstruct(*cloud_boundary);
	cout << "->边界点提取用时：" << time.toc() / 1000 << " s" << endl;
	//==================================================================


	//---------------------------- 保存边界点云 --------------------------
	cout << "->正在保存边界点云..." << endl;
	string outPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb格式\\Y_myalpha_estimation.pcd";
	if (!cloud_boundary->empty())
	{
		pcl::io::savePCDFileBinary(outPath, *cloud_boundary);
		cout << "->共提取到 " << cloud_boundary->points.size() << " 个边界点" << endl;
	}
	else
	{
		PCL_ERROR("边界点云为空！\a\n");
		system("pause");
		return -1;
	}
	//==================================================================

	return 0;
}


