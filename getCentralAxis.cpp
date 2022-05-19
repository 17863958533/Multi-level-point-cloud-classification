#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\common\common.h>
#include <pcl\console\time.h>

using namespace std;

typedef pcl::PointXYZ PointT;

//这部分程序是简单计算隧道在二维投影面上的中轴线点云
int main3()
{
	//---------------------------- 加载点云 ----------------------------
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb格式\\1Cloud.pcd";
	if (pcl::io::loadPCDFile(readPath, *cloud) < 0)
	{
		PCL_ERROR("->点云文件不存在！\a\n");
		system("pause");
		return -1;
	}
	cout << "->共加载了 " << cloud->points.size() << " 个点..." << endl;
	//=================================================================


	pcl::console::TicToc time;
	time.tic();
	//----------------------------- 点云投影 ---------------------------
	cout << "->正在水平面投影..." << endl;
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		cloud->points[i].z = 0.0f;
	}
	//=================================================================

	//-------------------------- 投影边界下采样 --------------------------
	cout << "->正在下采样投影点云..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud_sub(new pcl::PointCloud<PointT>);	//投影下采样点云
	pcl::VoxelGrid<PointT> vg;			//创建体素下采样对象
	vg.setInputCloud(cloud);			//设置下采样输入点云
	vg.setLeafSize(0.1f, 0.1f, 0.1f);	//设置体素栅格边长
	vg.filter(*cloud_sub);				//执行体素下采样
	//==================================================================

	//------------------------- XOY平面中线点提取 -----------------------
	cout << "->正在提取水平中线..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud_center(new pcl::PointCloud<PointT>);	//水平中线点云

	///获取投影点云Y坐标最值
	PointT min_y, max_y;
	pcl::getMinMax3D(*cloud_sub, min_y, max_y);

	///点云切片与中线点提取
	float delt_y = 1.0f;				//切片步长
	for (float yi = min_y.y; yi < max_y.y; yi += delt_y)
	{
		vector<int> vec_idx;				//切片点云索引
		for (size_t i = 0; i < cloud_sub->points.size(); i++)
		{
			if (cloud_sub->points[i].y >= yi && cloud_sub->points[i].y < (yi + delt_y))
			{
				vec_idx.push_back(i);
			}
		}
		pcl::PointCloud<PointT>::Ptr slice(new pcl::PointCloud<PointT>);
		pcl::copyPointCloud(*cloud_sub, vec_idx, *slice);		//根据点云索引提取切片点云

		///获取切片点云X坐标最值
		PointT min_x, max_x;
		pcl::getMinMax3D(*slice, min_x, max_x);

		///提取水平中线点
		PointT centerPt;
		centerPt.x = (min_x.x + max_x.x) / 2.0f;
		centerPt.y = yi;
		centerPt.z = 0.0f;
		cloud_center->push_back(centerPt);
	}
	//=================================================================
	cout << "->水平中线提取用时：" << time.toc() / 1000 << " s" << endl;


	//-------------------------- 保存中线点云 --------------------------
	string outPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb格式\\my_CentralAxis.pcd";
	if (!cloud_center->empty())
	{
		pcl::io::savePCDFileASCII(outPath, *cloud_center);
		cout << "->共提取中线点个数：" << cloud_center->points.size() << endl;
	}
	else
	{
		PCL_ERROR("->未提取到中线点！\a\n");
		system("pause");
		return -1;
	}
	//=================================================================

	return 0;
}

