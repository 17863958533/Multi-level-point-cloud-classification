#include <iostream>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
typedef pcl::PointXYZ PointT;

//本函数主要求取一堆点云当中的质心坐标，用于计算隧道切片中的中轴线的点
int main5(int argc, char **argv)
{
	// 导入点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	// 点云文件地址：https://github.com/PointCloudLibrary/data/blob/master/tutorials/table_scene_lms400.pcd
	string inFileName = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb格式\\Y_myalpha_estimation.pcd";
	if (pcl::io::loadPCDFile(inFileName, *cloud) == -1) { // 读取.pcd文件
		cerr << "can't read file table_scene_lms400.pcd" << endl;
		return -1;
	}

	// PCL函数计算质心
	Eigen::Vector4f centroid;					// 质心
	pcl::compute3DCentroid(*cloud, centroid);	// 齐次坐标，（c0,c1,c2,1）

	// 按公式计算质心
	PointT p_c;
	p_c.x = 0; p_c.y = 0; p_c.z = 0;
	for (auto p : cloud->points) {
		p_c.x += p.x;
		p_c.y += p.y;
		p_c.z += p.z;
	}

	p_c.x /= cloud->points.size();
	p_c.y /= cloud->points.size();
	p_c.z /= cloud->points.size();

	// 结果对比
	cout << "pcl计算点云质心结果：" << centroid << endl;
	cout << "按照公式计算点云质心结果：" << p_c << endl;
	// 可视化
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud);
	viewer.addCoordinateSystem();
	// 质心坐标
	PointT center;
	center.x = centroid(0);
	center.y = centroid(1);
	center.z = centroid(2);

	viewer.addSphere(center, 0.03, 1, 0, 0, "sphere", 0);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
	return 0;
}