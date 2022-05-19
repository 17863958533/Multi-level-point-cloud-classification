#include <iostream>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
typedef pcl::PointXYZ PointT;

//��������Ҫ��ȡһ�ѵ��Ƶ��е��������꣬���ڼ��������Ƭ�е������ߵĵ�
int main5(int argc, char **argv)
{
	// �������
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	// �����ļ���ַ��https://github.com/PointCloudLibrary/data/blob/master/tutorials/table_scene_lms400.pcd
	string inFileName = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb��ʽ\\Y_myalpha_estimation.pcd";
	if (pcl::io::loadPCDFile(inFileName, *cloud) == -1) { // ��ȡ.pcd�ļ�
		cerr << "can't read file table_scene_lms400.pcd" << endl;
		return -1;
	}

	// PCL������������
	Eigen::Vector4f centroid;					// ����
	pcl::compute3DCentroid(*cloud, centroid);	// ������꣬��c0,c1,c2,1��

	// ����ʽ��������
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

	// ����Ա�
	cout << "pcl����������Ľ����" << centroid << endl;
	cout << "���չ�ʽ����������Ľ����" << p_c << endl;
	// ���ӻ�
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud);
	viewer.addCoordinateSystem();
	// ��������
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