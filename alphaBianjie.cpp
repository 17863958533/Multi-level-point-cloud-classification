#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\surface\concave_hull.h>
#include <pcl\console\time.h>

using namespace std;
//�ó�����Ҫʹ��Alpha Shapes�㷨���е�����ĳһͶӰ���ϵı߽���ȡ
int main4()
{
	//----------------------------- ���ص��� ----------------------------
	cout << "->���ڼ��ص���..." << endl;
	string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb��ʽ\\1Cloud.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile(readPath, *cloud) < 0)
	{
		PCL_ERROR("->�����ļ������ڣ�\a\n");
		return -1;
	}
	cout << "->������ " << cloud->points.size() << " �����ݵ�" << endl;
	//==================================================================


	//----------------------------- ����ͶӰ ----------------------------
	cout << "->����ˮƽ��ͶӰ..." << endl;
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		//cloud->points[i].z = -1.0f;
		cloud->points[i].y = 99.0f;
		
	}
	//==================================================================

	//-------------------------- ͶӰ�߽��²��� --------------------------
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_sub(new pcl::PointCloud<pcl::PointXYZ>);	//ͶӰ�²�������
	pcl::VoxelGrid<pcl::PointXYZ> vg;			//���������²�������
	vg.setInputCloud(cloud);			//�����²����������
	vg.setLeafSize(0.05f, 0.05f, 0.05f);//��������դ��߳�
	vg.filter(*cloud_sub);				//ִ�������²���
	//==================================================================

	//-------------------- Alpha Shapes ��ȡͶӰ�߽� --------------------
	pcl::console::TicToc time;
	time.tic();
	cout << "->������ȡ�߽�..." << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> ch;
	ch.setInputCloud(cloud_sub);
	ch.setAlpha(0.15);
	ch.reconstruct(*cloud_boundary);
	cout << "->�߽����ȡ��ʱ��" << time.toc() / 1000 << " s" << endl;
	//==================================================================


	//---------------------------- ����߽���� --------------------------
	cout << "->���ڱ���߽����..." << endl;
	string outPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb��ʽ\\Y_myalpha_estimation.pcd";
	if (!cloud_boundary->empty())
	{
		pcl::io::savePCDFileBinary(outPath, *cloud_boundary);
		cout << "->����ȡ�� " << cloud_boundary->points.size() << " ���߽��" << endl;
	}
	else
	{
		PCL_ERROR("�߽����Ϊ�գ�\a\n");
		system("pause");
		return -1;
	}
	//==================================================================

	return 0;
}


