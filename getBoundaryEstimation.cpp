
#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\features\normal_3d.h>
#include <pcl\features\boundary.h>
#include <pcl\console\time.h>

using namespace std;

typedef pcl::PointXYZ PointT;

//�ú���ʹ��ͶӰ�߽���ȡ��Boundary Estimation���ķ�����ȡ������ĳһͶӰ���ϵı߽�
int main2()
{
	//����ʹ�õ���ͶӰ�߽���ȡ��Boundary Estimation���ķ���
	//----------------------------- ���ص��� ----------------------------
	cout << "->���ڼ��ص���..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb��ʽ\\1Cloud.pcd";

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
		cloud->points[i].z = 0.0f;
	}
	//==================================================================


	//-------------------------- ͶӰ�߽��²��� --------------------------
	pcl::PointCloud<PointT>::Ptr cloud_sub(new pcl::PointCloud<PointT>);	//ͶӰ�²�������
	pcl::VoxelGrid<PointT> vg;			//���������²�������
	vg.setInputCloud(cloud);			//�����²����������
	vg.setLeafSize(0.05f, 0.05f, 0.05f);//��������դ��߳�
	vg.filter(*cloud_sub);				//ִ�������²���
	//==================================================================


	//--------------------------- ͶӰ�߽���ȡ ---------------------------
	pcl::console::TicToc time;
	time.tic();
	cout << "->������ȡͶӰ�߽�..." << endl;

	///���߹���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;						//�������߶���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);	//�������߹��ƶ���
	normEst.setInputCloud(cloud_sub);	//���÷��߹����������
	normEst.setRadiusSearch(0.15);		//���߹�������뾶
	normEst.compute(*normals);			//ִ�з��߹���

	///�߽����
	pcl::PointCloud<pcl::Boundary> boundaries;										//�����߽����
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;	//�����߽���ƶ���
	boundEst.setInputCloud(cloud_sub);			//���ñ߽�����������
	boundEst.setInputNormals(normals);			//���÷�����
	boundEst.setRadiusSearch(0.15);				//�߽������뾶
	boundEst.setAngleThreshold(M_PI * 0.89);	//�߽��Ƕ���ֵ
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));	//����������ʽ
	boundEst.compute(boundaries);				//ִ�б߽����

	///��ȡ�߽��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>); //�߽�����
	for (size_t i = 0; i < cloud_sub->points.size(); i++)
	{
		if (boundaries[i].boundary_point > 0)
		{
			cloud_boundary->push_back(cloud_sub->points[i]);
		}
	}
	cout << "->�߽����ȡ��ʱ��" << time.toc() / 1000 << " s" << endl;
	//==================================================================

	//---------------------------- ����߽���� --------------------------
	cout << "->���ڱ���߽����..." << endl;
	string outPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb��ʽ\\myalpha_estimation.pcd";

	if (!cloud_boundary->empty())
	{
		pcl::io::savePCDFileBinary(outPath, *cloud_boundary);
		cout << "->����ȡ�� " << cloud_boundary->points.size() << " ���߽��" << endl;
	}
	else
	{
		PCL_ERROR("->�߽����Ϊ�գ�\a\n");
		system("pause");
		return -1;
	}
	//==================================================================

	return 0;
}


