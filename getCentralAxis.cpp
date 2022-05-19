#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\common\common.h>
#include <pcl\console\time.h>

using namespace std;

typedef pcl::PointXYZ PointT;

//�ⲿ�ֳ����Ǽ򵥼�������ڶ�άͶӰ���ϵ������ߵ���
int main3()
{
	//---------------------------- ���ص��� ----------------------------
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb��ʽ\\1Cloud.pcd";
	if (pcl::io::loadPCDFile(readPath, *cloud) < 0)
	{
		PCL_ERROR("->�����ļ������ڣ�\a\n");
		system("pause");
		return -1;
	}
	cout << "->�������� " << cloud->points.size() << " ����..." << endl;
	//=================================================================


	pcl::console::TicToc time;
	time.tic();
	//----------------------------- ����ͶӰ ---------------------------
	cout << "->����ˮƽ��ͶӰ..." << endl;
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		cloud->points[i].z = 0.0f;
	}
	//=================================================================

	//-------------------------- ͶӰ�߽��²��� --------------------------
	cout << "->�����²���ͶӰ����..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud_sub(new pcl::PointCloud<PointT>);	//ͶӰ�²�������
	pcl::VoxelGrid<PointT> vg;			//���������²�������
	vg.setInputCloud(cloud);			//�����²����������
	vg.setLeafSize(0.1f, 0.1f, 0.1f);	//��������դ��߳�
	vg.filter(*cloud_sub);				//ִ�������²���
	//==================================================================

	//------------------------- XOYƽ�����ߵ���ȡ -----------------------
	cout << "->������ȡˮƽ����..." << endl;
	pcl::PointCloud<PointT>::Ptr cloud_center(new pcl::PointCloud<PointT>);	//ˮƽ���ߵ���

	///��ȡͶӰ����Y������ֵ
	PointT min_y, max_y;
	pcl::getMinMax3D(*cloud_sub, min_y, max_y);

	///������Ƭ�����ߵ���ȡ
	float delt_y = 1.0f;				//��Ƭ����
	for (float yi = min_y.y; yi < max_y.y; yi += delt_y)
	{
		vector<int> vec_idx;				//��Ƭ��������
		for (size_t i = 0; i < cloud_sub->points.size(); i++)
		{
			if (cloud_sub->points[i].y >= yi && cloud_sub->points[i].y < (yi + delt_y))
			{
				vec_idx.push_back(i);
			}
		}
		pcl::PointCloud<PointT>::Ptr slice(new pcl::PointCloud<PointT>);
		pcl::copyPointCloud(*cloud_sub, vec_idx, *slice);		//���ݵ���������ȡ��Ƭ����

		///��ȡ��Ƭ����X������ֵ
		PointT min_x, max_x;
		pcl::getMinMax3D(*slice, min_x, max_x);

		///��ȡˮƽ���ߵ�
		PointT centerPt;
		centerPt.x = (min_x.x + max_x.x) / 2.0f;
		centerPt.y = yi;
		centerPt.z = 0.0f;
		cloud_center->push_back(centerPt);
	}
	//=================================================================
	cout << "->ˮƽ������ȡ��ʱ��" << time.toc() / 1000 << " s" << endl;


	//-------------------------- �������ߵ��� --------------------------
	string outPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb��ʽ\\my_CentralAxis.pcd";
	if (!cloud_center->empty())
	{
		pcl::io::savePCDFileASCII(outPath, *cloud_center);
		cout << "->����ȡ���ߵ������" << cloud_center->points.size() << endl;
	}
	else
	{
		PCL_ERROR("->δ��ȡ�����ߵ㣡\a\n");
		system("pause");
		return -1;
	}
	//=================================================================

	return 0;
}

