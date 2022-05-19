//���ǵڶ���ִ�е��ļ�
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <algorithm>
#include <ctime>
#include <pcl/visualization/pcl_plotter.h>
#include <fstream>//д��txt
#include<string>
#include <stdlib.h>//������ת�����ַ���


using namespace std;  // ���Լ��� std �������ռ�

//������������A��B����ȡ�������ƵĲ�֮ͬ����A��B��ʾ��A�в���B�еĵ㣬B��A��ʾ��B�в���A�еĵ�


int main(int argc, char** argv)
{
	string ReviseName;
	cout << "�Ƿ��Ѿ��޸�����ļ���A��B��B��A�������ƣ�������Y��N��" << endl;//�������ʱ�����������ʱ��ʱ���˵ķ�Ӧ�й�
	cin >> ReviseName;
	if (ReviseName != "Y")
	{
		return (-1);//������������
	}


	//-------------------------------------------------------------------------------
	srand(time(NULL));  //seeds rand() with the system time 
	time_t begin, end;
	begin = clock();  //��ʼ��ʱ
	//-------------------------------------------------------------------------------


	pcl::PointCloud<pcl::PointXYZ>::Ptr Acloud(new pcl::PointCloud<pcl::PointXYZ>);	// ���A����
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\LYY\\Desktop\\guwanqiData\\LandLubo\\111.pcd", *Acloud) == -1)//*�򿪵����ļ���
	{                                                                           //��·���ĸ�ʽ��ע��·���ķ�б��������Լ��Ĳ�ͬ����if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/rabbit_gra.pcd", *cloud) == -1) 
		PCL_ERROR("Couldn't read that boundary pcd file\n");                             // //����·���ĸ�ʽ��ֻ�ǰ�·��ɾ�����ɡ���if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit_gra.pcd", *cloud) == -1) 
		return(-1);
	}


	pcl::PointCloud<pcl::PointXYZ>::Ptr Bcloud(new pcl::PointCloud<pcl::PointXYZ>);	// ���B����
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\LYY\\Desktop\\guwanqiData\\LandLubo\\222.pcd", *Bcloud) == -1)//*�򿪵����ļ���
	{                                                                           //��·���ĸ�ʽ��ע��·���ķ�б��������Լ��Ĳ�ͬ����if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/rabbit_gra.pcd", *cloud) == -1) 
		PCL_ERROR("Couldn't read that outlier pcd file\n");                             // //����·���ĸ�ʽ��ֻ�ǰ�·��ɾ�����ɡ���if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit_gra.pcd", *cloud) == -1) 
		return(-1);
	}


	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(Bcloud);//��Bcloud�н�������
	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);//����1���������
	std::vector<float> pointNKNSquaredDistance(K);



	//���ò���
	int nr_Apoints = (int)Acloud->points.size();//���Acloud�Ĵ�С�����ڼ���ʱ��ѭ��
	int nr_Bpoints = (int)Bcloud->points.size();//���Bcloud�Ĵ�С�����ڼ���ʱ��ѭ��

	std::vector<int> LA01(nr_Apoints, 0); //��Ÿ��������غϵ�ı�ǡ���ʼ��Ϊ0������0��ʾ���غϵ㣬1��ʾ���غϵ�
	std::vector<int> LB01(nr_Bpoints, 0); //��Ÿ��������غϵ�ı�ǡ���ʼ��Ϊ0������0��ʾ���غϵ㣬1��ʾ���غϵ�

	std::vector<int> LA;  LA.clear();  //���A������B�ĵ㣬��ʼΪ�գ�����,�����峤�ȣ���˺���Ҫ��L.push_back(i)��ѹ�����ݡ�
	std::vector<int> LB;  LB.clear();  //���B������A�ĵ㣬��ʼΪ�գ�����,�����峤�ȣ���˺���Ҫ��L.push_back(i)��ѹ�����ݡ�

	for (int i = 0; i < nr_Apoints; ++i)//��A�е�ÿ������B�е���бȽ�
	{
		if (kdtree.nearestKSearch(Acloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			if (pointNKNSquaredDistance[0] == 0)//����������ĵ�һ�����Ϊ0����ô�����Ϊ�غϵ㣨��Ϊ��A��B�еĵ㣬�����ڰ�������������
			{
				LA01[i] = 1;
				LB01[pointIdxNKNSearch[0]] = 1;
			}
		}

	}



	for (int ii = 0; ii < nr_Apoints; ++ii)
	{
		if (LA01[ii] == 0)
			LA.push_back(ii);
	}

	for (int jj = 0; jj < nr_Bpoints; ++jj)
	{
		if (LB01[jj] == 0)
			LB.push_back(jj);
	}



	pcl::PointCloud<pcl::PointXYZ>::Ptr Arecloud(new pcl::PointCloud<pcl::PointXYZ>);// ��������Arecloud���A��ʣ�ࣨresidual���ĵ�
	pcl::copyPointCloud(*Acloud, LA, *Arecloud);//�����������Ƶ���

	pcl::PointCloud<pcl::PointXYZ>::Ptr Brecloud(new pcl::PointCloud<pcl::PointXYZ>);// ��������Arecloud���A��ʣ�ࣨresidual���ĵ�
	pcl::copyPointCloud(*Bcloud, LB, *Brecloud);//�����������Ƶ���


	//д�����
	pcl::io::savePCDFileASCII("C:\\Users\\LYY\\Desktop\\guwanqiData\\LandLubo\\A.pcd", *Arecloud); //�����Ʊ��浽PCD�ļ���
	pcl::io::savePCDFileASCII("C:\\Users\\LYY\\Desktop\\guwanqiData\\LandLubo\\B.pcd", *Brecloud); //�����Ʊ��浽PCD�ļ���




	//���A��B����һ�µĵ�
	std::vector<int> LAB;  LAB.clear();  //���AB��ͬ�ĵ㣬��ʼΪ�գ�����,�����峤�ȣ���˺���Ҫ��L.push_back(i)��ѹ�����ݡ�
	for (int iii = 0; iii < nr_Apoints; ++iii)
	{
		if (LA01[iii] == 1)
			LAB.push_back(iii);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr ABrecloud(new pcl::PointCloud<pcl::PointXYZ>);// ��������Arecloud���A��ʣ�ࣨresidual���ĵ�
	pcl::copyPointCloud(*Acloud, LAB, *ABrecloud);//�����������Ƶ���
	pcl::io::savePCDFileASCII("C:\\Users\\LYY\\Desktop\\guwanqiData\\LandLubo\\C.pcd", *ABrecloud); //�����Ʊ��浽PCD�ļ���


	//--------------------------------------------------------------------------------------------
	end = clock();  //������ʱ
	double Times = double(end - begin) / CLOCKS_PER_SEC; //��clock()�����Ľ��ת��Ϊ����Ϊ��λ����
	std::cout << "time: " << Times << "s" << std::endl;

	return 0;
}


//��������������Լ�д��ѭ�����������ڵ�kd��������λ�ã����Ǻ�ʱ������71���ӵ���kd��ֻ��12�롣��Ϊ�Լ�û���Ż���
//float Ax = Acloud->points[i].x;//���A�е�i��xyz���ꡣ
//float Ay = Acloud->points[i].y;
//float Az = Acloud->points[i].z;

//for (int j = 0; j < nr_Bpoints; ++j)
//{
//	if (LB01[j] == 0)
//	{
//		float Bx = Bcloud->points[j].x;//���B�е�i��xyz���ꡣ
//		float By = Bcloud->points[j].y;
//		float Bz = Bcloud->points[j].z;

//		if ((Ax == Bx) && (Ay == By)&&(Az == Bz))
//		{
//			LA01[i] = 1;
//			LB01[j] = 1;
//			break;//�ҵ��غϵ�֮��Ͳ��ü�������B��
//		}
//	}

//}