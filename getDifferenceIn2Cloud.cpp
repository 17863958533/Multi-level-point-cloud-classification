//这是第二个执行的文件
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <algorithm>
#include <ctime>
#include <pcl/visualization/pcl_plotter.h>
#include <fstream>//写入txt
#include<string>
#include <stdlib.h>//将整型转换成字符型


using namespace std;  // 可以加入 std 的命名空间

//给定两个点云A和B，求取两个点云的不同之处，A异B表示在A中不在B中的点，B异A表示在B中不在A中的点


int main(int argc, char** argv)
{
	string ReviseName;
	cout << "是否已经修改输出文件（A异B和B异A）的名称？请输入Y或N。" << endl;//不参与计时，因此与输入时间时的人的反应有关
	cin >> ReviseName;
	if (ReviseName != "Y")
	{
		return (-1);//跳出整个程序
	}


	//-------------------------------------------------------------------------------
	srand(time(NULL));  //seeds rand() with the system time 
	time_t begin, end;
	begin = clock();  //开始计时
	//-------------------------------------------------------------------------------


	pcl::PointCloud<pcl::PointXYZ>::Ptr Acloud(new pcl::PointCloud<pcl::PointXYZ>);	// 存放A点云
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\LYY\\Desktop\\guwanqiData\\LandLubo\\111.pcd", *Acloud) == -1)//*打开点云文件。
	{                                                                           //带路径的格式【注意路径的反斜杠与电脑自己的不同】：if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/rabbit_gra.pcd", *cloud) == -1) 
		PCL_ERROR("Couldn't read that boundary pcd file\n");                             // //不带路径的格式【只是把路径删掉即可】：if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit_gra.pcd", *cloud) == -1) 
		return(-1);
	}


	pcl::PointCloud<pcl::PointXYZ>::Ptr Bcloud(new pcl::PointCloud<pcl::PointXYZ>);	// 存放B点云
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\LYY\\Desktop\\guwanqiData\\LandLubo\\222.pcd", *Bcloud) == -1)//*打开点云文件。
	{                                                                           //带路径的格式【注意路径的反斜杠与电脑自己的不同】：if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/rabbit_gra.pcd", *cloud) == -1) 
		PCL_ERROR("Couldn't read that outlier pcd file\n");                             // //不带路径的格式【只是把路径删掉即可】：if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit_gra.pcd", *cloud) == -1) 
		return(-1);
	}


	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(Bcloud);//在Bcloud中进行搜索
	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);//进行1邻域点搜索
	std::vector<float> pointNKNSquaredDistance(K);



	//设置参数
	int nr_Apoints = (int)Acloud->points.size();//获得Acloud的大小，用于计算时的循环
	int nr_Bpoints = (int)Bcloud->points.size();//获得Bcloud的大小，用于计算时的循环

	std::vector<int> LA01(nr_Apoints, 0); //存放各点有无重合点的标记。初始化为0，其中0表示无重合点，1表示有重合点
	std::vector<int> LB01(nr_Bpoints, 0); //存放各点有无重合点的标记。初始化为0，其中0表示无重合点，1表示有重合点

	std::vector<int> LA;  LA.clear();  //存放A中异于B的点，初始为空！！！,不定义长度，因此后面要用L.push_back(i)来压入数据。
	std::vector<int> LB;  LB.clear();  //存放B中异于A的点，初始为空！！！,不定义长度，因此后面要用L.push_back(i)来压入数据。

	for (int i = 0; i < nr_Apoints; ++i)//对A中的每个点与B中点进行比较
	{
		if (kdtree.nearestKSearch(Acloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			if (pointNKNSquaredDistance[0] == 0)//如果搜索到的第一个点距为0，那么这个点为重合点（因为用A搜B中的点，不存在包含自身的情况）
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



	pcl::PointCloud<pcl::PointXYZ>::Ptr Arecloud(new pcl::PointCloud<pcl::PointXYZ>);// 创建点云Arecloud存放A中剩余（residual）的点
	pcl::copyPointCloud(*Acloud, LA, *Arecloud);//按照索引复制点云

	pcl::PointCloud<pcl::PointXYZ>::Ptr Brecloud(new pcl::PointCloud<pcl::PointXYZ>);// 创建点云Arecloud存放A中剩余（residual）的点
	pcl::copyPointCloud(*Bcloud, LB, *Brecloud);//按照索引复制点云


	//写入磁盘
	pcl::io::savePCDFileASCII("C:\\Users\\LYY\\Desktop\\guwanqiData\\LandLubo\\A.pcd", *Arecloud); //将点云保存到PCD文件中
	pcl::io::savePCDFileASCII("C:\\Users\\LYY\\Desktop\\guwanqiData\\LandLubo\\B.pcd", *Brecloud); //将点云保存到PCD文件中




	//输出A与B坐标一致的点
	std::vector<int> LAB;  LAB.clear();  //存放AB共同的点，初始为空！！！,不定义长度，因此后面要用L.push_back(i)来压入数据。
	for (int iii = 0; iii < nr_Apoints; ++iii)
	{
		if (LA01[iii] == 1)
			LAB.push_back(iii);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr ABrecloud(new pcl::PointCloud<pcl::PointXYZ>);// 创建点云Arecloud存放A中剩余（residual）的点
	pcl::copyPointCloud(*Acloud, LAB, *ABrecloud);//按照索引复制点云
	pcl::io::savePCDFileASCII("C:\\Users\\LYY\\Desktop\\guwanqiData\\LandLubo\\C.pcd", *ABrecloud); //将点云保存到PCD文件中


	//--------------------------------------------------------------------------------------------
	end = clock();  //结束计时
	double Times = double(end - begin) / CLOCKS_PER_SEC; //将clock()函数的结果转化为以秒为单位的量
	std::cout << "time: " << Times << "s" << std::endl;

	return 0;
}


//下面这个代码是自己写的循环，放在现在的kd树搜索的位置，但是耗时极长，71分钟的用kd树只需12秒。因为自己没有优化。
//float Ax = Acloud->points[i].x;//获得A中点i的xyz坐标。
//float Ay = Acloud->points[i].y;
//float Az = Acloud->points[i].z;

//for (int j = 0; j < nr_Bpoints; ++j)
//{
//	if (LB01[j] == 0)
//	{
//		float Bx = Bcloud->points[j].x;//获得B中点i的xyz坐标。
//		float By = Bcloud->points[j].y;
//		float Bz = Bcloud->points[j].z;

//		if ((Ax == Bx) && (Ay == By)&&(Az == Bz))
//		{
//			LA01[i] = 1;
//			LB01[j] = 1;
//			break;//找到重合点之后就不用继续搜索B了
//		}
//	}

//}