//���ǵ��������еĳ���
#include <vtkAutoInit.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>


using namespace std;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


int main788()
{
	//------------------------------- ���ص��� -------------------------------
	//string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\Ф�ǻ�ʦ�ֵ�EI����\\�����������ݻ���\\3ͨ�����������㷨��ȡ����ƽ̨�빩�綥��\\Shengyu4.pcd";
	//string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\Ф�ǻ�ʦ�ֵ�EI����\\�����������ݻ���\\0����ȥ��֮������\\1Cloudȥ��.pcd";
	//string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\��·�������-��ɽ�ƶ�\\�����������ݻ���2\\2ͨ��KT-DBCSAN�㷨��ȡ·��\\4road.pcd";

	string readPath = "C:\\Users\\LYY\\Desktop\\guwanqiData\\������.pcd";
	cout << "->���ڼ��ص���..." << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(readPath, *cloud) < 0)
	{
		PCL_ERROR("\a�����ļ������ڣ�\n");
		system("pause");
		return (-1);
	}
	cout << "->���ص�ĸ�����" << cloud->points.size() << endl;
	//========================================================================

	//------------------------------- ���߹��� -------------------------------
	cout << "->���ڹ��Ƶ��Ʒ���..." << endl;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;									//�������߹��ƶ���ne
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);	//������������
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);			//��ŷ���
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(20);
	ne.compute(*normals);
	//========================================================================

	//------------------------------- �������� -------------------------------
	cout << "->���ڽ�����������..." << endl;
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;	//�������������ָ����
	rg.setMinClusterSize(10);							//��������������С����  Ĭ��50
	rg.setMaxClusterSize(99999999);						//������������������
	rg.setSearchMethod(tree);							//������������
	rg.setNumberOfNeighbours(10);						//�������������ĵ���      Ĭ��30
	rg.setInputCloud(cloud);							//�����������
	rg.setInputNormals(normals);						//����������Ƶķ�����
	rg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);		//����ƽ����ֵ�����ȣ�������ȡͬһ����ĵ�
	rg.setCurvatureThreshold(0.8);						//����������ֵ�����������ķ���ƫ���С�������������֮��Ĳ��졣�����ֵС��������ֵ������㷨��ʹ������ӵĵ�����ص�����  Ĭ��1.0
	vector<pcl::PointIndices> clusters;					//������������
	rg.extract(clusters);								//��ȡ�������������浽����������

	cout << "->�������Ϊ" << clusters.size() << endl;
	//========================================================================

	//---------------------- Ϊ������������ɫ�������ӻ� ----------------------
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = rg.getColoredCloud();

	//string outFile_result = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\Ф�ǻ�ʦ�ֵ�EI����\\�����������ݻ���\\3ͨ�����������㷨��ȡ����ƽ̨�빩�綥��\\4_result.pcd";
	//string outFile_result ="C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\��·�������-��ɽ�ƶ�\\�����������ݻ���2\\3ͨ�����������㷨��ȡ����ƽ̨\\4road_result.pcd";

	string outFile_result = "C:\\Users\\LYY\\Desktop\\guwanqiData\\������_ReginGrowthResult.pcd";
	pcl::io::savePCDFile<pcl::PointXYZRGB>(outFile_result, *colored_cloud);

	//---------------------------- ��ȡ������������ ---------------------------
	int count = 0;
	pcl::PCDWriter writer;
	for (vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
	{
		PointCloudT::Ptr cloud_cluster(new PointCloudT);

		for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(colored_cloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		stringstream ss;
		ss << "cluster_" << count + 1 << ".pcd";
		//���ݽ�����˳������Դ�ļ�ͬ����Ŀ¼��
		writer.write(ss.str(), *cloud_cluster, true);
		cout << ss.str() << "������" << cloud_cluster->points.size() << endl;

		count++;
	}
	//==============================���ӻ�=====================================
	pcl::visualization::CloudViewer viewer("�����������");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped())
	{
	}
	

	return (0);
}
