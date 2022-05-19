//这是第三个运行的程序
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
	//------------------------------- 加载点云 -------------------------------
	//string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\肖智华师兄的EI论文\\分类论文数据汇总\\3通过区域生长算法提取逃生平台与供电顶轨\\Shengyu4.pcd";
	//string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\肖智华师兄的EI论文\\分类论文数据汇总\\0初步去噪之后数据\\1Cloud去噪.pcd";
	//string readPath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\公路隧道数据-秀山移动\\分类论文数据汇总2\\2通过KT-DBCSAN算法提取路面\\4road.pcd";

	string readPath = "C:\\Users\\LYY\\Desktop\\guwanqiData\\电力线.pcd";
	cout << "->正在加载点云..." << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(readPath, *cloud) < 0)
	{
		PCL_ERROR("\a点云文件不存在！\n");
		system("pause");
		return (-1);
	}
	cout << "->加载点的个数：" << cloud->points.size() << endl;
	//========================================================================

	//------------------------------- 法线估计 -------------------------------
	cout << "->正在估计点云法线..." << endl;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;									//创建法线估计对象ne
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);	//设置搜索方法
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);			//存放法线
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(20);
	ne.compute(*normals);
	//========================================================================

	//------------------------------- 区域生长 -------------------------------
	cout << "->正在进行区域生长..." << endl;
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;	//创建区域生长分割对象
	rg.setMinClusterSize(10);							//设置满足聚类的最小点数  默认50
	rg.setMaxClusterSize(99999999);						//设置满足聚类的最大点数
	rg.setSearchMethod(tree);							//设置搜索方法
	rg.setNumberOfNeighbours(10);						//设置邻域搜索的点数      默认30
	rg.setInputCloud(cloud);							//设置输入点云
	rg.setInputNormals(normals);						//设置输入点云的法向量
	rg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);		//设置平滑阈值，弧度，用于提取同一区域的点
	rg.setCurvatureThreshold(0.8);						//设置曲率阈值，如果两个点的法线偏差很小，则测试其曲率之间的差异。如果该值小于曲率阈值，则该算法将使用新添加的点继续簇的增长  默认1.0
	vector<pcl::PointIndices> clusters;					//聚类索引向量
	rg.extract(clusters);								//获取聚类结果，并保存到索引向量中

	cout << "->聚类个数为" << clusters.size() << endl;
	//========================================================================

	//---------------------- 为聚类点云添加颜色，并可视化 ----------------------
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = rg.getColoredCloud();

	//string outFile_result = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\肖智华师兄的EI论文\\分类论文数据汇总\\3通过区域生长算法提取逃生平台与供电顶轨\\4_result.pcd";
	//string outFile_result ="C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\公路隧道数据-秀山移动\\分类论文数据汇总2\\3通过区域生长算法提取逃生平台\\4road_result.pcd";

	string outFile_result = "C:\\Users\\LYY\\Desktop\\guwanqiData\\电力线_ReginGrowthResult.pcd";
	pcl::io::savePCDFile<pcl::PointXYZRGB>(outFile_result, *colored_cloud);

	//---------------------------- 提取区域生长聚类 ---------------------------
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
		//数据将按照顺序存放在源文件同级的目录下
		writer.write(ss.str(), *cloud_cluster, true);
		cout << ss.str() << "点数：" << cloud_cluster->points.size() << endl;

		count++;
	}
	//==============================可视化=====================================
	pcl::visualization::CloudViewer viewer("区域生长结果");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped())
	{
	}
	

	return (0);
}
