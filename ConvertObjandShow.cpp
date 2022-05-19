#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <boost/thread/thread.hpp>



int main12()
{
	//��������1
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::string inputFilePath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb��ʽ\\my_cylinder.pcd";
	std::string outputFilePath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\�м�����\\1CloudCylinder.ply";
	pcl::io::loadPCDFile(inputFilePath, *cloud);
	pcl::io::savePLYFile(outputFilePath, *cloud);*/
	//����������������1��ת��֮��ͨ��������վ��https://www.meshconvert.com/zh.html����ply��ʽ�ļ�ת��Ϊobj��ʽ�ļ�


	//��������2
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFile("��Ů.obj", mesh);
	//-----------------����ģ�Ϳ��ӻ�----------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->setWindowName("��ʾobj��Ƭģ��");
	viewer->addPolygonMesh(mesh, "my");
	//��������ģ����ʾģʽ
   //viewer->setRepresentationToSurfaceForAllActors(); //����ģ������Ƭ��ʽ��ʾ  
   //viewer->setRepresentationToPointsForAllActors(); //����ģ���Ե���ʽ��ʾ  
   //viewer->setRepresentationToWireframeForAllActors();  //����ģ�����߿�ͼģʽ��ʾ

	viewer->addCoordinateSystem(0.2);
	//viewer->initCameraParameters ();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);



}





