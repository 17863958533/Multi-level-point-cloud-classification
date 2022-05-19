#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <boost/thread/thread.hpp>



int main12()
{
	//操作部分1
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::string inputFilePath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\pdb格式\\my_cylinder.pcd";
	std::string outputFilePath = "C:\\Users\\LYY\\Desktop\\TurnnelPaper\\MyData\\中间数据\\1CloudCylinder.ply";
	pcl::io::loadPCDFile(inputFilePath, *cloud);
	pcl::io::savePLYFile(outputFilePath, *cloud);*/
	//在完成上面操作部分1的转换之后，通过在线网站（https://www.meshconvert.com/zh.html）将ply格式文件转化为obj格式文件


	//操作部分2
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFile("侍女.obj", mesh);
	//-----------------点云模型可视化----------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->setBackgroundColor(0, 0, 0);
	viewer->setWindowName("显示obj面片模型");
	viewer->addPolygonMesh(mesh, "my");
	//设置网格模型显示模式
   //viewer->setRepresentationToSurfaceForAllActors(); //网格模型以面片形式显示  
   //viewer->setRepresentationToPointsForAllActors(); //网格模型以点形式显示  
   //viewer->setRepresentationToWireframeForAllActors();  //网格模型以线框图模式显示

	viewer->addCoordinateSystem(0.2);
	//viewer->initCameraParameters ();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);



}





