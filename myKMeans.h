#pragma once
#pragma once
//Kmeans.h
#pragma once

#include <iostream>
#include <algorithm>
#include<pcl/point_cloud.h>
#include<pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
class KMeans
{
private:
	unsigned int max_iteration_;
	const unsigned int cluster_num_;//k
	double pointsDist(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);

public:

	pcl::PointCloud<pcl::PointXYZ>::Ptr centre_points_;
	//KMeans() = default;
	KMeans(unsigned int k, unsigned int max_iteration);
	void kMeans(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>> &cluster_cloud1);

	~KMeans() {}
};





