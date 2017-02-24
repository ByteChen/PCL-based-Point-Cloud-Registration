#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
int
 main (int argc, char** argv)
{ 
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::io::loadPCDFile(argv[1],*cloud1);
	pcl::io::loadPCDFile(argv[2],*cloud2);

	*cloud1 += *cloud2;

	pcl::io::savePCDFile("out_plus.pcd",*cloud1);
    return (0);
}
