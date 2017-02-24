#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
int
main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>);
  // 填入点云数据
	pcl::io::loadPCDFile(argv[1],*cloud_in);
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (cloud_in);
    sor.setLeafSize (0.001f, 0.001f, 0.001f);
    sor.filter (*cloud_out);
	pcl::io::savePCDFile("down.pcd",*cloud_out);
  return (0);
}
