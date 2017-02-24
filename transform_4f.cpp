#include <iostream>
#include  <fstream>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <vector>



void
	mytransformPointCloud (const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, 
                          pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out,
                          const Eigen::Matrix4f &transform)
{
  if (&cloud_in != &cloud_out)
  {
    // Note: could be replaced by cloud_out = cloud_in
    cloud_out.header   = cloud_in.header;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.points.reserve (cloud_out.points.size ());
    cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
    cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
    cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
  }

  if (cloud_in.is_dense)
  {
    // If the dataset is dense, simply transform it!
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      //cloud_out.points[i].getVector3fMap () = transform * cloud_in.points[i].getVector3fMap ();
      //Eigen::Matrix<Scalar, 3, 1> pt (cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
      cloud_out[i].x = static_cast<float> (transform (0, 0) * cloud_in[i].x + transform (0, 1) * cloud_in[i].y + transform (0, 2) * cloud_in[i].z + transform (0, 3));
      cloud_out[i].y = static_cast<float> (transform (1, 0) * cloud_in[i].x + transform (1, 1) * cloud_in[i].y + transform (1, 2) * cloud_in[i].z + transform (1, 3));
      cloud_out[i].z = static_cast<float> (transform (2, 0) * cloud_in[i].x + transform (2, 1) * cloud_in[i].y + transform (2, 2) * cloud_in[i].z + transform (2, 3));
	  cloud_out[i].rgba = cloud_in[i].rgba;
    }
  }
  //could have else
}


int
main(int argc, char** argv)
{
	if(argc<3)
	   { printf("error input!\n"); return -1;}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::io::loadPCDFile(argv[1],*cloud_in);

	Eigen::Matrix4f transformation;
	std::ifstream fin(argv[2]);
	for(int i=0;i<16;i++) fin>>transformation(i);
	std::cout<<transformation(0)<<std::endl;
	std::cout<<transformation(2)<<std::endl;
	std::cout<<transformation(5)<<std::endl;
	std::cout<<transformation(1,1)<<std::endl;

	mytransformPointCloud(*cloud_in,*cloud_out,transformation);
	pcl::io::savePCDFile ("out.pcd",*cloud_out);
	system("pause");
	return 0;
}
