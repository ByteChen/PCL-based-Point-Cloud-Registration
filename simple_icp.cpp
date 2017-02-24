
//说明：本程序是将src配准到tgt，也就是把命令参数里的1配到2.特此说明，以资鼓励

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <ctime>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

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
 main (int argc, char** argv)
{ 
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	PointCloud::Ptr result(new PointCloud);
   
	pcl::io::loadPCDFile(argv[1],*src);
	pcl::io::loadPCDFile(argv[2],*tgt);

   

	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyz_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
		PointCloud::Ptr src_filtered(new PointCloud);
		PointCloud::Ptr tgt_filtered(new PointCloud);

		pcl::VoxelGrid<pcl::PointXYZRGBA> so;
		so.setInputCloud(src);
		so.setLeafSize(0.0003,0.0003,0.0003);                   //.002  leafsize could be defined front
		so.filter (*xyz_filtered);
		pcl::copyPointCloud(*xyz_filtered,*src_filtered);
		std::cout<<src_filtered->points.size()<<std::endl;

		so.setInputCloud(tgt);
	//	so.setLeafSize(0.003,0.003,0.003);                   //.002  leafsize could be defined front
		so.filter (*xyz_filtered);
		pcl::copyPointCloud(*xyz_filtered,*tgt);
        std::cout<<tgt->points.size()<<std::endl;


	std::clock_t start,finish;
	start=clock();                                   

	    pcl::search::KdTree<PointT>::Ptr tree1 (new pcl::search::KdTree<PointT>());
        tree1->setInputCloud(src_filtered);
        pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT>());
        tree2->setInputCloud(tgt);
        icp.setSearchMethodSource(tree1);
        icp.setSearchMethodTarget(tree2);

	
		icp.setInputCloud  (src_filtered);
        icp.setInputTarget (tgt);
    
    icp.setMaxCorrespondenceDistance (0.001);                    //0.001
     // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (500);
     // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-15);
   // Set the euclidean distance difference epsilon (criterion 3)
      icp.setEuclideanFitnessEpsilon (1e-15);
   
   // Perform the alignment
    icp.align (*result);

	std::cout<<"Fitness score:"<<icp.getFitnessScore()<<"\n";
	Eigen::Matrix4f transformation = icp.getFinalTransformation ();
	std::cout<<"Matrix:\n";
	std::cout<<transformation<<"\n";                        //print matrix in screen
	std::ofstream fout("icp_matrix.txt");
	for (int i=0;i<16;i++) fout<<transformation(i)<<" ";   //save matrix into txt
	mytransformPointCloud(*src,*result,transformation);
	pcl::io::savePCDFile ("out.pcd",*result);              //save pcd file

	finish=clock();
	double totaltime;
	totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
	std::cout<<"Reg Total runtime is:"<<totaltime<<" sec\n";

//	pcl::io::savePCDFile("icp.pcd",*result);
	system("pause");
    return (0);
}
