#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <fstream>
#include <ctime>
class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZRGBA> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),                   //.02
      feature_radius_ (0.02f)                    //,02
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
	  filteratePointCloud ();
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

	//filterate the point clouds,such as passthrough,downsample
	void
	filteratePointCloud ()
	{
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyz_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::VoxelGrid<pcl::PointXYZRGBA> so;
		so.setInputCloud(xyz_);
		so.setLeafSize(0.003,0.003,0.003);                   //.002  leafsize could be defined front
		so.filter (*xyz_filtered);
		pcl::copyPointCloud(*xyz_filtered,*xyz_);
		std::cout<<xyz_->points.size()<<std::endl;
	}

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

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


int main(int argc, char **argv)
{
	std::clock_t start,finish;
	start=clock();
	if(argc<3)
	{
	  printf("No enough PCD file found!\n");
	  return (-1);
	}

	FeatureCloud src;
	FeatureCloud tgt;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile(argv[1],*cloud1);
	pcl::io::loadPCDFile(argv[2],*cloud2);
	src.setInputCloud(cloud1);
	tgt.setInputCloud(cloud2);

	//initial SAC-IA
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGBA, pcl::PointXYZRGBA, pcl::FPFHSignature33> sac_ia_;
	float min_sample_distance_ =0.02 ;                                  //0.05
    float max_correspondence_distance_ =0.01f*0.01f;                   //0.01f*0.01f;
    int nr_iterations_ =500;                                            //500
	sac_ia_.setMinSampleDistance (min_sample_distance_);
    sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
    sac_ia_.setMaximumIterations (nr_iterations_);
	sac_ia_.setInputCloud(src.getPointCloud());
	sac_ia_.setSourceFeatures(src.getLocalFeatures());
	sac_ia_.setInputTarget(tgt.getPointCloud());
	sac_ia_.setTargetFeatures(tgt.getLocalFeatures());

	pcl::PointCloud<pcl::PointXYZ>::Ptr reg_output;
	pcl::Registration<pcl::PointXYZRGBA,pcl::PointXYZRGBA,float>::PointCloudSource output;
	sac_ia_.align(output);
	std::cout<<"Fitness Score:"<<sac_ia_.getFitnessScore()<<std::endl;

	 Eigen::Matrix4f final_transformation;
	 final_transformation = sac_ia_.getFinalTransformation();
	 // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = final_transformation.block<3,3>(0, 0);
    Eigen::Vector3f translation = final_transformation.block<3,1>(0, 3);

    printf ("\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

	pcl::io::loadPCDFile(argv[1],*cloud1);
	mytransformPointCloud (*cloud1,*cloud3, final_transformation);
	pcl::io::savePCDFile ("sac.pcd",*cloud3);

	std::ofstream fout("sac_matrix.txt");
	for (int i=0;i<16;i++) fout<<final_transformation(i)<<" ";
	
	finish=clock();
	double totaltime;
	totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
	std::cout<<"Total runtime is:"<<totaltime<<" sec\n";
	system("pause");
}