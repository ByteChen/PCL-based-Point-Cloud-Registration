#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
 

 int user_data;
void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0, 0, 0);
 /*   pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
 */   
}


int 
main ()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("test_01.pcd", *cloud);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");    
    //showCloud������ͬ���ģ��ڴ˴��ȴ�ֱ����Ⱦ��ʾΪֹ
    viewer.showCloud(cloud);
	


    //��ע�ắ���ڿ��ӻ�ʱֻ����һ��
 viewer.runOnVisualizationThreadOnce (viewerOneOff);
    //��ע�ắ������Ⱦ���ʱÿ�ζ�����
 //   viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {
    //�ڴ˴����������������
//    user_data++;
    }
    return 0;
}

