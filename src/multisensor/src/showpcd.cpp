/****************************************************************************
*@文件:      showpcd:显示pcd文件
*@说明：
*@日期：     2018.11.26
*@修订者:    zps & qxx
*@修订日期:  2018.12.14
*****************************************************************************/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/icp.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <boost/thread/thread.hpp>
#include "time.h"
using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/airobot/map.pcd", *target_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file 666.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size () << " data points from 666.pcd" << std::endl;

    //点云变换 UTM格式 origin(457246.000000,4404758.000000)
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 457246, 4404758, 0.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr transform_cloud1 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*target_cloud,*transform_cloud1,transform_2);  //不言而喻
    // 保存转换的输入点云
    pcl::io::savePCDFileASCII ("777.pcd", *transform_cloud1);

    // 初始化点云可视化界面
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
    viewer_final_ndt (new pcl::visualization::PCLVisualizer ("3D Viewer Algorithm 1"));
    viewer_final_ndt->setBackgroundColor (0, 0, 0);

    // 对目标点云着色（红色）并可视化
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
    target_color (target_cloud, 255, 0, 0);
    viewer_final_ndt->addPointCloud<pcl::PointXYZI> (target_cloud, target_color, "target cloud");
    viewer_final_ndt->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "target cloud");
                                                        // 启动可视化
    viewer_final_ndt->addCoordinateSystem (1.0, "global");
    viewer_final_ndt->initCameraParameters ();


    // 初始化点云可视化界面
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
    viewer_final_icp (new pcl::visualization::PCLVisualizer ("3D Viewer Algorithm 2"));
    viewer_final_icp->setBackgroundColor (0, 0, 0);

    // 对目标点云着色（红色）并可视化
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
    target_color_icp (transform_cloud1, 255, 0, 0);
    viewer_final_icp->addPointCloud<pcl::PointXYZI> (transform_cloud1, target_color_icp, "target cloud");
    viewer_final_icp->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "target cloud");
                                                    // 启动可视化
    viewer_final_icp->addCoordinateSystem (1.0, "global");
    viewer_final_icp->initCameraParameters ();

    // 等待直到可视化窗口关闭。
    while ((!viewer_final_ndt->wasStopped ())||(!viewer_final_icp->wasStopped ()))
    {
        viewer_final_ndt->spinOnce (100);
        viewer_final_icp->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
