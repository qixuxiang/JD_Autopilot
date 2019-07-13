/*****************************************************************************
*@文件:      pcl_ndt:地图匹配
*@说明：     采用ndt与icp地图匹配算法，目标实现粗匹配与精准匹配
*@日期：     2018.11.25
*@修订者:    zps & qxx
*@修订日期:  2018.12.14
*****************************************************************************/
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/icp.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "time.h"
#include <ctime>
#include <angles/angles.h>
#include <global_coords.h>

using namespace std;
using namespace global_coords;


int main (int argc, char** argv)
{
  cout << "M_PI IN C++" << setprecision(20) << M_PI <<endl; // pi in C++, setprecision
  clock_t start,ends;

  // 加载地图点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/airobot/1543669258343287.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file Map.pcd \n");
    return (-1);
  }
  cout << "Loaded " << target_cloud->size () << " data points from Map.pcd" << endl;

  // 加载第一视角点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/airobot/1542356255752738.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file CarLidar.pcd \n");
    return (-1);
  }
  cout << "Loaded " << input_cloud->size () << " data points from CarLidar.pcd" << endl;
  start = clock();
  // 去除NAN点
  vector<int> indices_src; //保存去除的点的索引
  pcl::removeNaNFromPointCloud(*input_cloud,*input_cloud, indices_src);
  cout<<"remove *input_cloud nan"<<endl;

  // 采样滤波
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.5, 0.5, 0.5);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_Tcloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_Tfilter;
  approximate_voxel_Tfilter.setLeafSize (0.5, 0.5, 0.5);
  approximate_voxel_Tfilter.setInputCloud (target_cloud);
  approximate_voxel_Tfilter.filter (*filtered_Tcloud);

  ends = clock();
  cout<<(double)(ends-start)/CLOCKS_PER_SEC<<endl;

  cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from CarLidar.pcd" << endl;

  //初始化无人车在地图的初始位置39.7917338786 116.498618594  39.791748,116.498636
  LatLonCoords latlon(39.791748,116.498636);       //initial latitude and longitude
  UtmCoords utm = latLonToUtm(latlon);             //convert to utm

  float angle=3.9;
  // 设置使用机器人测距法得到的初始对准估计结果
  Eigen::AngleAxisf init_rotation (angle, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (utm.x - 457246 , utm.y - 4404758, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  cout<< utm.x - 457246  <<endl;
  cout<< utm.y - 4404758 <<endl;

  cout<< init_guess <<endl;

  // icp配准
  pcl::PointCloud<pcl::PointXYZI>::Ptr icp_result (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(filtered_cloud);
  icp.setInputTarget(target_cloud);
  // Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (1000);
  // 最大迭代次数
  icp.setMaximumIterations (30);
  // 两次变化矩阵之间的差值
  icp.setTransformationEpsilon (1e-10);
  // 均方误差
  icp.setEuclideanFitnessEpsilon (0.2);
  icp.align(*icp_result,init_guess);

  cout << "Iterative Closest Point has converged:" << icp.hasConverged ()
            << " score: " << icp.getFitnessScore () << endl;

  // 使用创建的变换对未过滤的输入点云进行变换
  pcl::transformPointCloud (*input_cloud, *icp_result, icp.getFinalTransformation ());

  ends = clock();
  cout<<(double)(ends-start)/CLOCKS_PER_SEC<<endl;
  // 保存转换的输入点云
  //pcl::io::savePCDFileASCII ("room_scan2_transformed_icp.pcd", *icp_result);

  // 设置ndt得到的初始对准估计结果
  Eigen::Matrix4f icp_trans;
  icp_trans=icp.getFinalTransformation();

  // 初始化正态分布变换（NDT）
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

  // 设置依赖尺度NDT参数
  // 为终止条件设置最小转换差异
  ndt.setTransformationEpsilon (0.01);
  // 为More-Thuente线搜索设置最大步长
  ndt.setStepSize (0.1);
  // 设置NDT网格结构的分辨率（VoxelGridCovariance）
  ndt.setResolution (0.7);

  // 设置匹配迭代的最大次数
  ndt.setMaximumIterations (1000);

  // 设置要配准的点云
  ndt.setInputSource (filtered_cloud);
  // 设置点云配准目标
  ndt.setInputTarget (target_cloud);

  // 计算需要的刚体变换以便将输入的点云匹配到目标点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  // 开始配准
  ndt.align (*output_cloud, icp_trans);

  cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << endl;
  // 使用创建的变换对未过滤的输入点云进行变换
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

  cout<< ndt.getFinalTransformation() <<endl;

  ends = clock();
  cout<<(double)(ends-start)/CLOCKS_PER_SEC<<endl;

  Eigen::Matrix4f utmFinal = ndt.getFinalTransformation();

  UtmCoords utmm(utmFinal(0,3) + 457246, utmFinal(1,3) + 4404758, "50N");       //string can be "50S" or "50N",get the same result
  cout << "utmm.x: " << utmm.x << endl;         //x,double
  cout << "utmm.y: " << utmm.y << endl;         //y,double
  LatLonCoords lll = utmToLatLon(utmm);
  cout << "lll.lat: " << lll.lat << endl;       //lat,double,it follows precision from M_PI
  cout << "lll.lon: " << lll.lon << endl;       //lon,double,it follows precision from M_PI

  // 保存转换的输入点云
  //pcl::io::savePCDFileASCII ("room_scan2_transformed_ndt.pcd", *output_cloud);

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

  // 对转换后的目标点云着色（绿色）并可视化
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
  output_color (output_cloud, 0, 255, 0);
  viewer_final_ndt->addPointCloud<pcl::PointXYZI> (output_cloud, output_color, "output cloud");
  viewer_final_ndt->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");


  // 初始化点云可视化界面
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final_icp (new pcl::visualization::PCLVisualizer ("3D Viewer Algorithm 2"));
  viewer_final_icp->setBackgroundColor (0, 0, 0);

  // 对目标点云着色（红色）并可视化
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
  target_color_icp (target_cloud, 255, 0, 0);
  viewer_final_icp->addPointCloud<pcl::PointXYZI> (target_cloud, target_color_icp, "target cloud");
  viewer_final_icp->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // 对转换后的目标点云着色（绿色）并可视化
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
  output_color_icp (output_cloud, 0, 255, 0);
  viewer_final_icp->addPointCloud<pcl::PointXYZI> (icp_result, output_color_icp, "output cloud");
  viewer_final_icp->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // 启动可视化
  viewer_final_icp->addCoordinateSystem (1.0, "global");
  viewer_final_icp->initCameraParameters ();

  // 等待直到可视化窗口关闭。
  while ((!viewer_final_ndt->wasStopped ())&&(!viewer_final_icp->wasStopped ()))
  {
    viewer_final_ndt->spinOnce (100);
    viewer_final_icp->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}
