/*****************************************************************************
*@文件:      omp_ndt:地图匹配
*@说明：     采用omp_ndt实现地图匹配
*@日期：     2018.11.25
*@修订者:    zps & qxx
*@修订日期:  2018.12.14
*@原始代码： https://github.com/BurryChen/ndt_omp
*****************************************************************************/
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/icp.h>


#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <ctime>
#include <angles/angles.h>
#include <global_coords.h>
#include <gicp_omp.h>
#include <ndt_omp.h>


using namespace std;
using namespace global_coords;
using namespace pclomp;

void ndtMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr targetCloud,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud,
                  LatLonCoords inLatLon,double inAngel,
                  LatLonCoords *outLatLon,double *outAngle)
{
  ros::Time::init();
  auto t1 = ros::WallTime::now();
  vector<int> indices_src; //保存去除的点的索引
  pcl::removeNaNFromPointCloud(*inputCloud,*inputCloud, indices_src);
  UtmCoords utm = latLonToUtm(inLatLon);             //convert to utm
  float angle=3.9;
  // 设置使用机器人测距法得到的初始对准估计结果
  Eigen::AngleAxisf init_rotation (angle, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (utm.x - 457246 , utm.y - 4404758, 21);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  // 滤波
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(0.5, 0.5, 0.5);

  auto t2 = ros::WallTime::now();
  std::cout << "single Time : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

  voxelgrid.setInputCloud(inputCloud);
  voxelgrid.filter(*downsampled);
  inputCloud = downsampled;
  std::cout<<"target size:"<<targetCloud->size()<<" source size:"<<inputCloud->size()<<std::endl;

  t2 = ros::WallTime::now();
  std::cout << "single Time : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
  ndt_omp->setResolution(0.7);
  ndt_omp->setStepSize(0.2);
  ndt_omp->setTransformationEpsilon(0.01);
  ndt_omp->setMaximumIterations (200);
  ndt_omp->setInputTarget(targetCloud);
  ndt_omp->setInputSource(inputCloud);
  ndt_omp->setNumThreads(12);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  pcl::PointCloud<pcl::PointXYZI>::Ptr matched(new pcl::PointCloud<pcl::PointXYZI>());
  ndt_omp->align(*matched,init_guess);

  t2 = ros::WallTime::now();
  std::cout << "single Time : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

  std::cout<<"Transform: \n"<<ndt_omp->getFinalTransformation()<<std::endl;

  Eigen::Matrix4f utmFinal = ndt_omp->getFinalTransformation();

  UtmCoords utmm(utmFinal(0,3) + 457246, utmFinal(1,3) + 4404758, "50N");       //string can be "50S" or "50N",get the same result
  //cout << "utmm.x: " << utmm.x << endl;         //x,double
  //cout << "utmm.y: " << utmm.y << endl;         //y,double
  //LatLonCoords lll = utmToLatLon(utmm);
  //cout << "lll.lat: " << lll.lat << endl;       //lat,double,it follows precision from M_PI
  //cout << "lll.lon: " << lll.lon << endl;       //lon,double,it follows precision from M_PI
  *outLatLon = utmToLatLon(utmm);
  t2 = ros::WallTime::now();
  std::cout << "single Time : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr merageMap(new pcl::PointCloud<pcl::PointXYZI>());
  *merageMap = *targetCloud + *matched;
  //可视化
  pcl::visualization::PCLVisualizer vis("vis");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> target_handler(targetCloud, 255.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> source_handler(inputCloud, 0.0, 255.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> matched_handler(matched, 216.0, 200.0, 255.0);
  vis.addPointCloud(targetCloud, target_handler, "target");
  vis.addPointCloud(inputCloud, source_handler, "source");
  vis.addPointCloud(matched, matched_handler, "matched");

  pcl::visualization::PCLVisualizer vism("merage");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> merage_handler(merageMap, 255.0, 0.0, 0.0);
  vism.addPointCloud(merageMap, merage_handler, "merageMap");

  vis.spin();
  vism.spin();
}

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
  // 地图PCD滤波
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(0.5, 0.5, 0.5);
  voxelgrid.setInputCloud(target_cloud);
  voxelgrid.filter(*downsampled);
  *target_cloud = *downsampled;

  //初始化无人车在地图的初始位置39.7917338786 116.498618594  39.791748,116.498636
  LatLonCoords latlon(39.791748,116.498636);       //initial latitude and longitude
  double finalAngel;
  LatLonCoords finalLatLon;
  ndtMatching(target_cloud,input_cloud,latlon,-144,&finalLatLon,&finalAngel);
  cout << "finalLatLon.lat: " << finalLatLon.lat << endl;       //lat,double,it follows precision from M_PI
  cout << "finalLatLon.lon: " << finalLatLon.lon << endl;       //lon,double,it follows precision from M_PI
  /*
  cout << "Loaded " << input_cloud->size () << " data points from CarLidar.pcd" << endl;
  vector<int> indices_src; //保存去除的点的索引
  pcl::removeNaNFromPointCloud(*input_cloud,*input_cloud, indices_src);
  cout<<"remove *input_cloud nan"<<endl;

  //初始化无人车在地图的初始位置39.7917338786 116.498618594  39.791748,116.498636
  LatLonCoords latlon(39.791748,116.498636);       //initial latitude and longitude
  UtmCoords utm = latLonToUtm(latlon);             //convert to utm

  float angle=3.9;
  // 设置使用机器人测距法得到的初始对准估计结果
  Eigen::AngleAxisf init_rotation (angle, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (utm.x - 457246 , utm.y - 4404758, 21);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();


  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(0.5, 0.5, 0.5);

  voxelgrid.setInputCloud(target_cloud);
  voxelgrid.filter(*downsampled);
  *target_cloud = *downsampled;

  voxelgrid.setInputCloud(input_cloud);
  voxelgrid.filter(*downsampled);
  input_cloud = downsampled;
  std::cout<<"target size:"<<target_cloud->size()<<" source size:"<<input_cloud->size()<<std::endl;

  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
  ndt_omp->setResolution(0.7);
  ndt_omp->setStepSize(0.2);
  ndt_omp->setTransformationEpsilon(0.01);
  ndt_omp->setMaximumIterations (200);
  ndt_omp->setInputTarget(target_cloud);
  ndt_omp->setInputSource(input_cloud);
  ndt_omp->setNumThreads(8);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  pcl::PointCloud<pcl::PointXYZI>::Ptr matched(new pcl::PointCloud<pcl::PointXYZI>());
  ros::Time::init();
  auto t1 = ros::WallTime::now();
  ndt_omp->align(*matched,init_guess);
  auto t2 = ros::WallTime::now();
  std::cout << "single Time : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

  std::cout<<"Transform: \n"<<ndt_omp->getFinalTransformation()<<std::endl;

  Eigen::Matrix4f utmFinal = ndt_omp->getFinalTransformation();

  UtmCoords utmm(utmFinal(0,3) + 457246, utmFinal(1,3) + 4404758, "50N");       //string can be "50S" or "50N",get the same result
  cout << "utmm.x: " << utmm.x << endl;         //x,double
  cout << "utmm.y: " << utmm.y << endl;         //y,double
  LatLonCoords lll = utmToLatLon(utmm);
  cout << "lll.lat: " << lll.lat << endl;       //lat,double,it follows precision from M_PI
  cout << "lll.lon: " << lll.lon << endl;       //lon,double,it follows precision from M_PI

  pcl::visualization::PCLVisualizer vis("vis");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> target_handler(target_cloud, 255.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> source_handler(input_cloud, 0.0, 255.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> matched_handler(matched, 216.0, 200.0, 255.0);
  vis.addPointCloud(target_cloud, target_handler, "target");
  vis.addPointCloud(input_cloud, source_handler, "source");
  vis.addPointCloud(matched, matched_handler, "matched");
  vis.spin();
  */

  return (0);
}
