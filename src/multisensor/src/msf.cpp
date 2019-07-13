/****************************************************************************
*@文件:      msf:实时多传感器融合
*@说明：
*@日期：     2018.11.30
*@修订者:    zps & qxx
*@修订日期:  2018.12.14
*****************************************************************************/
#include <iostream>
#include <cstdio>
#include <vector>
#include <fstream>
#include <cstring>
#include <string>
#include <sstream>
#define _USE_MATH_DEFINES
#include <cmath>
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sys/time.h>
#include <angles/angles.h>
#include <global_coords.h>

#include <pcl_conversions/pcl_conversions.h>

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
#include <gicp_omp.h>
#include <ndt_omp.h>

using namespace std;
using namespace global_coords;

void ndtMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr targetCloud,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud,
                  LatLonCoords inLatLon,double inAngel,
                  LatLonCoords *outLatLon,double *outAngle);

double calculateDeltaTime(ros::Time thisTime , ros::Time lastTime);
double GpsCalcPositionChanged(double lat1,double lon1,double lat2,double lon2);
void ConvertDistanceToLogLat(double lat1,double lon1,double distance,double azimuth,double *lat2,double *lon2);

#define RESULT_PATH "/home/airobot/JD_semifinal/result/data/result_201812132330.csv"
#define LIDAR_PATH "/home/airobot/JD_semifinal/result/data/lidar_201812132330.csv"

ros::Time latestStamp;// 最新的时间戳
pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZI>);//地图PCD
ros::CallbackQueue lidar_callback_queue;//雷达队列
double lidarHight = 21;
int init_state_flag = 0;
// fix数据结构体
typedef struct
{
    int32_t count;     //计数 不超过3500条 3047 3405
    ros::Time stamp;   //时间戳
    uint8_t status;    //定位状态 0-无GPS信号，1-GPS单点定位，2-GPS伪距差分定位，4-RTK定点解，5-RTK浮点解
    double latitude;   //纬度
    double longitude;  //经度
    double altitude;   //高度
    double yaw;        //航向
    double satNums;    //可见卫星数量
} fixStruct;
fixStruct fixData;
// odom里程计数据
typedef struct
{
    int32_t count;    //计数 不超过3500条 3047 3405
    ros::Time stamp;  //时间戳
    double speed;     //前向速度
    double deltaDistance;//速度积分
} odomStruct;
odomStruct odomData;
// IMU数据
typedef struct
{
    int32_t count;     //计数 不超过3500条 3047 3405
    ros::Time stamp;   //时间戳
    double gyro[3];    //陀螺仪角速度  三轴陀螺仪输出（°/s），x轴朝左，y轴朝前，z轴朝下
    double accel[3];   //加速度       三轴加速度输出(m/s2)，x轴朝左，y轴朝前，z轴朝下
    double deltaYaw;   //角速度积分
} imuStruct;
imuStruct imuData;
// 融合数据输出
typedef struct
{
    int32_t count;     //计数
    ros::Time stampDatas[20000];   //时间戳
    double latitudeDatas[20000];   //纬度
    double longitudeDatas[20000];  //经度
    double yawDatas[20000];        //航向
    double latitude;
    double longitude;
    double yaw;
    double latTrue;//矫正后的
    double lonTrue;
} msfStruct;
msfStruct msfData;
// 雷达数据输出
typedef struct
{
    int32_t count;     //计数
    ros::Time stampDatas[20000];   //时间戳
    double latitudeDatas[20000];   //纬度
    double longitudeDatas[20000];  //经度
    double yawDatas[20000];        //航向
    double latitude;
    double longitude;
    double yaw;
    int tansState;
    // 设置ndt得到的初始对准估计结果
    Eigen::Matrix4f ndt_trans;
} lidarStruct;
lidarStruct lidarData;
// 获取里程计数据
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(odomData.count<2)
    {
        odomData.deltaDistance = 0;
    }
    static ros::Time lastTime;
    latestStamp = msg->header.stamp;
    odomData.stamp= msg->header.stamp;
    //平滑滤波
    odomData.speed = 0.0 * odomData.speed + 1 * (msg->twist.twist.linear.x * 100);
    odomData.deltaDistance +=  odomData.speed * calculateDeltaTime(latestStamp,lastTime);
    // 数据融合 里程计频率为50HZ 这里融合算法统一为25HZ
    if((msfData.count<5)||(fixData.count<5))
    {
        msfData.yaw = fixData.yaw;
        msfData.latitude = fixData.latitude;
        msfData.longitude = fixData.longitude;
        lidarData.latitude = fixData.latitude;
        lidarData.longitude = fixData.longitude;
	msfData.latTrue = fixData.latitude;
	msfData.lonTrue = fixData.longitude;
    }
    if(odomData.count % 2 ==0)
    {
        msfData.stampDatas[msfData.count] = latestStamp;
        // 航向一阶互补滤波
        if(abs(fixData.yaw)<179)
        {
            if((fixData.status >= 4)&&(abs(fixData.yaw)>0.5))//RTK 精度高 GPS原始数据航向为0 代表异常，不参与互补滤波
            {
                msfData.yaw = 0.996 * (msfData.yaw + imuData.deltaYaw) + 0.004 * fixData.yaw ;
            }
            else
            {
                msfData.yaw = msfData.yaw + imuData.deltaYaw;
            }
        }
        else // +- 180跳变 需要快速收敛
        {
            msfData.yaw = fixData.yaw ;
        }
        if(msfData.yaw > 180) msfData.yaw -= 360;
        if(msfData.yaw <-180) msfData.yaw += 360;
        imuData.deltaYaw =0;//清空积分
        // 经纬度一阶互补滤波
        double lat,lon,lat1,lon1,lat2,lon2,angle1;;
        //ROS_INFO("deltaDistance = [%lf]",odomData.deltaDistance);
        //轮速计融合
        ConvertDistanceToLogLat(msfData.latitude,
                                msfData.longitude,
                                odomData.deltaDistance,
                                msfData.yaw,
                                &lat,
                                &lon);
        // 判断GPS状态 差分定位并且航向解算正常
        if((fixData.status >= 4)&&(abs(fixData.yaw)>0.5))//RTK 精度高
        {
            msfData.latitude  = 0.95 * lat + 0.05 * fixData.latitude;
            msfData.longitude = 0.95 * lon + 0.05 * fixData.longitude;
        }
        else//惯导融合
        {
            msfData.latitude  = lat;
            msfData.longitude = lon;
        }
        ROS_INFO("Time = [%d].[%d]",latestStamp.sec,latestStamp.nsec);
        //ROS_INFO("msf.lat = [%lf] , msf.lon = [%lf]",msfData.latitude,msfData.longitude);
        // 转换为坐标原点GPS坐标 后车轮轴线中间
        angle1 = msfData.yaw + 80.51570345;
        if(angle1>180) angle1-=360;
        ConvertDistanceToLogLat(msfData.latitude,
                                msfData.longitude,
                                31.4,
                                angle1,
                                &lat1,
                                &lon1);

        // 地图匹配的经纬度 惯性融合
        ConvertDistanceToLogLat(msfData.latTrue,
                         msfData.lonTrue,
                        odomData.deltaDistance,
                        msfData.yaw,
                        &lat2,
                        &lon2);
        // 计算融合的经纬度 与 地图匹配的经纬度距离误差
        if(lidarData.tansState == 1)
        {
            msfData.latTrue = 0.12 * lidarData.latitude + 0.88 * msfData.latTrue;
            msfData.lonTrue = 0.12 * lidarData.longitude + 0.88 * msfData.lonTrue;
            lidarData.tansState = 2;
            ROS_INFO("lidarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr correct");
        }
        else if(lidarData.tansState > 1)
        {
            msfData.latTrue = 0.05 * lat1 + 0.90 * lat2 + 0.05 * msfData.latTrue;
            msfData.lonTrue = 0.05 * lon1 + 0.90 * lon2 + 0.05 * msfData.lonTrue;
            ROS_INFO("lidarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr correcting");
        }
        else
        {
            msfData.latTrue = 0.5 * lat1 + 0.5 * lat2;
            msfData.lonTrue = 0.5 * lon1 + 0.5 * lon2;
        }
        msfData.latitudeDatas[msfData.count] =  msfData.latTrue;//msfData.latitude;
        msfData.longitudeDatas[msfData.count] =   msfData.lonTrue;//msfData.longitude;
        msfData.yawDatas[msfData.count] = msfData.yaw;
        ROS_INFO("msf.lat = [%lf] , msf.lon = [%lf]",msfData.latTrue,msfData.lonTrue);
        odomData.deltaDistance = 0;//清空积分
        msfData.count ++;
    }
    lastTime = latestStamp;
    odomData.count++;
}
// 获取IMU数据
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(imuData.count < 2)
    {
        imuData.deltaYaw = 0;
    }
    static ros::Time lastTime;
    latestStamp = msg->header.stamp;
    imuData.gyro[2] = msg->angular_velocity.z;
    imuData.deltaYaw +=  (57.29577951 * imuData.gyro[2] * calculateDeltaTime(latestStamp,lastTime));
    lastTime = latestStamp;
    imuData.count++;
}
// 获取FIX数据
void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    latestStamp = msg->header.stamp;
    fixData.stamp = msg->header.stamp;
    fixData.status = msg->status.status;
    fixData.latitude = msg->latitude;
    fixData.longitude = msg->longitude;
    fixData.yaw = msg->position_covariance[0];
    fixData.satNums = msg->position_covariance_type;

    fixData.count ++;
}
// 获取LIDAR数据
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    static int lcount = 0;
    latestStamp = msg->header.stamp;
    lidarData.stampDatas[lidarData.count] = latestStamp;
    if(lcount < 5)
    {
        lidarData.latitude = fixData.latitude;
        lidarData.longitude = fixData.longitude;
    }
    if(lcount % 8 == 0)
    {
        sensor_msgs::PointCloud2 output;  //ROS中点云的数据格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        //pcl::PointCloud<pcl::PointXYZI> cloud; // With color
        output = *msg;
        pcl::fromROSMsg(output,*cloud);

        double finalAngel = 0;
        LatLonCoords finalLatLon;
        //初始化无人车在地图的初始位置39.7917338786 116.498618594  39.791748,116.498636
        LatLonCoords latlon(msfData.latTrue,msfData.lonTrue);       //initial latitude and longitude
        double angle=0;
        if(msfData.yaw < 0) angle = 90 - msfData.yaw;
        else if(msfData.yaw < 90)
        {
            angle = 90 - msfData.yaw;
        }
        else if(msfData.yaw >= 90)
        {
            angle = 270 + 180 - msfData.yaw;
        }
        // 匹配
        ndtMatching(target_cloud,cloud,latlon,angle,&finalLatLon,&finalAngel);
        ROS_INFO("lidarTime = [%d].[%d]",msg->header.stamp.sec,msg->header.stamp.nsec);
        cout << "finalLatLon.lat: " << finalLatLon.lat << endl;       //lat,double,it follows precision from M_PI
        cout << "finalLatLon.lon: " << finalLatLon.lon << endl;       //lon,double,it follows precision from M_PI

	double distanceError = GpsCalcPositionChanged(latlon.lat,latlon.lon,finalLatLon.lat,finalLatLon.lon);

	if(abs(distanceError) > 126) lidarData.tansState = 0;
	else lidarData.tansState = 1;

        float angle1;
        // 转换为坐标原点GPS坐标 后车轮轴线中间
	if(lidarData.tansState == 1)
	{
		angle1 = -msfData.yaw;
		ConvertDistanceToLogLat(finalLatLon.lat,
		                        finalLatLon.lon,
		                        82.5,
		                        angle1,
		                        &lidarData.latitude,
		                        &lidarData.longitude);
	}
		lidarData.latitudeDatas[lidarData.count] = lidarData.latitude;
		lidarData.longitudeDatas[lidarData.count] = lidarData.longitude;
		lidarData.yawDatas[lidarData.count] = msfData.yaw;
   	        lidarData.count++;
    }
    lcount++;
}

int main(int argc, char **argv)
{
  cout << "M_PI IN C++" << setprecision(20) << M_PI <<endl; // pi in C++, setprecision
  // 系统初始化
  ros::init(argc, argv, "listenSensorsTopic");
  ofstream outFile;
  outFile.open(RESULT_PATH, ios::out); // 打开模式可省略
  if(!outFile)                        //如果打开失败，outFile返回0值
  {
        cerr<<"open error!"<<endl;
        exit(1);
  }
  ofstream outFileLidar;
  outFileLidar.open(LIDAR_PATH, ios::out); // 打开模式可省略
  if(!outFileLidar)                        //如果打开失败，outFile返回0值
  {
        cerr<<"open error!"<<endl;
        exit(1);
  }
  // 加载地图点云
  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/airobot/1543669258343287.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file Map.pcd \n");
    return (-1);
  }
  // 地图PCD滤波
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(0.5, 0.5, 0.5);
  voxelgrid.setInputCloud(target_cloud);
  voxelgrid.filter(*downsampled);
  *target_cloud = *downsampled;

  ROS_INFO("listenSensorTopic Running start");
  ros::NodeHandle n;
  // 消息订阅
  ros::Subscriber subOdom = n.subscribe("odom", 1000, odometryCallback);
  ros::Subscriber subImu = n.subscribe("compensated_imu", 1000, imuCallback);
  ros::Subscriber subFix = n.subscribe("fix", 1000, fixCallback);
  // ros::Subscriber subLidar = n.subscribe("lidar_points", 1000, lidarCallback);
  fixData.count = 0;
  imuData.count = 0;
  odomData.count = 0;
  lidarData.count = 0;
  // 多线程
  ros::SubscribeOptions ops=ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
                "lidar_points",
                1,
                lidarCallback,
                ros::VoidPtr(),
                &lidar_callback_queue
                );
  ros::Subscriber listenLidar_state=n.subscribe(ops);
  ros::AsyncSpinner state_spinner(6,&lidar_callback_queue);

  // Topic 接收消息
  ros::Rate r(500); // 500HZ
  uint8_t rosRun = 1;
  int64_t count = 0;
  int64_t endcount = 0;
  while (rosRun)
  {
        if((int32_t)latestStamp.sec - 1542356961 > 0)//1542356961 1542353956
        {
            endcount++;
            ROS_INFO("rostopic received success");
            if(endcount > 1000)
            {
                rosRun = 0;
            }
            ROS_INFO("Count = [%d] , Time = [%d]",(int)msfData.count-1,latestStamp.sec);
        }
        if((int32_t)msfData.stampDatas[msfData.count].sec - 1542356961 > 0)
        {
            endcount++;
            ROS_INFO("rostopic received success");
            if(endcount > 2)
            {
                rosRun = 0;
            }
        }
        ros::spinOnce();
        r.sleep();

        if(count%9000 == 0)
        {
            ROS_INFO("Count = [%d] , Time = [%d]",(int)msfData.count-1,latestStamp.sec);
            //ROS_INFO("fix.lon = [%lf] , msf.lon = [%lf]",fixData.longitude,msfData.longitude);
            //ROS_INFO("msf.lat = [%lf] , msf.lon = [%lf]",msfData.latitudeDatas[msfData.count-1],msfData.longitudeDatas[msfData.count-1]);
            //ROS_INFO("fix.yaw = [%lf] , msf.yaw = [%lf]",fixData.yaw,msfData.yaw);
        }
        if(init_state_flag == 1)
        {
            state_spinner.stop();
            init_state_flag = 0;
        }
        else
        {
            state_spinner.start();
        }
        count++;
  }
  ROS_INFO("write result file start");
  for(int i=0;i<msfData.count;i++)
  {
        outFile <<fixed<<setprecision(10)<< msfData.stampDatas[i].sec << '.' << msfData.stampDatas[i].nsec\
        << ',' << msfData.latitudeDatas[i] << ',' << msfData.longitudeDatas[i]\
        << ',' << msfData.yawDatas[i] << endl;
  }
  for(int i=0;i<lidarData.count;i++)
  {
        outFileLidar <<fixed<<setprecision(10)<< lidarData.stampDatas[i].sec << '.' << lidarData.stampDatas[i].nsec\
        << ',' << lidarData.latitudeDatas[i] << ',' << lidarData.longitudeDatas[i]\
        << ',' << lidarData.yawDatas[i] << endl;
  }
  /*
  pcl::io::savePCDFileASCII ("map.pcd", *target_cloud);
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
  // 启动可视化
  viewer_final_icp->addCoordinateSystem (1.0, "global");
  viewer_final_icp->initCameraParameters ();

  // 等待直到可视化窗口关闭。
  while ((!viewer_final_icp->wasStopped ()))
  {
    viewer_final_icp->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  */
  ROS_INFO("end end end");
  outFile.close();
  outFileLidar.close();
  return 0;
}
// 根据时间戳计算间隔时间 单位：秒
double calculateDeltaTime(ros::Time thisTime , ros::Time lastTime)
{

    double deltaSec,deltaNsec,deltaTime;
    ros::Duration delta;
    delta = thisTime - lastTime;
    deltaTime = delta.toNSec()/1000000;
    deltaTime/=1000;
    return deltaTime;
}

// 角度转弧度
double Radians(double deg)
{
    return deg*M_PI/180;
}
// 弧度转角度
double Degrees(double rad)
{
    return rad*180/M_PI;
}
// 计算两点距离
double GpsCalcPositionChanged(double lat1,double lon1,double lat2,double lon2)
{
    double rads = Radians(abs(lat1));
    double gpsLngDownScale = cos(rads);
    double deltaDis_y = (lon1 - lon2) * gpsLngDownScale * 11131950;
    double deltaDis_x = (lat1 - lat2) * 11131950;
    double distance = sqrt(deltaDis_y*deltaDis_y + deltaDis_x*deltaDis_x);
    return distance;
}
// 计算两点角度
double getDegree(double latA, double lonA, double latB, double lonB)
{
    double radLatA = Radians(latA);
    double radLonA = Radians(lonA);
    double radLatB = Radians(latB);
    double radLonB = Radians(lonB);
    double dLon = radLonB - radLonA;
    double y = sin(dLon) * cos(radLatB);
    double x = cos(radLatA) * sin(radLatB) - sin(radLatA) * cos(radLatB) * cos(dLon);
    double brng = Degrees(atan2(y, x));
    brng = ((int)brng + 360) % 360;//不知道为什么不能用浮点，这样丢失了精度
    return brng;
}
// 已知一点经纬度，方向角和距离，求另一个经纬度坐标
void ConvertDistanceToLogLat(double lat1,double lon1,double distance,double azimuth,double *lat2,double *lon2)
{
    azimuth=-azimuth;
    double EARTH_ARC = 11131950;
    azimuth = Radians(azimuth);
    azimuth += M_PI/2;
    //将距离转换成经度的计算公式
    double lon = lon1 + (distance * sin(azimuth))/ (EARTH_ARC * cos(Radians(lat1)));
    double lat = lat1 + (distance * cos(azimuth)) / EARTH_ARC;
    *lat2 = lat;
    *lon2 = lon;
    //ROS_INFO("Lon = [%10lf],Lat = [%10lf]",*lon2,*lat2);
}
// ndt
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
  float angle= Radians(inAngel);
  // 设置使用机器人测距法得到的初始对准估计结果
  Eigen::AngleAxisf init_rotation (angle, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (utm.x - 457246 , utm.y - 4404758, lidarHight);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  // 滤波
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(0.5, 0.5, 0.5);
  voxelgrid.setInputCloud(inputCloud);
  voxelgrid.filter(*downsampled);
  inputCloud = downsampled;


  // icp配准
  pcl::PointCloud<pcl::PointXYZI>::Ptr icp_result (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(inputCloud);
  icp.setInputTarget(targetCloud);
  // Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (1000);
  // 最大迭代次数
  icp.setMaximumIterations (100);
  // 两次变化矩阵之间的差值
  icp.setTransformationEpsilon (1e-10);
  // 均方误差
  icp.setEuclideanFitnessEpsilon (0.2);
  icp.align(*icp_result,init_guess);

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
  ndt.setMaximumIterations (500);

  // 设置要配准的点云
  ndt.setInputSource (inputCloud);
  // 设置点云配准目标
  ndt.setInputTarget (targetCloud);

  // 计算需要的刚体变换以便将输入的点云匹配到目标点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  // 开始配准
  ndt.align (*output_cloud, icp_trans);



  lidarData.ndt_trans = ndt.getFinalTransformation();

  Eigen::Matrix4f utmFinal = ndt.getFinalTransformation();
  UtmCoords utmm(utmFinal(0,3) + 457246, utmFinal(1,3) + 4404758, "50N");       //string can be "50S" or "50N",get the same result
  lidarHight = utmFinal(2,3);
  //cout << "utmm.x: " << utmm.x << endl;         //x,double
  //cout << "utmm.y: " << utmm.y << endl;         //y,double
  //LatLonCoords lll = utmToLatLon(utmm);
  //cout << "lll.lat: " << lll.lat << endl;       //lat,double,it follows precision from M_PI
  //cout << "lll.lon: " << lll.lon << endl;       //lon,double,it follows precision from M_PI
  *outLatLon = utmToLatLon(utmm);
  auto t2 = ros::WallTime::now();
  std::cout << "single Time : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
  //*target_cloud += *matched;
}
