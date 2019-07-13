/****************************************************************************
*@文件:      listenSensorsTopic:话题订阅
*@说明：
*@日期：     2018.11.26
*@修订者:    zps & zxp & qxx
*@修订日期:  2018.12.14
*****************************************************************************/
#include <iostream>
#include <cstdio>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sys/time.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <angles/angles.h>
#include <global_coords.h>

using namespace std;
using namespace global_coords;

double calculateDeltaTime(ros::Time thisTime , ros::Time lastTime);


#define FIX_PATH "/home/airobot/JD_semifinal/result/data/fix_pre_1130.txt"
FILE *fixFp = NULL;  //文件指针
#define IMU_PATH "/home/airobot/JD_semifinal/result/data/imu_pre_1130.txt"
FILE *imuFp = NULL;  //文件指针
#define ODOM_PATH "/home/airobot/JD_semifinal/result/data/odom_pre_1130.txt"
FILE *odomFp = NULL; //文件指针

ros::Time latestStamp;// 最新的时间戳

// fix数据结构体
typedef struct
{
    int32_t count;           //计数 不超过3500条 3047 3405
    ros::Time stamp[4000];   //时间戳
    uint8_t status[4000];    //定位状态
    double latitude[4000];   //纬度
    double longitude[4000];  //经度
    double altitude[4000];   //高度
    double yaw[4000];        //航向
    uint8_t satNums[4000];    //可见卫星数量
} fixStruct;
fixStruct fixData;
// odom里程计数据
typedef struct
{
    int32_t count;           //计数 不超过3500条 3047 3405
    ros::Time stamp[40000];  //时间戳
    double speed[40000];     //前向速度
} odomStruct;
odomStruct odomData;
// IMU数据
typedef struct
{
    int32_t count;            //计数 不超过3500条 3047 3405
    ros::Time stamp[75000];   //时间戳
    double gyro[75000][3];    //陀螺仪角速度  三轴陀螺仪输出（°/s），x轴朝左，y轴朝前，z轴朝下
    double accel[75000][3];   //加速度       三轴加速度输出(m/s2)，x轴朝左，y轴朝前，z轴朝下
    double deltaYaw;
    double deltaYawData[75000];
    double deltaAngle;
    double deltaAngleData[75000];
} imuStruct;
imuStruct imuData;
// 获取里程计数据
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    latestStamp = msg->header.stamp;
    odomData.stamp[odomData.count] = msg->header.stamp;
    odomData.speed[odomData.count] = msg->twist.twist.linear.x;
    odomData.count++;
}
// 获取IMU数据
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(imuData.count < 2)
    {
        imuData.deltaYaw = 0;
        imuData.deltaAngle = 0;
    }
    static ros::Time lastTime;
    ros::Time thisTime = msg->header.stamp;
    latestStamp = thisTime;
    imuData.stamp[imuData.count] = msg->header.stamp;
    imuData.accel[imuData.count][0] = msg->linear_acceleration.x;
    imuData.accel[imuData.count][1] = msg->linear_acceleration.y;
    imuData.accel[imuData.count][2] = msg->linear_acceleration.z;
    imuData.gyro[imuData.count][0] = msg->angular_velocity.x;
    imuData.gyro[imuData.count][1] = msg->angular_velocity.y;
    imuData.gyro[imuData.count][2] = msg->angular_velocity.z;
    imuData.deltaYaw +=  (imuData.gyro[imuData.count][2] * calculateDeltaTime(thisTime,lastTime));
    imuData.deltaYawData[imuData.count] = imuData.deltaYaw;
    imuData.deltaAngle +=  imuData.gyro[imuData.count][2]/ M_PI * 180.0 * (calculateDeltaTime(thisTime,lastTime));
    imuData.deltaAngleData[imuData.count] = imuData.deltaAngle;
    ROS_INFO("deltaTime = [%lf]",calculateDeltaTime(latestStamp,lastTime));
    ROS_INFO("deltaYaw = [%lf]",imuData.deltaYaw);
    ROS_INFO("deltaAng = [%lf]",imuData.deltaAngle);
    imuData.count++;
    lastTime = thisTime;
}
// 获取FIX数据
void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    latestStamp = msg->header.stamp;
    fixData.stamp[fixData.count] = msg->header.stamp;
    fixData.latitude[fixData.count] = msg->latitude;
    fixData.longitude[fixData.count] = msg->longitude;
    fixData.yaw[fixData.count] = msg->position_covariance[0];
    fixData.status[fixData.count] = msg->status.status;
    fixData.satNums[fixData.count] = msg->position_covariance_type;
    fixData.count ++;
}
// 获取LIDAR数据
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

}

int main(int argc, char **argv)
{
  // test code begins  here

  cout << "M_PI IN C++" << setprecision(20) << M_PI <<endl; // pi in C++
  //setprecision for the decimal precision, you need only set it once,all variables will follow it
  LatLonCoords latlon(39.7936580439358, 116.49891977436);  //initial latitude and longitude
  UtmCoords utm = latLonToUtm(latlon);         //convert to utm a first time
  cout << "utm.x: " << utm.x << endl;          //x,double, it follows precision from M_PI
  cout << "utm.y: " << utm.y << endl;          //y,double,it follows precision from M_PI
  //cout << "utm.zone: " << utm.zone<< endl;   //zone, string(char array in C)
  LatLonCoords ll = utmToLatLon(utm);          //convert back to lat lon
  cout << "ll.lat: " << ll.lat << endl;        //lat,double,it follows precision from M_PI
  cout << "ll.lon: " << ll.lon << endl;        //lon,double,it follows precision from M_PI


  UtmCoords utmm(457246, 4404758, "50N");       //string can be "50S" or "50N",get the same result
  cout << "utmm.x: " << utmm.x << endl;         //x,double
  cout << "utmm.y: " << utmm.y << endl;         //y,double
  LatLonCoords lll = utmToLatLon(utmm);
  cout << "lll.lat: " << lll.lat << endl;       //lat,double,it follows precision from M_PI
  cout << "lll.lon: " << lll.lon << endl;       //lon,double,it follows precision from M_PI

  //test code ends  here.
  ros::init(argc, argv, "listenSensorsTopic");
  fixFp = fopen(FIX_PATH, "a");
  imuFp = fopen(IMU_PATH, "a");
  odomFp = fopen(ODOM_PATH, "a");

  if((NULL == fixFp)||(NULL == imuFp)||(NULL == odomFp))
  {
      return -1; //返回错误代码
  }
  fixData.count = 0;
  imuData.count = 0;
  odomData.count = 0;
  imuData.deltaYaw = 0;
  imuData.deltaAngle = 0;
  ROS_INFO("listenSensorTopic Running start");
  ros::NodeHandle n;
  // 消息订阅
  ros::Subscriber subOdom = n.subscribe("odom", 1000, odometryCallback);
  ros::Subscriber subImu = n.subscribe("compensated_imu", 1000, imuCallback);
  ros::Subscriber subFix = n.subscribe("fix", 1000, fixCallback);
  ros::Subscriber subLidar = n.subscribe("lidar_points", 1000, lidarCallback);
  ros::Rate r(500); // 500HZ
  uint8_t rosRun = 1;
  int64_t count = 0;
  int64_t endcount = 0;
  while (rosRun)
  {
        if(count < 2)
        {
            fixData.count = 0;
            imuData.count = 0;
            odomData.count = 0;
            imuData.deltaYaw = 0;
            imuData.deltaAngle = 0;
        }
        if((int32_t)latestStamp.sec - 1542356961 > 0)//1542356962 1542353956
        {
            endcount++;
            ROS_INFO("rostopic received success");
            if(endcount > 9999)
            {
                rosRun = 0;
            }
        }
        ros::spinOnce();
        r.sleep();
        if(count%5000 == 0)
            ROS_INFO("Count = [%d] , Time = [%d]",(int)count/5000,latestStamp.sec);
        count++;
  }
  ROS_INFO("write file start");
  for(int i=0;i<fixData.count;i++)
  {
        fprintf(fixFp, "%d.%d,%lf,%lf,%lf,%d,%d\n",
        fixData.stamp[i].sec,
        fixData.stamp[i].nsec,
        fixData.latitude[i],
        fixData.longitude[i],
        fixData.yaw[i],
        fixData.status[i],
        fixData.satNums[i]);
  }
  for(int i=0;i<imuData.count;i++)
  {
        fprintf(imuFp, "%d.%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
        imuData.stamp[i].sec,
        imuData.stamp[i].nsec,
        imuData.gyro[i][0],
        imuData.gyro[i][1],
        imuData.gyro[i][2],
        imuData.accel[i][0],
        imuData.accel[i][1],
        imuData.accel[i][2],
        imuData.deltaYawData[i],
        imuData.deltaAngleData[i]);

  }
  for(int i=0;i<odomData.count;i++)
  {
      fprintf(odomFp, "%d.%d,%lf\n",
      odomData.stamp[i].sec,
      odomData.stamp[i].nsec,
      odomData.speed[i]);
  }
  ROS_INFO("end end end");
  fclose(fixFp);
  fclose(imuFp);
  fclose(odomFp);
  fixFp = imuFp = odomFp = NULL; //需要指向空，否则会指向原打开文件地址
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
