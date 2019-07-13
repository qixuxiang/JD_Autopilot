#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "load_pcd_map/JDMap.h"
#include "load_pcd_map/JDMetaMap.h"
//Custom Point format,PointXYZIL
//I:intensity; L:label,0 :ground points, 1:non-ground points
struct EIGEN_ALIGN16 PointXYZIL
{
    PCL_ADD_POINT4D;
    union
    {
        struct
        {
            float intensity;
            unsigned int label;
        };
        float data_c[4];
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (unsigned int, label, label))


ros::Publisher point_cloud_pub;
double origin_x(0.),origin_y(0.);
pcl::PointCloud<PointXYZIL>::Ptr cloud_map(new(pcl::PointCloud<PointXYZIL>));

//Publish PointCloud Map,which you can use with lidar to do localization
void PointCloudPublish(const pcl::PointCloud<PointXYZIL>& points) {
    sensor_msgs::PointCloud2 ros_points;
    pcl::toROSMsg(points,ros_points);
    ros_points.header.stamp = ros::Time::now();
    ros_points.header.frame_id = "map";
    point_cloud_pub.publish(ros_points);
    ROS_INFO("Test pointcloud map publish!");
}
//Subscribe map list topic
void MapListCb(const load_pcd_map::JDMapConstPtr& map_list_) {
    ROS_INFO("Receive map_list topic...");
    origin_x = map_list_->originX;
    origin_y = map_list_->originY;

    ROS_INFO("Get origin(%f,%f)",origin_x,origin_y);
    for(int i=0;i<map_list_->maps.size();i++)
    {
        pcl::PointCloud<PointXYZIL>::Ptr cloud_tile(new(pcl::PointCloud<PointXYZIL>));
        std::string map_name = map_list_->maps[i].map_name;
        if (pcl::io::loadPCDFile<PointXYZIL> (map_name.c_str(), *cloud_tile) == -1)
        {
            ROS_ERROR("Couldn't read %s file \n",map_name.c_str());
            continue;
        }
        for(int j=0;j<cloud_tile->size();j++)
        {
            PointXYZIL point = cloud_tile->points[j];
            cloud_map->push_back(point);
        }
    }
    if(cloud_map->empty())
    {
        ROS_ERROR("Cannot open any pcd file");
        return;
    }
    //cloud_map is the pointcloud map ptr, you can use it with lidar data to do localization
    PointCloudPublish(*cloud_map);
}

int main(int argc,char** argv) {
    ros::init(argc,argv,"load_pcd_map_node");
    ROS_INFO("load_pcd_map_node start...");
    ros::NodeHandle nh;
    ros::Subscriber map_list_sub = nh.subscribe("map_list",1,&MapListCb);
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud_map",1,true);
    ros::spin();
    return 0;
}
