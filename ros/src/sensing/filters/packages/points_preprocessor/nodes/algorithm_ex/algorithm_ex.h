#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <sensor_msgs/LaserScan.h>
//#include "typedef.h"
#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
//#include "colors.h"
#include "puck.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
 //#include <geometry_msgs/Twist.h>

//#include <array>


//#include <boost/thread/mutex.hpp>
//#include <boost/thread/lock_guard.hpp>

//#include <dynamic_reconfigure/server.h>
//#include <velodyne_laserscan/VelodyneLaserScanConfig.h>


// Global variables
ros::Publisher g_pub;
ros::Publisher g_pub2;

ros::Subscriber g_sub;
ros::Subscriber g_sub2;
sensor_msgs::LaserScan g_scan;
volatile bool g_scan_new = false;

void recv(const sensor_msgs::LaserScanConstPtr& msg) {
  g_scan = *msg;
  g_scan_new = true;
}
void recvgridmap(const nav_msgs::GridCells& msg)
{
  ROS_INFO("Gridmap received \n");
}

//}
