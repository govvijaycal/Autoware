#include "algorithm_ex.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Puckalgorithm");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");


  ros::Rate r(100);

    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
  {

//    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
//            ros::Time::now(),"base_link", "velodyne"));
//    r.sleep();

    //ros::Subscriber sub = node.subscribe("velodyne_points", 1000, messageCallback);
    //g_pub = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 2);
    g_pub = nh.advertise<sensor_msgs::PointCloud2>("/rear_lidar/points_raw", 2);
    g_pub2 = nh.advertise<sensor_msgs::PointCloud2>("/center_lidar/points_raw", 2);

    g_sub = nh.subscribe("scan", 2, recv);
    g_sub2 = nh.subscribe("map", 2, recvgridmap);


    velodyne_pointcloud::PuckAlgorithm puck(nh, priv_nh);

    // velodyne_pointcloud::PuckAlgorithm puck(nh, priv_nh);

    // handle callbacks until shut down
    ros::spin();
    //  loop_rate.sleep(); // Sleep for the rest of the cycle
    //  count++;


  }
  return 0;
}

