#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>

void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr pointCloud)
{
  ROS_INFO("Received Markers");

  for(size_t i = 0; i < pointCloud->points.size(); ++i) {
    ROS_INFO("[%f,%f,%f]",
      pointCloud->points[i].x / 1000.0,
      pointCloud->points[i].y / 1000.0,
      pointCloud->points[i].z / 1000.0);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "configuration");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/vicon/pointCloud", 1, pointCloudCallback);

  ros::spin();

  return 0;
}
