#include "ros/ros.h"
#include "vicon_bridge/Marker.h"
#include "vicon_bridge/Markers.h"


void markersCallback(const vicon_bridge::Markers::ConstPtr& msg)
{
  ROS_INFO("Received Markers");
  // for (const vicon_bridge::Marker& marker : msg->markers) {
  for (size_t i = 0; i < msg->markers.size(); ++i) {
    const vicon_bridge::Marker& marker = msg->markers[i];
    if (marker.subject_name == "cf_config1") {
      ROS_INFO("[%f,%f,%f]",
        marker.translation.x / 1000.0,
        marker.translation.y / 1000.0,
        marker.translation.z / 1000.0);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "configuration");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/vicon/markers", 1, markersCallback);

  ros::spin();

  return 0;
}
