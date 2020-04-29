#include <ros/ros.h>
#include "hmoving.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hexy_moving");
  ros::NodeHandle nh;
  hmoving hmov = hmoving(&nh);
  ros::Rate rate(40);

  ROS_INFO("Hexy moving started");

  while(ros::ok())
  {
    hmov.refresh_data();
    rate.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Hexy moving finished");
  return 0;
}
