#include "ros/ros.h"
#include "hexy_msgs/LegRefPos.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<hexy_msgs::LegRefPos>("leg_ref_pos", 1);

  ros::Rate loop_rate(40);

  hexy_msgs::LegRefPos msg;
  msg.x[0] = 0.2;
  msg.y[0] = 0.2;
  msg.z[0] = -0.05;

  msg.x[1] = 0.02685;
  msg.y[1] = 0.19881;
  msg.z[1] = -0.1046;

  msg.x[2] = -0.2;
  msg.y[2] = 0.2;
  msg.z[2] = -0.05;

  msg.x[3] = 0.2;
  msg.y[3] = -0.2;
  msg.z[3] = -0.05;

  msg.x[4] = 0.02685;
  msg.y[4] = -0.19881;
  msg.z[4] = -0.1046;

  msg.x[5] = -0.2;
  msg.y[5] = -0.2;
  msg.z[5] = -0.05;

  while (ros::ok())
  {
    ROS_INFO("I've published!");
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
