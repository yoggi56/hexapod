#include "ros/ros.h"
#include "leg_msgs/LegRefPos.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<leg_msgs::LegRefPos>("leg_ref_pos", 1);

  ros::Rate loop_rate(40);

  leg_msgs::LegRefPos msg;
  msg.x = 0.02685;
  msg.y = -0.19881;
  msg.z = -0.1046;

  while (ros::ok())
  {
    ROS_INFO("I've published!");
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
