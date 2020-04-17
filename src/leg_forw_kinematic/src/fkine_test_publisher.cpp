#include "ros/ros.h"
#include <hexy_msgs/LegCurJoints.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fkine_test_publisher");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<hexy_msgs::LegCurJoints>("joints_cur_angles", 1000);

  ros::Rate loop_rate(40);

  hexy_msgs::LegCurJoints msg;
  msg.L1_joint[0] = 2;
  msg.L1_joint[1] = 1;
  msg.L1_joint[2] = 2;

  msg.L2_joint[0] = 2;
  msg.L2_joint[1] = 1;
  msg.L2_joint[2] = 2;

  msg.L3_joint[0] = 2;
  msg.L3_joint[1] = 1;
  msg.L3_joint[2] = 2;

  msg.R1_joint[0] = 2;
  msg.R1_joint[1] = 1;
  msg.R1_joint[2] = 2;

  msg.R2_joint[0] = 2;
  msg.R2_joint[1] = 1;
  msg.R2_joint[2] = 2;

  msg.R3_joint[0] = 2;
  msg.R3_joint[1] = 1;
  msg.R3_joint[2] = 2;

  while (ros::ok())
  {
    ROS_INFO("I've published!");
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
