#include "ros/ros.h"
#include "std_msgs/String.h"
#include "leg_msgs/LegJointState.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<leg_msgs::LegJointState>("joints_ref_angles", 1);

  ros::Rate loop_rate(40);

  leg_msgs::LegJointState msg;
  msg.joint[0] = 0.0;
  msg.joint[1] = 0.0;
  msg.joint[2] = 0.0;

  while (ros::ok())
  {
    msg.joint[0] += 0.005;
    msg.joint[1] += 0.005;
    msg.joint[2] += 0.005;

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
