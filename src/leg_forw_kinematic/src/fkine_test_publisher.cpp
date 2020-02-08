#include "ros/ros.h"
#include "../../../devel/include/leg_msgs/LegCurAngles.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fkine_test_publisher");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<leg_msgs::LegCurAngles>("joints_cur_angles", 1000);

  ros::Rate loop_rate(40);

  leg_msgs::LegCurAngles msg;
  msg.cur_joint[0] = 2;
  msg.cur_joint[1] = 1;
  msg.cur_joint[2] = 2;

  while (ros::ok())
  {
    ROS_INFO("I've published!");
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
