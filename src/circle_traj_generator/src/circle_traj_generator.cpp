#include "ros/ros.h"
#include "../../../devel/include/leg_msgs/LegRefPos.h"
#include <cmath>

int main(int argc, char **argv)
{
  double phi = 0; // point of a circle
  double A = 4.0; // radius of a circle in cm
  // start points of a leg
  double x_idle = 0.0;
  double y_idle = -0.19881;
  double z_idle = -0.1046;

  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<leg_msgs::LegRefPos>("leg_ref_pos", 1000);
  ros::Rate loop_rate(40);  //start publications with frequency 40Hz. According to Craig J. it's minimal frequency needed for smooth motion

  leg_msgs::LegRefPos msg;
  // start point of a leg
  msg.x = x_idle;
  msg.y = y_idle;
  msg.z = z_idle;

  while (ros::ok())
  {
    //generate new point of circle trajectory
    msg.x = A*cos(phi)*0.01 + x_idle;
    msg.y = A*sin(phi)*0.01 + y_idle;
    phi += 0.05;

    //public new data
    chatter_pub.publish(msg);
    ROS_INFO("Fuck off: [%.2f] [%.2f] [%.2f]", msg.x, msg.y, phi);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
