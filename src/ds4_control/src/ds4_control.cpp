#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "../../../devel/include/leg_msgs/LegRefPos.h"

#define SQUARE    0
#define TRIANGLE  1
#define CIRCLE    2
#define CROSS     3
#define L1        4
#define L2        5
#define R1        6
#define R2        7
#define SHARE     8
#define OPTIONS   9
#define PS        10
#define TOUCH     11
#define L3        12
#define R3        13
#define LEFT      14
#define UP        15
#define RIGHT     16
#define DOWN      17

#define L3_HOR     0
#define L3_VERT    1
#define R3_HOR     2
#define R3_VERT    3
#define L2_FLOAT  4
#define R2_FLOAT  5

#define CIRCLE_MODE 1
#define JOY_MODE    2
#define SQUARE_MODE 4

static int mode = 2;
static double x_vel, y_vel, z_vel;

void joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  //  for (int i = 0; i < 18; i++)
  //    ROS_INFO("[%d]: [%d]", i, msg->buttons[i]);
  //  for (int i = 0; i < 6; i++)
  //    ROS_INFO("[%d]: [%f]", i, msg->axes[i]);

  x_vel = msg->axes[L3_VERT];
  y_vel = msg->axes[L3_HOR];
  z_vel = msg->axes[R3_VERT];

  if (msg->buttons[RIGHT])
  {
    ROS_INFO("RIGHT is pressed");
    mode = mode << 1;
  }
  else if (msg->buttons[LEFT])
  {
    ROS_INFO("LEFT is pressed");
    mode = mode >> 1;
  }

  if (mode == 0)
    mode = 1;
  else if (mode == 8)
    mode = 4;

  //  ROS_INFO("mode = [%d]", mode);
  //  ROS_INFO("X velocity: [%f]", msg->axes[L3_VERT]);
  //  ROS_INFO("Y velocity: [%f]", msg->axes[L3_HOR]);
  //  ROS_INFO("Z velocity: [%f]", msg->axes[R3_VERT]);
}

int main(int argc, char **argv)
{
  //circle params
  double phi = 0; // point of a circle
  double A = 4.0; // radius of a circle in cm

  // square params
  int line = 1;
  double a = 0.06;
  double inc = 0.003;
  // init position
  double x_ref_pos = 0.0, y_ref_pos = -0.19881, z_ref_pos = -0.1046;

  ros::init(argc, argv, "ds4_control");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("joy", 1000, joyCallback);
  ros::Publisher ref_pos_pub = nh.advertise<leg_msgs::LegRefPos>("leg_ref_pos", 1000);

  leg_msgs::LegRefPos msg;
  // start point of a leg
  msg.x = x_ref_pos;
  msg.y = y_ref_pos;
  msg.z = z_ref_pos;

  ros::Rate loop_rate(40);

  while(ros::ok())
  {
    switch(mode)
    {
    case CIRCLE_MODE :
      msg.x = A*cos(phi)*0.01 + x_ref_pos;
      msg.y = A*sin(phi)*0.01 + y_ref_pos;
      phi += 0.09;
      break;
    case JOY_MODE :
      x_ref_pos = x_ref_pos + x_vel/500.0;
      y_ref_pos = y_ref_pos + y_vel/500.0;
      z_ref_pos = z_ref_pos + z_vel/500.0;
      msg.x = x_ref_pos;
      msg.y = y_ref_pos;
      msg.z = z_ref_pos;

      break;
    case SQUARE_MODE :
      switch(line)
      {
      case 1:
        if (x_ref_pos >= -a/2.0)
          x_ref_pos -= inc;
        else
          line = 2;
        break;
      case 2:
        if (y_ref_pos <= (-0.19881+a/2.0))
          y_ref_pos += inc;
        else
          line = 3;
        break;
      case 3:
        if (x_ref_pos <= a/2.0)
          x_ref_pos += inc;
        else
          line = 4;
        break;
      case 4:
        if (y_ref_pos >= (-0.19881-a/2.0))
          y_ref_pos -= inc;
        else
          line = 1;
        break;
      default:
        break;
      }
      msg.x = x_ref_pos;
      msg.y = y_ref_pos;
      msg.z = z_ref_pos;
    default:
      break;
    }
    ROS_INFO("X: [%f]     Y: [%f]     Z: [%f]", x_ref_pos*100, y_ref_pos*100, z_ref_pos*100);
    ref_pos_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
