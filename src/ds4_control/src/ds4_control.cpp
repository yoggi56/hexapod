#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ds4_driver/Status.h"
#include "ds4_driver/Feedback.h"
#include "leg_msgs/LegRefPos.h"
#include "geometry_constraints/geom_constr.h"

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
static bool btn_left = 0, btn_right = 0;
void joyCallback(const ds4_driver::Status::ConstPtr &msg)
{
  //  for (int i = 0; i < 18; i++)
  //    ROS_INFO("[%d]: [%d]", i, msg->buttons[i]);
  //  for (int i = 0; i < 6; i++)
  //    ROS_INFO("[%d]: [%f]", i, msg->axes[i]);

  x_vel = msg->axis_left_y;
  y_vel = msg->axis_left_x;
  z_vel = msg->axis_right_y;
  //ROS_INFO("x: %f   y: %f   z: %f", x_vel, y_vel, z_vel);

  if (msg->button_dpad_right && btn_right == 0)
  {
    ROS_INFO("RIGHT is pressed");
    btn_right = 1;
    mode = mode << 1;
  }
  else if (!msg->button_dpad_right)
  {
    btn_right = 0;
  }

  if (msg->button_dpad_left && btn_left == 0)
  {
    ROS_INFO("LEFT is pressed");
    btn_left = 1;
    mode = mode >> 1;
  }
  else if (!msg->button_dpad_left)
  {
    btn_left = 0;
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
  double A = 3.0; // radius of a circle in cm
  double x_idle = 0.0;
  double y_idle = -0.19881;
  double z_idle = -0.0746;

  // square params
  int line = 1;
  double a = 0.06;
  double inc = 0.003;
  // init position
  double x_ref_pos = 0.0, y_ref_pos = -0.19881, z_ref_pos = -0.1046;

  ros::init(argc, argv, "ds4_control");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("status", 1000, joyCallback);
  ros::Publisher ref_pos_pub = nh.advertise<leg_msgs::LegRefPos>("leg_ref_pos", 1000);
  ros::Publisher ds_fb_pub = nh.advertise<ds4_driver::Feedback>("set_feedback", 1000);
  ros::ServiceClient client = nh.serviceClient<geometry_constraints::geom_constr>("geometry_constraints");

  geometry_constraints::geom_constr srv;
  leg_msgs::LegRefPos msg;
  // start point of a leg
  msg.x = x_ref_pos;
  msg.y = y_ref_pos;
  msg.z = z_ref_pos;

  ds4_driver::Feedback fb_msg;


  ros::Rate loop_rate(40);

  while(ros::ok())
  {

    switch(mode)
    {
    case CIRCLE_MODE :
      x_ref_pos = A*cos(phi)*0.01 + x_idle;
      y_ref_pos = A*sin(phi)*0.01 + y_idle;
      z_ref_pos = z_idle;
      phi += 0.09;
      break;
    case JOY_MODE :
      x_ref_pos = x_ref_pos + x_vel/500.0;
      y_ref_pos = y_ref_pos + y_vel/500.0;
      z_ref_pos = z_ref_pos + z_vel/500.0;
      //      msg.x = x_ref_pos;
      //      msg.y = y_ref_pos;
      //      msg.z = z_ref_pos;

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
      //      msg.x = x_ref_pos;
      //      msg.y = y_ref_pos;
      //      msg.z = z_ref_pos;
    default:
      break;
    }

    // checking position
    srv.request.x = x_ref_pos;
    srv.request.y = y_ref_pos;
    srv.request.z = z_ref_pos;

    if (client.call(srv))
    {
      if (srv.response.ok)
      {
        fb_msg.set_rumble = false;
        fb_msg.rumble_big = 0.0;
      }
      else {
        //ROS_INFO("Vibration");
        fb_msg.set_rumble = true;
        fb_msg.rumble_big = 0.5;
        x_ref_pos = srv.response.x_out;
        y_ref_pos = srv.response.y_out;
        z_ref_pos = srv.response.z_out;
      }
    }

    msg.x = x_ref_pos;
    msg.y = y_ref_pos;
    msg.z = z_ref_pos;

    //ROS_INFO("X: [%f]     Y: [%f]     Z: [%f]", x_ref_pos*100, y_ref_pos*100, z_ref_pos*100);
    ref_pos_pub.publish(msg);
    ds_fb_pub.publish(fb_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
