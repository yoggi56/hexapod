#include "ros/ros.h"
#include <std_msgs/String.h>
#include "ds4_driver/Status.h"
#include "ds4_driver/Feedback.h"
#include <hexy_msgs/LegRefPos.h>
#include <geometry_constraints/geom_constr.h>
#include <hexy_lib/hexy_lib.h>

//#define SQUARE    0
//#define TRIANGLE  1
//#define CIRCLE    2
//#define CROSS     3
//#define L1        4
//#define L2        5
//#define R1        6
//#define R2        7
//#define SHARE     8
//#define OPTIONS   9
//#define PS        10
//#define TOUCH     11
//#define L3        12
//#define R3        13
//#define LEFT      14
//#define UP        15
//#define RIGHT     16
//#define DOWN      17

//#define L3_HOR     0
//#define L3_VERT    1
//#define R3_HOR     2
//#define R3_VERT    3
//#define L2_FLOAT  4
//#define R2_FLOAT  5

//#define CIRCLE_MODE 1
//#define JOY_MODE    2

//#define L1_MODE 1
//#define L2_MODE 2
//#define L3_MODE 3
//#define R1_MODE 4
//#define R2_MODE 5
//#define R3_MODE 6

//#define LEG_L1 0
//#define LEG_L2 1
//#define LEG_L3 2
//#define LEG_R1 3
//#define LEG_R2 4
//#define LEG_R3 5

#define GO_IDLE       1
#define LEG_CONTROL   2
#define BODY_CONTROL  3
#define WALK          4
#define GO_SLEEP      5

#define WAITING           0
#define STAND_UP          1
#define FIRST_LEGS_UP     2
#define FIRST_LEGS_DOWN   3
#define SECOND_LEGS_UP    4
#define SECOND_LEGS_DOWN  5

static int traj_mode = 2, leg_mode = 5;
static double x_vel, y_vel, z_vel;
static bool btn_left = 0, btn_right = 0;
static bool btn_up = 0, btn_down = 0;
static bool btn_cross = 0, btn_square = 0;
static bool btn_circle = 0, btn_triangle = 0, btn_options = 0;
static int robot_state = 0, idle_state = 0;
void joyCallback(const ds4_driver::Status::ConstPtr &msg)
{
  //  for (int i = 0; i < 18; i++)
  //    ROS_INFO("[%d]: [%d]", i, msg->buttons[i]);
  //  for (int i = 0; i < 6; i++)
  //    ROS_INFO("[%d]: [%f]", i, msg->axes[i]);


  //---------------------------------------------
  // start going to the idle state
  if (msg->button_cross && btn_cross == 0)
  {
    btn_cross = 1;
    robot_state = GO_IDLE;
    ROS_INFO("GO IDLE");
  }
  else if(!msg->button_cross)
    btn_cross = 0;

  // start going to the leg control state
  if (msg->button_square && btn_square == 0)
  {
    btn_square = 1;
    robot_state = LEG_CONTROL;
    ROS_INFO("LEG CONTROL");
  }
  else if(!msg->button_square)
    btn_square = 0;

  // start going to the body control state
  if (msg->button_triangle && btn_triangle == 0)
  {
    btn_triangle = 1;
    robot_state = BODY_CONTROL;
    ROS_INFO("BODY CONTROL");
  }
  else if(!msg->button_triangle)
    btn_triangle = 0;

  // start going to the walking state
  if (msg->button_options && btn_options == 0)
  {
    btn_options = 1;
    robot_state = WALK;
    ROS_INFO("WALKING STATE");
  }
  else if(!msg->button_options)
    btn_options = 0;

  // start going to the sleeping state
  if (msg->button_circle && btn_circle == 0)
  {
    btn_circle = 1;
    robot_state = WALK;
    ROS_INFO("GO SLEEPING");
  }
  else if(!msg->button_circle)
    btn_circle = 0;

  //--------------------------------------------
  //--body_control------------------------------
  if (robot_state == BODY_CONTROL)
  {
    // reading velocity of the body moving
    x_vel = double(msg->axis_left_y);
    y_vel = double(msg->axis_left_x);
    z_vel = double(msg->axis_right_y);
  }

  //--------------------------------------------
  //--leg_control-------------------------------
  if (robot_state == LEG_CONTROL)
  {
    // reading velocity of the end effector moving
    x_vel = double(msg->axis_left_y);
    y_vel = double(msg->axis_left_x);
    z_vel = double(msg->axis_right_y);
    //ROS_INFO("x: %f   y: %f   z: %f", x_vel, y_vel, z_vel);
    // switching between hand control and circle mode
    if (msg->button_dpad_up && btn_up == 0)
    {
      btn_up = 1;
      traj_mode = traj_mode << 1;
      if (traj_mode == 4)
        traj_mode = 2;
      ROS_INFO("UP is pressed. Mode: %d", traj_mode);
    }
    else if (!msg->button_dpad_up)
    {
      btn_up = 0;
    }

    if (msg->button_dpad_down && btn_down == 0)
    {
      btn_down = 1;
      traj_mode = traj_mode >> 1;
      if (traj_mode == 0)
        traj_mode = 1;
      ROS_INFO("DOWN is pressed. Mode: %d", traj_mode);
    }
    else if (!msg->button_dpad_down)
    {
      btn_down = 0;
    }

    //switching between legs
    if (msg->button_dpad_left && btn_left == 0)
    {
      btn_left = 1;
      leg_mode -= 1;
      if (leg_mode == -1)
        leg_mode = 0;
      ROS_INFO("LEFT is pressed. Mode: %d", leg_mode);
    }
    else if (!msg->button_dpad_left)
    {
      btn_left = 0;
    }

    if (msg->button_dpad_right && btn_right == 0)
    {
      btn_right = 1;
      leg_mode += 1;
      if (leg_mode == 6)
        leg_mode = 5;
      ROS_INFO("RIGHT is pressed. Mode: %d", leg_mode);
    }
    else if (!msg->button_dpad_right)
    {
      btn_right = 0;
    }
  }
  //  ROS_INFO("mode = [%d]", mode);
  //  ROS_INFO("X velocity: [%f]", msg->axes[L3_VERT]);
  //  ROS_INFO("Y velocity: [%f]", msg->axes[L3_HOR]);
  //  ROS_INFO("Z velocity: [%f]", msg->axes[R3_VERT]);
}

int main(int argc, char **argv)
{
  bool isIdle = 0;
  double stand_up_vel = 0.002;
  //circle params
  double phi = 0; // point of a circle
  double A = 3.0; // radius of a circle in cm
  double x_idle[6], y_idle[6], z_idle[6];
  x_idle[LEG_L1] = 0.1937;
  y_idle[LEG_L1] = 0.1975;
  z_idle[LEG_L1] = -0.0746;
  x_idle[LEG_L2] = 0.0;
  y_idle[LEG_L2] = 0.19881;
  z_idle[LEG_L2] = -0.0746;
  x_idle[LEG_L3] = -0.1937;
  y_idle[LEG_L3] = 0.1975;
  z_idle[LEG_L3] = -0.0746;
  x_idle[LEG_R1] = 0.1937;
  y_idle[LEG_R1] = -0.1975;
  z_idle[LEG_R1] = -0.0746;
  x_idle[LEG_R2] = 0.0;
  y_idle[LEG_R2] = -0.19881;
  z_idle[LEG_R2] = -0.0746;
  x_idle[LEG_R3] = -0.1937;
  y_idle[LEG_R3] = -0.1975;
  z_idle[LEG_R3] = -0.0746;

  // initial leg positions
  double x_ref_pos[6], y_ref_pos[6], z_ref_pos[6];
  x_ref_pos[LEG_L1] = 0.16;
  y_ref_pos[LEG_L1] = 0.125;
  z_ref_pos[LEG_L1] = -0.05;
  x_ref_pos[LEG_L2] = 0.0;
  y_ref_pos[LEG_L2] = 0.17;
  z_ref_pos[LEG_L2] = -0.05;
  x_ref_pos[LEG_L3] = -0.16;
  y_ref_pos[LEG_L3] = 0.125;
  z_ref_pos[LEG_L3] = -0.05;
  x_ref_pos[LEG_R1] = 0.16;
  y_ref_pos[LEG_R1] = -0.125;
  z_ref_pos[LEG_R1] = -0.05;
  x_ref_pos[LEG_R2] = 0.0;
  y_ref_pos[LEG_R2] = -0.17;
  z_ref_pos[LEG_R2] = -0.05;
  x_ref_pos[LEG_R3] = -0.16;
  y_ref_pos[LEG_R3] = -0.125;
  z_ref_pos[LEG_R3] = -0.05;

  ros::init(argc, argv, "ds4_control");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("status", 1000, joyCallback);
  ros::Publisher ref_pos_pub = nh.advertise<hexy_msgs::LegRefPos>("leg_ref_pos", 1000);
  ros::Publisher ds_fb_pub = nh.advertise<ds4_driver::Feedback>("set_feedback", 1000);
  ros::ServiceClient client = nh.serviceClient<geometry_constraints::geom_constr>("geometry_constraints");

  geometry_constraints::geom_constr srv;
  hexy_msgs::LegRefPos msg;
  // start point of a leg
  msg.x[0] = x_ref_pos[0];
  msg.y[0] = y_ref_pos[0];
  msg.z[0] = z_ref_pos[0];

  msg.x[1] = x_ref_pos[1];
  msg.y[1] = y_ref_pos[1];
  msg.z[1] = z_ref_pos[1];

  msg.x[2] = x_ref_pos[2];
  msg.y[2] = y_ref_pos[2];
  msg.z[2] = z_ref_pos[2];

  msg.x[3] = x_ref_pos[3];
  msg.y[3] = y_ref_pos[3];
  msg.z[3] = z_ref_pos[3];

  msg.x[4] = x_ref_pos[4];
  msg.y[4] = y_ref_pos[4];
  msg.z[4] = z_ref_pos[4];

  msg.x[5] = x_ref_pos[5];
  msg.y[5] = y_ref_pos[5];
  msg.z[5] = z_ref_pos[5];

  ds4_driver::Feedback fb_msg;

  ros::Rate loop_rate(40);

  while(ros::ok())
  {

    switch(robot_state)
    {
    //-----------------------------------------
    //--going to the idle state----------------
    case GO_IDLE:
      switch(idle_state)
      {
      case WAITING:
        if (isIdle == false)
          idle_state = STAND_UP;
        break;
      case STAND_UP:
        for (int i = 0; i < 6; i++)
        {
          if (z_ref_pos[i] <= -0.09)
            idle_state = FIRST_LEGS_UP;
          else
            z_ref_pos[i] = z_ref_pos[i] - stand_up_vel;
        }
        break;
      case FIRST_LEGS_UP:
        if (z_ref_pos[LEG_L1] <= -0.06)  {
          x_ref_pos[LEG_L1] = x_ref_pos[LEG_L1] + stand_up_vel;
          y_ref_pos[LEG_L1] = y_ref_pos[LEG_L1] + 2*stand_up_vel;
          z_ref_pos[LEG_L1] = z_ref_pos[LEG_L1] + stand_up_vel;
          x_ref_pos[LEG_L3] = x_ref_pos[LEG_L3] - stand_up_vel;
          y_ref_pos[LEG_L3] = y_ref_pos[LEG_L3] + 2*stand_up_vel;
          z_ref_pos[LEG_L3] = z_ref_pos[LEG_L3] + stand_up_vel;
          y_ref_pos[LEG_R2] = y_ref_pos[LEG_R2] - 2*stand_up_vel;
          z_ref_pos[LEG_R2] = z_ref_pos[LEG_R2] + stand_up_vel;
        }
        else
          idle_state = FIRST_LEGS_DOWN;
        break;
      case FIRST_LEGS_DOWN:
        if (z_ref_pos[LEG_L1] >= -0.09)
        {
          z_ref_pos[LEG_L1] = z_ref_pos[LEG_L1] - stand_up_vel;
          z_ref_pos[LEG_L3] = z_ref_pos[LEG_L3] - stand_up_vel;
          z_ref_pos[LEG_R2] = z_ref_pos[LEG_R2] - stand_up_vel;
        }
        else
          idle_state = SECOND_LEGS_UP;
        break;
      case SECOND_LEGS_UP:
        if (z_ref_pos[LEG_L2] <= -0.06)  {
          x_ref_pos[LEG_R1] += stand_up_vel;
          y_ref_pos[LEG_R1] -= 2*stand_up_vel;
          z_ref_pos[LEG_R1] += stand_up_vel;
          x_ref_pos[LEG_R3] -= stand_up_vel;
          y_ref_pos[LEG_R3] -= 2*stand_up_vel;
          z_ref_pos[LEG_R3] += stand_up_vel;
          y_ref_pos[LEG_L2] += 2*stand_up_vel;
          z_ref_pos[LEG_L2] += stand_up_vel;
        }
        else
          idle_state = SECOND_LEGS_DOWN;
        break;
      case SECOND_LEGS_DOWN:
        if (z_ref_pos[LEG_R1] >= -0.09)
        {
          z_ref_pos[LEG_R1] -= stand_up_vel;
          z_ref_pos[LEG_R3] -= stand_up_vel;
          z_ref_pos[LEG_L2] -= stand_up_vel;
        }
        else
        {
          idle_state = WAITING;
          isIdle = true;
        }
        break;
      default:
        break;
      }
      break;
    //-----------------------------------------
    //--hand control of all the legs-----------
    case LEG_CONTROL:
      switch(traj_mode)
      {
      case CIRCLE_MODE :
        x_ref_pos[leg_mode] = A*cos(phi)*0.01 + x_idle[leg_mode];
        y_ref_pos[leg_mode] = A*sin(phi)*0.01 + y_idle[leg_mode];
        z_ref_pos[leg_mode] = z_idle[leg_mode];
        phi += 0.09;
        break;
      case JOY_MODE :
        x_ref_pos[leg_mode] = x_ref_pos[leg_mode] + x_vel/500.0;
        y_ref_pos[leg_mode] = y_ref_pos[leg_mode] + y_vel/500.0;
        z_ref_pos[leg_mode] = z_ref_pos[leg_mode] + z_vel/500.0;
        //      msg.x = x_ref_pos;
        //      msg.y = y_ref_pos;
        //      msg.z = z_ref_pos;
        break;
      default:
        break;
      }
      break;

    //-----------------------------------------
    //--hand control of all the legs-----------
    case BODY_CONTROL:
      for (int i = 0; i < 6; i++)
      {
        x_ref_pos[i] -= x_vel/400.0;
        y_ref_pos[i] -= y_vel/400.0;
        z_ref_pos[i] -= z_vel/400.0;
      }
      break;
    case WALK:
      break;
    case GO_SLEEP:
      break;
    default:
      break;
    }


    // checking position
    for(int i = 0; i < 6; i++)
    {
      srv.request.x[i] = x_ref_pos[i];
      srv.request.y[i] = y_ref_pos[i];
      srv.request.z[i] = z_ref_pos[i];

      if (client.call(srv))
      {
        if (srv.response.ok[i])
        {
          fb_msg.set_rumble = false;
          fb_msg.rumble_big = 0.0;
        }
        else {
          //ROS_INFO("Vibration");
          fb_msg.set_rumble = true;
          fb_msg.rumble_big = 0.5;
          x_ref_pos[i] = srv.response.x_out[i];
          y_ref_pos[i] = srv.response.y_out[i];
          z_ref_pos[i] = srv.response.z_out[i];
        }
      }
    }

    for(int i = 0; i <= 5; i++)
    {
      msg.x[i] = x_ref_pos[i];
      msg.y[i] = y_ref_pos[i];
      msg.z[i] = z_ref_pos[i];
    }

    //    ROS_INFO("LEG CHOSEN: %d", leg_mode);
    //    ROS_INFO("L1: X:[%f]     Y: [%f]     Z: [%f]", x_ref_pos[0]*100, y_ref_pos[0]*100, z_ref_pos[0]*100);
    //    ROS_INFO("L2: X:[%f]     Y: [%f]     Z: [%f]", x_ref_pos[1]*100, y_ref_pos[1]*100, z_ref_pos[1]*100);
    //    ROS_INFO("L3: X:[%f]     Y: [%f]     Z: [%f]", x_ref_pos[2]*100, y_ref_pos[2]*100, z_ref_pos[2]*100);
    //    ROS_INFO("R1: X:[%f]     Y: [%f]     Z: [%f]", x_ref_pos[3]*100, y_ref_pos[3]*100, z_ref_pos[3]*100);
    //    ROS_INFO("R2: X:[%f]     Y: [%f]     Z: [%f]", x_ref_pos[4]*100, y_ref_pos[4]*100, z_ref_pos[4]*100);
    //    ROS_INFO("R3: X:[%f]     Y: [%f]     Z: [%f]", x_ref_pos[5]*100, y_ref_pos[5]*100, z_ref_pos[5]*100);

    ref_pos_pub.publish(msg);
    ds_fb_pub.publish(fb_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
