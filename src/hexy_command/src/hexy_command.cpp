#include <ros/ros.h>
#include <ds4_driver/Status.h>
#include <ds4_driver/Feedback.h>
#include <hexy_msgs/HexyRefVel.h>
#include <hexy_lib/hexy_lib.h>

//#define GO_IDLE       1
//#define GO_SLEEP      2
//#define MOVING        3

//#define X 0
//#define Y 1
//#define Z 2

static bool btn_options = 0, btn_share = 0;
static bool btn_left = 0, btn_right = 0;
static int robot_command = 2;

static double walk_dir[2], walk_rotate;
static int walk_type = 1;

static double leg_vel[3];
static double body_transl[3], body_rot[3];

static short leg_mode = 0;

static short command_it = 0;

void joyCallback(const ds4_driver::Status::ConstPtr &msg)
{
  //---------------------------------------------------
  //- Options is pressed -> go to the idle state ------
  if (msg->button_options && btn_options == 0)
  {
    btn_options = 1;
    robot_command = GO_IDLE;
    ROS_INFO("GO IDLE");
  }
  else if(!msg->button_options)
    btn_options = 0;

  if (robot_command == GO_IDLE && command_it <= 100)
  {
    command_it++;
  }
  else if (robot_command == GO_IDLE && command_it > 100)//if (robot_command != GO_SLEEP)
  {
    robot_command = MOVING;
    command_it = 0;
  }
  //---------------------------------------------------
  //- Share is pressed -> go to the sleeping state ----
  if (msg->button_share && btn_share == 0)
  {
    btn_share = 1;
    robot_command = GO_SLEEP;
    ROS_INFO("GO SLEEP");
  }
  else if(!msg->button_share)
    btn_share = 0;

  //---------------------------------------------------
  //- ref walking velocities --------------------------
  if (!msg->button_l1 && !msg->button_r1)
  {
    walk_dir[X] = double(msg->axis_left_y)/500.0;
    walk_dir[Y] = double(msg->axis_left_x)/500.0;
    walk_rotate = double(msg->axis_r2 - msg->axis_l2)/500.0;
  }

  //- switching between walking types -----------------
  if (!msg->button_l1 && !msg->button_r1 && msg->button_dpad_left && btn_left == 0)
  {
    btn_left = 1;
    walk_type -= 1;
    if (walk_type <= 0)
      walk_type = 1;
    ROS_INFO("LEFT is pressed. Walking type is: %d", walk_type);
  }
  else if (!msg->button_dpad_left)
  {
    btn_left = 0;
  }

  if (!msg->button_l1 && !msg->button_r1 && msg->button_dpad_right && btn_right == 0)
  {
    btn_right = 1;
    walk_type += 1;
    if (walk_type >= 4)
      walk_type = 3;
    ROS_INFO("RIGHT is pressed. Walking type: %d", walk_type);
  }
  else if (!msg->button_dpad_right)
  {
    btn_right = 0;
  }

  //---------------------------------------------------
  //- ref velocities for legs--------------------------
  if (msg->button_l1)
  {
    leg_vel[X] = double(msg->axis_left_y)/500.0;
    leg_vel[Y] = double(msg->axis_left_x)/500.0;
    leg_vel[Z] = double(msg->axis_right_y)/500.0;
    //- switching between legs-------------------------
    if (msg->button_dpad_left && btn_left == 0)
    {
      btn_left = 1;
      leg_mode -= 1;
      if (leg_mode == -1)
        leg_mode = 0;
      ROS_INFO("L1+LEFT is pressed. Leg number: %d", leg_mode);
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
      ROS_INFO("L1+RIGHT is pressed. Leg number: %d", leg_mode);
    }
    else if (!msg->button_dpad_right)
    {
      btn_right = 0;
    }
  }

  //---------------------------------------------------
  //- ref velocities for the body----------------------
  else if (msg->button_r1)
  {
    // translation velocities
    body_transl[X] = double(msg->axis_left_y)/400.0;
    body_transl[Y] = double(msg->axis_left_x)/400.0;
    if (msg->button_dpad_up)
      body_transl[Z] = 0.0015;
    else if (msg->button_dpad_down)
      body_transl[Z] = -0.0015;
    else
      body_transl[Z] = 0.0;

    // rotation velocities
    body_rot[X] = double(msg->axis_right_x)/100.0;
    body_rot[Y] = double(msg->axis_right_y)/100.0;
    body_rot[Z] = double(msg->axis_r2 - msg->axis_l2)/100.0;
  }

  //---------------------------------------------------
  //- reset all legs and body velocities---------------
  else {
    leg_vel[X] = 0.0;
    leg_vel[Y] = 0.0;
    leg_vel[Z] = 0.0;
    body_transl[X] = 0.0;
    body_transl[Y] = 0.0;
    body_transl[Z] = 0.0;
    body_rot[X] = 0.0;
    body_rot[Y] = 0.0;
    body_rot[Z] = 0.0;
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hexy_command");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("status", 1000, joyCallback);
  ros::Publisher ref_vel_pub = nh.advertise<hexy_msgs::HexyRefVel>("robot_ref_vel", 1000);
  ros::Publisher ds_fb_pub = nh.advertise<ds4_driver::Feedback>("set_feedback", 1000);
  hexy_msgs::HexyRefVel vel_msg;

  ros::Rate loop_rate(40);
  ROS_INFO("Hexy command node is ready to work");

  while(ros::ok())
  {
    if (robot_command == MOVING)
    {
      vel_msg.leg_x[leg_mode] = leg_vel[X];
      vel_msg.leg_y[leg_mode] = leg_vel[Y];
      vel_msg.leg_z[leg_mode] = leg_vel[Z];
      for(int i = 0; i < 3; i++)
      {
        vel_msg.body_transl[i] = body_transl[i];
        vel_msg.body_rotate[i] = body_rot[i];
      }
      vel_msg.walk_dir[X] = walk_dir[X];
      vel_msg.walk_dir[Y] = walk_dir[Y];
      vel_msg.walk_rotate = walk_rotate;
      vel_msg.walk_type = walk_type;
    }
    vel_msg.command = robot_command;

    ref_vel_pub.publish(vel_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
