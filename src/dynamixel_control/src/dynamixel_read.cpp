#include <ros/ros.h>
//#include "std_msgs/Float64.h"
#include <hexy_msgs/LegCurJoints.h>
#include <dynamixel_msgs/JointState.h>
#include <hexy_lib/hexy_lib.h>

//#define LEG_L1 0
//#define LEG_L2 1
//#define LEG_L3 2
//#define LEG_R1 3
//#define LEG_R2 4
//#define LEG_R3 5

//#define COXA  0
//#define FEMUR 1
//#define TIBIA 2

static double cur_joints[6][3];

void L1_chatterCoxa(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_L1][COXA] = state.current_pos;
  //ROS_INFO("coxa: [%.2f]", state.current_pos);
}

void L1_chatterFemur(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_L1][FEMUR] = state.current_pos;
  //ROS_INFO("femur: [%.2f]", state.current_pos);
}

void L1_chatterTibia(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_L1][TIBIA] = state.current_pos;
  //ROS_INFO("tibia: [%.2f]", state.current_pos);
}

void L2_chatterCoxa(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_L2][COXA] = state.current_pos;
  //ROS_INFO("coxa: [%.2f]", state.current_pos);
}

void L2_chatterFemur(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_L2][FEMUR] = state.current_pos;
  //ROS_INFO("femur: [%.2f]", state.current_pos);
}

void L2_chatterTibia(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_L2][TIBIA] = state.current_pos;
  //ROS_INFO("tibia: [%.2f]", state.current_pos);
}

void L3_chatterCoxa(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_L3][COXA] = state.current_pos;
  //ROS_INFO("coxa: [%.2f]", state.current_pos);
}

void L3_chatterFemur(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_L3][FEMUR] = state.current_pos;
  //ROS_INFO("femur: [%.2f]", state.current_pos);
}

void L3_chatterTibia(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_L3][TIBIA] = state.current_pos;
  //ROS_INFO("tibia: [%.2f]", state.current_pos);
}

void R1_chatterCoxa(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_R1][COXA] = state.current_pos;
  //ROS_INFO("coxa: [%.2f]", state.current_pos);
}

void R1_chatterFemur(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_R1][FEMUR] = state.current_pos;
  //ROS_INFO("femur: [%.2f]", state.current_pos);
}

void R1_chatterTibia(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_R1][TIBIA] = state.current_pos;
  //ROS_INFO("tibia: [%.2f]", state.current_pos);
}

void R2_chatterCoxa(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_R2][COXA] = state.current_pos;
  //ROS_INFO("coxa: [%.2f]", state.current_pos);
}

void R2_chatterFemur(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_R2][FEMUR] = state.current_pos;
  //ROS_INFO("femur: [%.2f]", state.current_pos);
}

void R2_chatterTibia(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_R2][TIBIA] = state.current_pos;
  //ROS_INFO("tibia: [%.2f]", state.current_pos);
}

void R3_chatterCoxa(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_R3][COXA] = state.current_pos;
  //ROS_INFO("coxa: [%.2f]", state.current_pos);
}

void R3_chatterFemur(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_R3][FEMUR] = state.current_pos;
  //ROS_INFO("femur: [%.2f]", state.current_pos);
}

void R3_chatterTibia(const dynamixel_msgs::JointState &state)
{
  cur_joints[LEG_R3][TIBIA] = state.current_pos;
  //ROS_INFO("tibia: [%.2f]", state.current_pos);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_read");
  ros::NodeHandle nh;

  ros::Subscriber L1_coxa_sub = nh.subscribe("L1_coxa/state", 1000, L1_chatterCoxa);
  ros::Subscriber L1_femur_sub = nh.subscribe("L1_femur/state", 1000, L1_chatterFemur);
  ros::Subscriber L1_tibia_sub = nh.subscribe("L1_tibia/state", 1000, L1_chatterTibia);
  ros::Subscriber L2_coxa_sub = nh.subscribe("L2_coxa/state", 1000, L2_chatterCoxa);
  ros::Subscriber L2_femur_sub = nh.subscribe("L2_femur/state", 1000, L2_chatterFemur);
  ros::Subscriber L2_tibia_sub = nh.subscribe("L2_tibia/state", 1000, L2_chatterTibia);
  ros::Subscriber L3_coxa_sub = nh.subscribe("L3_coxa/state", 1000, L3_chatterCoxa);
  ros::Subscriber L3_femur_sub = nh.subscribe("L3_femur/state", 1000, L3_chatterFemur);
  ros::Subscriber L3_tibia_sub = nh.subscribe("L3_tibia/state", 1000, L3_chatterTibia);
  ros::Subscriber R1_coxa_sub = nh.subscribe("R1_coxa/state", 1000, R1_chatterCoxa);
  ros::Subscriber R1_femur_sub = nh.subscribe("R1_femur/state", 1000, R1_chatterFemur);
  ros::Subscriber R1_tibia_sub = nh.subscribe("R1_tibia/state", 1000, R1_chatterTibia);
  ros::Subscriber R2_coxa_sub = nh.subscribe("R2_coxa/state", 1000, R2_chatterCoxa);
  ros::Subscriber R2_femur_sub = nh.subscribe("R2_femur/state", 1000, R2_chatterFemur);
  ros::Subscriber R2_tibia_sub = nh.subscribe("R2_tibia/state", 1000, R2_chatterTibia);
  ros::Subscriber R3_coxa_sub = nh.subscribe("R3_coxa/state", 1000, R3_chatterCoxa);
  ros::Subscriber R3_femur_sub = nh.subscribe("R3_femur/state", 1000, R3_chatterFemur);
  ros::Subscriber R3_tibia_sub = nh.subscribe("R3_tibia/state", 1000, R3_chatterTibia);

  ros::Publisher cur_angles_pub = nh.advertise<hexy_msgs::LegCurJoints>("joints_cur_angles", 1000);
  hexy_msgs::LegCurJoints msg;
  ros::Rate rate(40);
  ROS_INFO("check");

  for(int i = 0; i < 6; i++)
  {
    cur_joints[i][COXA] = 0;
    cur_joints[i][FEMUR] = 0;
    cur_joints[i][TIBIA] = 0;
  }

  while(ros::ok())
  {
    for(int i = 0; i < 3; i++)
    {
    msg.L1_joint[i] = cur_joints[LEG_L1][i];
    msg.L2_joint[i] = cur_joints[LEG_L2][i];
    msg.L3_joint[i] = cur_joints[LEG_L3][i];
    msg.R1_joint[i] = cur_joints[LEG_R1][i];
    msg.R2_joint[i] = cur_joints[LEG_R2][i];
    msg.R3_joint[i] = cur_joints[LEG_R3][i];
    }

    //ROS_INFO("coxa: [%.3f]; femur: [%.3f]; tibia: [%.3f]", msg.cur_joint[0], msg.cur_joint[1], msg.cur_joint[2]);
    cur_angles_pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

}
