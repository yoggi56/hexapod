#include "ros/ros.h"
//#include "std_msgs/String.h"
// This node computes inverse kinematic problem for right second leg of a hexapod robot with insect configuration

#include <hexy_msgs/LegRefPos.h>
#include <hexy_msgs/LegRefJoints.h>
#include "ikine.h"

static double x[6], y[6], z[6]; // reference coordinates for end effector

// handler for topic /leg_ref_pos
void chatterLegPos(const hexy_msgs::LegRefPos::ConstPtr &state)
{
  for(int i = 0; i < 6; i++)
  {
    x[i] = state->x[i];
    y[i] = state->y[i];
    z[i] = state->z[i];
  }
  //ROS_INFO("[%f]   [%f]   [%f]", x[LEG_R2], y[LEG_R2], z[LEG_R2]);
}

//bool load_mech_params(ros::NodeHandle &n)
//{
//  bool ok = false;
//  double L1_legth, L2_legth, L3_legth;
//  double body_width_small, body_width_big, body_length, body_heigth;
//  if (n.getParam("hexy/mech/L1_legth", L1_legth)) ok = true;
//  if (n.getParam("hexy/mech/L2_legth", L2_legth)) ok = true;
//  if (n.getParam("hexy/mech/L2_legth", L3_legth)) ok = true;
//  if (n.getParam("hexy/mech/body_width_small", body_width_small)) ok = true;
//  if (n.getParam("hexy/mech/body_width_big", body_width_big)) ok = true;
//  if (n.getParam("hexy/mech/body_length", body_length)) ok = true;
//  if (n.getParam("hexy/mech/body_heigth", body_heigth)) ok = true;



//  return ok;
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "leg_inv_kinematic");

  ikine ik;
  vector <vector <double> > cal_joints(6, vector<double>(3));

  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<hexy_msgs::LegRefJoints>("joints_ref_angles", 1000);
  hexy_msgs::LegRefJoints msg;
  ros::Subscriber sub = n.subscribe("leg_ref_pos", 1000, chatterLegPos);

  // loading mechanical parameters
  bool ok = false;
  double L1_legth, L2_legth, L3_legth;
  double body_width_small, body_width_big, body_length, body_heigth;
  if (n.getParam("hexy/mech/L1_legth", L1_legth)) ok = true;
  if (n.getParam("hexy/mech/L2_legth", L2_legth)) ok = true;
  if (n.getParam("hexy/mech/L3_legth", L3_legth)) ok = true;
  if (n.getParam("hexy/mech/body_width_small", body_width_small)) ok = true;
  if (n.getParam("hexy/mech/body_width_big", body_width_big)) ok = true;
  if (n.getParam("hexy/mech/body_length", body_length)) ok = true;
  if (n.getParam("hexy/mech/body_heigth", body_heigth)) ok = true;

  if (ok)
  {
    ik.set_mechanical_params(L1_legth, L2_legth, L3_legth, body_width_small,
                             body_width_big, body_length, body_heigth);
  }
  else
  {
    ROS_FATAL("leg_inv_kinematic: Some of the mechanical parameters weren't loaded. "
              "Check the file hexy_mechanical_parameters.yaml in the package hexy_lib.");
    return -1;
  }

  //start publications with frequency 40Hz. According to Craig J. it's minimal frequency needed for smooth motion
  ros::Rate rate(40);
  while(ros::ok())
  {
    // update inv kine data
    cal_joints = ik.run(x, y, z);
    ROS_INFO("-------");
    ROS_INFO("[%2f] [%2f] [%2f] [%2f] [%2f] [%2f] [%2f]", L1_legth, L2_legth, L3_legth, body_width_small, body_width_big, body_length, body_heigth);
    ROS_INFO("L1: [%f]   [%f]   [%f]", cal_joints[LEG_L1][COXA], cal_joints[LEG_L1][FEMUR], cal_joints[LEG_L1][TIBIA]);
    ROS_INFO("L2: [%f]   [%f]   [%f]", cal_joints[LEG_L2][COXA], cal_joints[LEG_L2][FEMUR], cal_joints[LEG_L2][TIBIA]);
    ROS_INFO("L3: [%f]   [%f]   [%f]", cal_joints[LEG_L3][COXA], cal_joints[LEG_L3][FEMUR], cal_joints[LEG_L3][TIBIA]);
    ROS_INFO("R1: [%f]   [%f]   [%f]", cal_joints[LEG_R1][COXA], cal_joints[LEG_R1][FEMUR], cal_joints[LEG_R1][TIBIA]);
    ROS_INFO("R2: [%f]   [%f]   [%f]", cal_joints[LEG_R2][COXA], cal_joints[LEG_R2][FEMUR], cal_joints[LEG_R2][TIBIA]);
    ROS_INFO("R3: [%f]   [%f]   [%f]", cal_joints[LEG_R3][COXA], cal_joints[LEG_R3][FEMUR], cal_joints[LEG_R3][TIBIA]);

    msg.L1_joint[COXA] = double(cal_joints[LEG_L1][COXA]);
    msg.L1_joint[FEMUR] = double(cal_joints[LEG_L1][FEMUR]);
    msg.L1_joint[TIBIA] = double(cal_joints[LEG_L1][TIBIA]);
    msg.L2_joint[COXA] = double(cal_joints[LEG_L2][COXA]);
    msg.L2_joint[FEMUR] = double(cal_joints[LEG_L2][FEMUR]);
    msg.L2_joint[TIBIA] = double(cal_joints[LEG_L2][TIBIA]);
    msg.L3_joint[COXA] = double(cal_joints[LEG_L3][COXA]);
    msg.L3_joint[FEMUR] = double(cal_joints[LEG_L3][FEMUR]);
    msg.L3_joint[TIBIA] = double(cal_joints[LEG_L3][TIBIA]);
    msg.R1_joint[COXA] = double(cal_joints[LEG_R1][COXA]);
    msg.R1_joint[FEMUR] = double(cal_joints[LEG_R1][FEMUR]);
    msg.R1_joint[TIBIA] = double(cal_joints[LEG_R1][TIBIA]);
    msg.R2_joint[COXA] = double(cal_joints[LEG_R2][COXA]);
    msg.R2_joint[FEMUR] = double(cal_joints[LEG_R2][FEMUR]);
    msg.R2_joint[TIBIA] = double(cal_joints[LEG_R2][TIBIA]);
    msg.R3_joint[COXA] = double(cal_joints[LEG_R3][COXA]);
    msg.R3_joint[FEMUR] = double(cal_joints[LEG_R3][FEMUR]);
    msg.R3_joint[TIBIA] = double(cal_joints[LEG_R3][TIBIA]);
    // public new data
    joint_pub.publish(msg);

    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
