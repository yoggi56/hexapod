#include "ros/ros.h"
#include <hexy_msgs/LegCurJoints.h>
#include <hexy_msgs/LegCurPos.h>

#include "fkine.h"

static double cur_joints[6][3];

// handler for topic /joints_cur_angles
void curAnglesCallback(const hexy_msgs::LegCurJoints::ConstPtr &msg)
{
  for(int i = 0; i < 3; i++)
  {
    cur_joints[LEG_L1][i] = msg->L1_joint[i];
    cur_joints[LEG_L2][i] = msg->L2_joint[i];
    cur_joints[LEG_L3][i] = msg->L3_joint[i];
    cur_joints[LEG_R1][i] = msg->R1_joint[i];
    cur_joints[LEG_R2][i] = msg->R2_joint[i];
    cur_joints[LEG_R3][i] = msg->R3_joint[i];
  }
  ROS_INFO("Current joint angles: ");
  ROS_INFO("L1: COXA: [%f2] FEMUR: [%f2] TIBIA: [%f2]", msg->L1_joint[COXA], msg->L1_joint[FEMUR], msg->L1_joint[TIBIA]);
  ROS_INFO("L2: COXA: [%f2] FEMUR: [%f2] TIBIA: [%f2]", msg->L2_joint[COXA], msg->L2_joint[FEMUR], msg->L2_joint[TIBIA]);
  ROS_INFO("L3: COXA: [%f2] FEMUR: [%f2] TIBIA: [%f2]", msg->L3_joint[COXA], msg->L3_joint[FEMUR], msg->L3_joint[TIBIA]);
  ROS_INFO("R1: COXA: [%f2] FEMUR: [%f2] TIBIA: [%f2]", msg->R1_joint[COXA], msg->R1_joint[FEMUR], msg->R1_joint[TIBIA]);
  ROS_INFO("R2: COXA: [%f2] FEMUR: [%f2] TIBIA: [%f2]", msg->R2_joint[COXA], msg->R2_joint[FEMUR], msg->R2_joint[TIBIA]);
  ROS_INFO("R3: COXA: [%f2] FEMUR: [%f2] TIBIA: [%f2]", msg->R3_joint[COXA], msg->R3_joint[FEMUR], msg->R3_joint[TIBIA]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "leg_forw_kinematic");
  ros::NodeHandle nh;

  fkine fk;
  vector <vector <double> > cal_pos(6, vector<double>(3));

  ros::Subscriber sub = nh.subscribe("joints_cur_angles", 1000, curAnglesCallback);
  ros::Publisher pub = nh.advertise<hexy_msgs::LegCurPos>("leg_cur_pos", 1000);
  hexy_msgs::LegCurPos msg;

  // loading mechanical parameters
  bool ok = false;
  double L1_legth, L2_legth, L3_legth;
  double body_width_small, body_width_big, body_length, body_heigth;
  if (nh.getParam("hexy/mech/L1_legth", L1_legth)) ok = true;
  if (nh.getParam("hexy/mech/L2_legth", L2_legth)) ok = true;
  if (nh.getParam("hexy/mech/L3_legth", L3_legth)) ok = true;
  if (nh.getParam("hexy/mech/body_width_small", body_width_small)) ok = true;
  if (nh.getParam("hexy/mech/body_width_big", body_width_big)) ok = true;
  if (nh.getParam("hexy/mech/body_length", body_length)) ok = true;
  if (nh.getParam("hexy/mech/body_heigth", body_heigth)) ok = true;

  if (ok)
  {
    fk.set_mechanical_params(L1_legth, L2_legth, L3_legth, body_width_small,
                             body_width_big, body_length, body_heigth);
  }
  else
  {
    ROS_FATAL("leg_forw_kinematic: Some of the mechanical parameters weren't loaded. "
              "Check the file hexy_mechanical_parameters.yaml in the package hexy_lib.");
    return -1;
  }

  //start publications with frequency 40Hz. According to Craig J. it's minimal frequency needed for smooth motion
  ros::Rate rate(40);
  while(ros::ok())
  {
    //update forward kinematic data
    cal_pos = fk.run(cur_joints);
    for (int i = 0; i < 6; i++)
    {
      msg.x[i] = cal_pos[i][COXA];
      msg.y[i] = cal_pos[i][FEMUR];
      msg.z[i] = cal_pos[i][TIBIA];
    }

    ROS_INFO("------------------------");
    ROS_INFO("Current position: ");
    ROS_INFO("L1: X: [%f2] Y: [%f2] Z: [%f2]", cal_pos[LEG_L1][COXA], cal_pos[LEG_L1][FEMUR], cal_pos[LEG_L1][TIBIA]);
    ROS_INFO("L2: X: [%f2] Y: [%f2] Z: [%f2]", cal_pos[LEG_L2][COXA], cal_pos[LEG_L2][FEMUR], cal_pos[LEG_L2][TIBIA]);
    ROS_INFO("L3: X: [%f2] Y: [%f2] Z: [%f2]", cal_pos[LEG_L3][COXA], cal_pos[LEG_L3][FEMUR], cal_pos[LEG_L3][TIBIA]);
    ROS_INFO("R1: X: [%f2] Y: [%f2] Z: [%f2]", cal_pos[LEG_R1][COXA], cal_pos[LEG_R1][FEMUR], cal_pos[LEG_R1][TIBIA]);
    ROS_INFO("R2: X: [%f2] Y: [%f2] Z: [%f2]", cal_pos[LEG_R2][COXA], cal_pos[LEG_R2][FEMUR], cal_pos[LEG_R2][TIBIA]);
    ROS_INFO("R3: X: [%f2] Y: [%f2] Z: [%f2]", cal_pos[LEG_R3][COXA], cal_pos[LEG_R3][FEMUR], cal_pos[LEG_R3][TIBIA]);
    ROS_INFO("------------------------");
    //public new data
    pub.publish(msg);

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
