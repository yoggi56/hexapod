#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <hexy_msgs/LegRefJoints.h>
#include <dynamixel_controllers/SetSpeed.h>
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

static double joints[6][3];

void chatterJoints(const hexy_msgs::LegRefJoints::ConstPtr &state)
{
  for(int i = 0; i < 3; i++)
  {
    joints[LEG_L1][i]=state->L1_joint[i];
    joints[LEG_L2][i]=state->L2_joint[i];
    joints[LEG_L3][i]=state->L3_joint[i];
    joints[LEG_R1][i]=state->R1_joint[i];
    joints[LEG_R2][i]=state->R2_joint[i];
    joints[LEG_R3][i]=state->R3_joint[i];
  }
  ROS_INFO("I've received : [%.2f] [%.2f] [%.2f]", state->R3_joint[0], state->R3_joint[1], state->R3_joint[2]);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_write");
  ros::NodeHandle nh;
  ros::Duration(3).sleep();

  //ros::ServiceClient client = nh.serviceClient<dynamixel_controllers::SetSpeed>("/coxa_controller/set_speed");
  ros::Publisher L1_pub_coxa  = nh.advertise<std_msgs::Float64>("L1_coxa/command", 1);
  ros::Publisher L1_pub_femur = nh.advertise<std_msgs::Float64>("L1_femur/command", 1);
  ros::Publisher L1_pub_tibia = nh.advertise<std_msgs::Float64>("L1_tibia/command", 1);
  ros::Publisher L2_pub_coxa  = nh.advertise<std_msgs::Float64>("L2_coxa/command", 1);
  ros::Publisher L2_pub_femur = nh.advertise<std_msgs::Float64>("L2_femur/command", 1);
  ros::Publisher L2_pub_tibia = nh.advertise<std_msgs::Float64>("L2_tibia/command", 1);
  ros::Publisher L3_pub_coxa  = nh.advertise<std_msgs::Float64>("L3_coxa/command", 1);
  ros::Publisher L3_pub_femur = nh.advertise<std_msgs::Float64>("L3_femur/command", 1);
  ros::Publisher L3_pub_tibia = nh.advertise<std_msgs::Float64>("L3_tibia/command", 1);
  ros::Publisher R1_pub_coxa  = nh.advertise<std_msgs::Float64>("R1_coxa/command", 1);
  ros::Publisher R1_pub_femur = nh.advertise<std_msgs::Float64>("R1_femur/command", 1);
  ros::Publisher R1_pub_tibia = nh.advertise<std_msgs::Float64>("R1_tibia/command", 1);
  ros::Publisher R2_pub_coxa  = nh.advertise<std_msgs::Float64>("R2_coxa/command", 1);
  ros::Publisher R2_pub_femur = nh.advertise<std_msgs::Float64>("R2_femur/command", 1);
  ros::Publisher R2_pub_tibia = nh.advertise<std_msgs::Float64>("R2_tibia/command", 1);
  ros::Publisher R3_pub_coxa  = nh.advertise<std_msgs::Float64>("R3_coxa/command", 1);
  ros::Publisher R3_pub_femur = nh.advertise<std_msgs::Float64>("R3_femur/command", 1);
  ros::Publisher R3_pub_tibia = nh.advertise<std_msgs::Float64>("R3_tibia/command", 1);

  ros::Subscriber sub = nh.subscribe("joints_ref_angles", 1000, chatterJoints);

  // angular speed inititalization
  double max_speed = 3.0;
  nh.setParam("/R1_coxa/joint_speed", max_speed);
  nh.setParam("/R2_coxa/joint_speed", max_speed);
  nh.setParam("/R3_coxa/joint_speed", max_speed);
  nh.setParam("/R1_coxa/joint_speed", max_speed);
  nh.setParam("/R2_coxa/joint_speed", max_speed);
  nh.setParam("/R2_femur/joint_speed", max_speed);
  nh.setParam("/R2_tibia/joint_speed", max_speed);
  nh.setParam("/R3_coxa/joint_speed", max_speed);

  ros::Rate loop_rate(40);
  while (ros::ok())
  {
    std_msgs::Float64 msg_R1[3], msg_R2[3], msg_R3[3];
    std_msgs::Float64 msg_L1[3], msg_L2[3], msg_L3[3];

    for (int i = 0; i < 3; i++)
    {
      msg_L1[i].data=joints[LEG_L1][i];
      msg_L2[i].data=joints[LEG_L2][i];
      msg_L3[i].data=joints[LEG_L3][i];
      msg_R1[i].data=joints[LEG_R1][i];
      msg_R2[i].data=joints[LEG_R2][i];
      msg_R3[i].data=joints[LEG_R3][i];
    }

    L1_pub_coxa.publish(msg_L1[COXA]);
    L1_pub_femur.publish(msg_L1[FEMUR]);
    L1_pub_tibia.publish(msg_L1[TIBIA]);
    L2_pub_coxa.publish(msg_L2[COXA]);
    L2_pub_femur.publish(msg_L2[FEMUR]);
    L2_pub_tibia.publish(msg_L2[TIBIA]);
    L3_pub_coxa.publish(msg_L3[COXA]);
    L3_pub_femur.publish(msg_L3[FEMUR]);
    L3_pub_tibia.publish(msg_L3[TIBIA]);
    R1_pub_coxa.publish(msg_R1[COXA]);
    R1_pub_femur.publish(msg_R1[FEMUR]);
    R1_pub_tibia.publish(msg_R1[TIBIA]);
    R2_pub_coxa.publish(msg_R2[COXA]);
    R2_pub_femur.publish(msg_R2[FEMUR]);
    R2_pub_tibia.publish(msg_R2[TIBIA]);
    R3_pub_coxa.publish(msg_R3[COXA]);
    R3_pub_femur.publish(msg_R3[FEMUR]);
    R3_pub_tibia.publish(msg_R3[TIBIA]);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
