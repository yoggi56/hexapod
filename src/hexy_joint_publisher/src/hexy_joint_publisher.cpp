// This node gets angle joints data and public them to robot_state_publisher

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <hexy_msgs/LegRefJoints.h>
#include <hexy_lib/hexy_lib.h>

static double R1_joint1, R1_joint2, R1_joint3; //angles of leg joints
static double R2_joint1, R2_joint2, R2_joint3; //angles of leg joints
static double R3_joint1, R3_joint2, R3_joint3; //angles of leg joints

static double L1_joint1, L1_joint2, L1_joint3; //angles of leg joints
static double L2_joint1, L2_joint2, L2_joint3;
static double L3_joint1, L3_joint2, L3_joint3;

// handler for topic /joints_state_publisher
void chatterLegsState(const hexy_msgs::LegRefJoints::ConstPtr &state)
{

  R1_joint1 = state->R1_joint[COXA];
  R1_joint2 = state->R1_joint[FEMUR];
  R1_joint3 = state->R1_joint[TIBIA];

  R2_joint1 = state->R2_joint[COXA];
  R2_joint2 = state->R2_joint[FEMUR];
  R2_joint3 = state->R2_joint[TIBIA];

  R3_joint1 = state->R3_joint[COXA];
  R3_joint2 = state->R3_joint[FEMUR];
  R3_joint3 = state->R3_joint[TIBIA];

  L1_joint1 = state->L1_joint[COXA];
  L1_joint2 = state->L1_joint[FEMUR];
  L1_joint3 = state->L1_joint[TIBIA];

  L2_joint1 = state->L2_joint[COXA];
  L2_joint2 = state->L2_joint[FEMUR];
  L2_joint3 = state->L2_joint[TIBIA];

  L3_joint1 = state->L3_joint[COXA];
  L3_joint2 = state->L3_joint[FEMUR];
  L3_joint3 = state->L3_joint[TIBIA];

  ROS_INFO("I've received : [%.2f] [%.2f] [%.2f]", state->L3_joint[0], state->L3_joint[1], state->L3_joint[2]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hexy_joint_publisher");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Subscriber sub = n.subscribe("joints_ref_angles", 1, chatterLegsState);
  sensor_msgs::JointState joint_msg;
  ros::Rate rate(40); //start publications with frequency 40Hz. According to Craig J. it's minimal frequency needed for smooth motion

  while(ros::ok())
  {
    //update joints
    joint_msg.header.stamp = ros::Time::now();
    joint_msg.header.frame_id = "base_link";
    joint_msg.name.resize(18);
    joint_msg.position.resize(18);

    joint_msg.name[0] ="coxa_joint_R1";
    joint_msg.position[0] = R1_joint1;
    joint_msg.name[1] ="femur_joint_R1";
    joint_msg.position[1] = R1_joint2;
    joint_msg.name[2] ="tibia_joint_R1";
    joint_msg.position[2] = R1_joint3;

    joint_msg.name[3] ="coxa_joint_R2";
    joint_msg.position[3] = R2_joint1;
    joint_msg.name[4] ="femur_joint_R2";
    joint_msg.position[4] = R2_joint2;
    joint_msg.name[5] ="tibia_joint_R2";
    joint_msg.position[5] = R2_joint3;

    joint_msg.name[6] ="coxa_joint_R3";
    joint_msg.position[6] = R3_joint1;
    joint_msg.name[7] ="femur_joint_R3";
    joint_msg.position[7] = R3_joint2;
    joint_msg.name[8] ="tibia_joint_R3";
    joint_msg.position[8] = R3_joint3;

    joint_msg.name[9] ="coxa_joint_L1";
    joint_msg.position[9] = L1_joint1;
    joint_msg.name[10] ="femur_joint_L1";
    joint_msg.position[10] = L1_joint2;
    joint_msg.name[11] ="tibia_joint_L1";
    joint_msg.position[11] = L1_joint3;

    joint_msg.name[12] ="coxa_joint_L2";
    joint_msg.position[12] = L2_joint1;
    joint_msg.name[13] ="femur_joint_L2";
    joint_msg.position[13] = L2_joint2;
    joint_msg.name[14] ="tibia_joint_L2";
    joint_msg.position[14] = L2_joint3;

    joint_msg.name[15] ="coxa_joint_L3";
    joint_msg.position[15] = L3_joint1;
    joint_msg.name[16] ="femur_joint_L3";
    joint_msg.position[16] = L3_joint2;
    joint_msg.name[17] ="tibia_joint_L3";
    joint_msg.position[17] = L3_joint3;

    //send the joint state and transform
    joint_pub.publish(joint_msg);

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
