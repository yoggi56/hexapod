#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <hexy_lib/hexy_lib.h>
#include <hexy_msgs/LegRefJoints.h>

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hexy_gazebo_publisher");
  ros::NodeHandle nh;

  // publishers for Gazebo command topics
  ros::Publisher L1_coxa_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_coxa_L1_position_controller/command", 1000);
  ros::Publisher L2_coxa_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_coxa_L2_position_controller/command", 1000);
  ros::Publisher L3_coxa_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_coxa_L3_position_controller/command", 1000);
  ros::Publisher R1_coxa_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_coxa_R1_position_controller/command", 1000);
  ros::Publisher R2_coxa_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_coxa_R2_position_controller/command", 1000);
  ros::Publisher R3_coxa_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_coxa_R3_position_controller/command", 1000);

  ros::Publisher L1_femur_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_femur_L1_position_controller/command", 1000);
  ros::Publisher L2_femur_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_femur_L2_position_controller/command", 1000);
  ros::Publisher L3_femur_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_femur_L3_position_controller/command", 1000);
  ros::Publisher R1_femur_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_femur_R1_position_controller/command", 1000);
  ros::Publisher R2_femur_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_femur_R2_position_controller/command", 1000);
  ros::Publisher R3_femur_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_femur_R3_position_controller/command", 1000);

  ros::Publisher L1_tibia_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_tibia_L1_position_controller/command", 1000);
  ros::Publisher L2_tibia_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_tibia_L2_position_controller/command", 1000);
  ros::Publisher L3_tibia_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_tibia_L3_position_controller/command", 1000);
  ros::Publisher R1_tibia_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_tibia_R1_position_controller/command", 1000);
  ros::Publisher R2_tibia_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_tibia_R2_position_controller/command", 1000);
  ros::Publisher R3_tibia_pos_pub = nh.advertise<std_msgs::Float64>("/hexapod_scarab/joint_tibia_R3_position_controller/command", 1000);

  ros::Subscriber sub = nh.subscribe("joints_ref_angles", 1, chatterLegsState);

  std_msgs::Float64 msg;

  ros::Rate loop_rate(40);
  while (ros::ok())
  {
    msg.data = L1_joint1;
    L1_coxa_pos_pub.publish(msg);
    msg.data = L2_joint1;
    L2_coxa_pos_pub.publish(msg);
    msg.data = L3_joint1;
    L3_coxa_pos_pub.publish(msg);
    msg.data = R1_joint1;
    R1_coxa_pos_pub.publish(msg);
    msg.data = R2_joint1;
    R2_coxa_pos_pub.publish(msg);
    msg.data = R3_joint1;
    R3_coxa_pos_pub.publish(msg);

    msg.data = L1_joint2;
    L1_femur_pos_pub.publish(msg);
    msg.data = L2_joint2;
    L2_femur_pos_pub.publish(msg);
    msg.data = L3_joint2;
    L3_femur_pos_pub.publish(msg);
    msg.data = R1_joint2;
    R1_femur_pos_pub.publish(msg);
    msg.data = R2_joint2;
    R2_femur_pos_pub.publish(msg);
    msg.data = R3_joint2;
    R3_femur_pos_pub.publish(msg);

    msg.data = L1_joint3;
    L1_tibia_pos_pub.publish(msg);
    msg.data = L2_joint3;
    L2_tibia_pos_pub.publish(msg);
    msg.data = L3_joint3;
    L3_tibia_pos_pub.publish(msg);
    msg.data = R1_joint3;
    R1_tibia_pos_pub.publish(msg);
    msg.data = R2_joint3;
    R2_tibia_pos_pub.publish(msg);
    msg.data = R3_joint3;
    R3_tibia_pos_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
