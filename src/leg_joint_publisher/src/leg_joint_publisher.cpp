// This node gets angle joints data and public them to robot_state_publisher

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "leg_msgs/LegJointState.h"

static double joint1, joint2, joint3; //angles of leg joints

// handler for topic /joints_state_publisher
void chatterLegsState(const leg_msgs::LegJointState::ConstPtr &state)
{

  joint1 = state->joint[0];
  joint2 = state->joint[1];
  joint3 = state->joint[2];
  ROS_INFO("I've received : [%.2f] [%.2f] [%.2f]", state->joint[0], state->joint[1], state->joint[2]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leg_joint_publisher");
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
    joint_msg.name.resize(3);
    joint_msg.position.resize(3);

    joint_msg.name[0] ="coxa_joint_R2";
    joint_msg.position[0] = joint1;
    joint_msg.name[1] ="femur_joint_R2";
    joint_msg.position[1] = joint2;
    joint_msg.name[2] ="tibia_joint_R2";
    joint_msg.position[2] = joint3;

    //send the joint state and transform
    joint_pub.publish(joint_msg);

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
