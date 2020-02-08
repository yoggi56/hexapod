#include <ros/ros.h>
//#include "std_msgs/Float64.h"
#include "../../../devel/include/leg_msgs/LegCurAngles.h"
#include "dynamixel_msgs/JointState.h"

static double cur_joints[3];

void chatterCoxa(const dynamixel_msgs::JointState &state)
{
  cur_joints[0] = state.current_pos;
  //ROS_INFO("coxa: [%.2f]", state.current_pos);
}

void chatterFemur(const dynamixel_msgs::JointState &state)
{
  cur_joints[1] = state.current_pos;
  //ROS_INFO("femur: [%.2f]", state.current_pos);
}

void chatterTibia(const dynamixel_msgs::JointState &state)
{
  cur_joints[2] = state.current_pos;
  //ROS_INFO("tibia: [%.2f]", state.current_pos);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_read");
  ros::NodeHandle nh;

  ros::Subscriber coxa_sub = nh.subscribe("coxa_controller/state", 1000, chatterCoxa);
  ros::Subscriber femur_sub = nh.subscribe("femur_controller/state", 1000, chatterFemur);
  ros::Subscriber tibia_sub = nh.subscribe("tibia_controller/state", 1000, chatterTibia);

  ros::Publisher cur_angles_pub = nh.advertise<leg_msgs::LegCurAngles>("joints_cur_angles", 1000);
  leg_msgs::LegCurAngles msg;
  ros::Rate rate(40);
  ROS_INFO("check");
  cur_joints[0] = 0;
  cur_joints[1] = 0;
  cur_joints[2] = 0;
  while(ros::ok())
  {
    msg.cur_joint[0] = cur_joints[0];
    msg.cur_joint[1] = cur_joints[1];
    msg.cur_joint[2] = cur_joints[2];
    //ROS_INFO("coxa: [%.3f]; femur: [%.3f]; tibia: [%.3f]", msg.cur_joint[0], msg.cur_joint[1], msg.cur_joint[2]);
    cur_angles_pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

}
