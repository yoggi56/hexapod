#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "../../../devel/include/leg_msgs/LegJointState.h"
#include "dynamixel_controllers/SetSpeed.h"

static double joints[3];

void chatterJoints(const leg_msgs::LegJointState::ConstPtr &state)
{

  joints[0]=state->joint[0];
  joints[1]=state->joint[1];
  joints[2]=state->joint[2];
  ROS_INFO("I've received : [%.2f] [%.2f] [%.2f]", state->joint[0], state->joint[1], state->joint[2]);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_control");
  ros::NodeHandle nh;
  ros::Duration(3).sleep();

  //ros::ServiceClient client = nh.serviceClient<dynamixel_controllers::SetSpeed>("/coxa_controller/set_speed");
  ros::Publisher pub_coxa= nh.advertise<std_msgs::Float64>("coxa_controller/command", 1);
  ros::Publisher pub_femur = nh.advertise<std_msgs::Float64>("femur_controller/command", 1);
  ros::Publisher pub_tibia = nh.advertise<std_msgs::Float64>("tibia_controller/command", 1);
  ros::Subscriber sub = nh.subscribe("joints_ref_angles", 1, chatterJoints);
  ros::Rate loop_rate(40);
  while (ros::ok())
  {
    std_msgs::Float64 msg_coxa, msg_femur, msg_tibia;
    msg_coxa.data=joints[0];
    msg_femur.data=joints[1];
    msg_tibia.data=joints[2];
    pub_coxa.publish(msg_coxa);
    pub_femur.publish(msg_femur);
    pub_tibia.publish(msg_tibia);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
