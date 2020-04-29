#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <hexy_msgs/LegCurJoints.h>

static double cur_joints[18];

void states_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
  for(int i = 0; i < 18; i++)
    cur_joints[i] = msg->position[i];
  //ROS_INFO("I heard: [%2f]", msg->position[0]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hexy_gazebo_subscriber");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("hexapod_scarab/joint_states", 1, states_callback);
  ros::Publisher cur_angles_pub = nh.advertise<hexy_msgs::LegCurJoints>("joints_cur_angles", 1);
  hexy_msgs::LegCurJoints msg;

  ros::Rate rate(40);

  while(ros::ok())
  {
    for(int i = 0; i < 3; i++)
    {
      msg.L1_joint[i] = cur_joints[i + i*5];
      msg.L2_joint[i] = cur_joints[i+1 + i*5];
      msg.L3_joint[i] = cur_joints[i+2 + i*5];
      msg.R1_joint[i] = cur_joints[i+3 + i*5];
      msg.R2_joint[i] = cur_joints[i+4 + i*5];
      msg.R3_joint[i] = cur_joints[i+5 + i*5];
    }

    cur_angles_pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
