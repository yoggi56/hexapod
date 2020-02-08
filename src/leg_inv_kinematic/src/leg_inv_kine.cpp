#include "ros/ros.h"
//#include "std_msgs/String.h"
// This node compute inverse kinematic problem for right second leg of a hexapod robot with insect configuration

#include "leg_msgs/LegRefPos.h"
#include "leg_msgs/LegJointState.h"
#include "ikine.h"

static double x[6], y[6], z[6]; // reference coordinates for end effector

// handler for topic /leg_ref_pos
void chatterLegPos(const leg_msgs::LegRefPos::ConstPtr &state)
{
  x[LEG_R2] = state->x;
  y[LEG_R2] = state->y;
  z[LEG_R2] = state->z;
  //ROS_INFO("[%f]   [%f]   [%f]", x[LEG_R2], y[LEG_R2], z[LEG_R2]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "leg_inv_kinematic");

  ikine ik;
  vector <vector <double> > cal_joints(6, vector<double>(3));

  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<leg_msgs::LegJointState>("joints_ref_angles", 1000);
  leg_msgs::LegJointState msg;
  ros::Subscriber sub = n.subscribe("leg_ref_pos", 1, chatterLegPos);

  ros::Rate rate(40); //start publications with frequency 40Hz. According to Craig J. it's minimal frequency needed for smooth motion
  while(ros::ok())
  {
    // update inv kine data
    cal_joints = ik.run(x, y, z);
    ROS_INFO("[%f]   [%f]   [%f]", cal_joints[LEG_R2][COXA], cal_joints[LEG_R2][FEMUR], cal_joints[LEG_R2][TIBIA]);
    msg.joint[0] = double(cal_joints[LEG_R2][COXA]);
    msg.joint[1] = double(cal_joints[LEG_R2][FEMUR]);
    msg.joint[2] = double(cal_joints[LEG_R2][TIBIA]);

    // public new data
    joint_pub.publish(msg);

    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
