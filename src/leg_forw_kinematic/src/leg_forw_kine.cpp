#include "ros/ros.h"
#include "../../../devel/include/leg_msgs/LegCurAngles.h"
#include "../../../devel/include/leg_msgs/LegCurPos.h"

#include "fkine.h"

static double cur_joints[6][3];

// handler for topic /joints_cur_angles
void curAnglesCallback(const leg_msgs::LegCurAngles::ConstPtr &msg)
{
  cur_joints[LEG_R2][COXA] = msg->cur_joint[COXA];
  cur_joints[LEG_R2][FEMUR] = msg->cur_joint[FEMUR];
  cur_joints[LEG_R2][TIBIA] = msg->cur_joint[TIBIA];
  ROS_INFO("COXA: [%f2]", msg->cur_joint[COXA]);
  ROS_INFO("FEMUR: [%f2]", msg->cur_joint[FEMUR]);
  ROS_INFO("TIBIA: [%f2]", msg->cur_joint[TIBIA]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "leg_forw_kinematic");
  ros::NodeHandle nh;

  fkine fk;
  vector <vector <double> > cal_pos(6, vector<double>(3));

  ros::Subscriber sub = nh.subscribe("joints_cur_angles", 1000, curAnglesCallback);
  ros::Publisher pub = nh.advertise<leg_msgs::LegCurPos>("leg_cur_pos", 1000);
  leg_msgs::LegCurPos msg;

  ros::Rate rate(40);

  while(ros::ok())
  {
    //update forward kinematic data
    cal_pos = fk.run(cur_joints);
    msg.x = cal_pos[LEG_R2][COXA];
    msg.y = cal_pos[LEG_R2][FEMUR];
    msg.z = cal_pos[LEG_R2][TIBIA];

    ROS_INFO("X: [%f2]", cal_pos[LEG_R2][COXA]);
    ROS_INFO("Y: [%f2]", cal_pos[LEG_R2][FEMUR]);
    ROS_INFO("Z: [%f2]", cal_pos[LEG_R2][TIBIA]);

    //public new data
    pub.publish(msg);

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
