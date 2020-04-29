#ifndef HMOVING_H
#define HMOVING_H

#include <ros/ros.h>
#include <hexy_msgs/HexyRefVel.h>
#include <hexy_msgs/LegRefPos.h>
#include <hexy_msgs/LegCurPos.h>
#include <hexy_lib/hexy_lib.h>
#include <geometry_constraints/geom_constr.h>
#include <array>
#include <queue>

//-- idle constants
#define IDLE_H        (0.03)
#define IDLE_Z        (-0.09)
#define IDLE_Z_UP     (IDLE_Z+IDLE_H)

#define IDLE_L1_X     (0.194)
#define IDLE_L1_Y     (0.195)

#define IDLE_L2_X     (0)
#define IDLE_L2_Y     (0.229)

#define IDLE_L3_X     (-0.194)
#define IDLE_L3_Y     (0.195)

#define IDLE_R1_X     (0.194)
#define IDLE_R1_Y     (-0.195)

#define IDLE_R2_X     (0)
#define IDLE_R2_Y     (-0.229)

#define IDLE_R3_X     (-0.194)
#define IDLE_R3_Y     (-0.195)

//-- sleeping contants
#define SLEEP_H        (0.03)
#define SLEEP_Z        (-0.09)
#define SLEEP_Z_UP     (IDLE_Z+IDLE_H)

#define SLEEP_L1_X     (0.194)
#define SLEEP_L1_Y     (0.195)

#define SLEEP_L2_X     (0)
#define SLEEP_L2_Y     (0.229)

#define SLEEP_L3_X     (-0.194)
#define SLEEP_L3_Y     (0.195)

#define SLEEP_R1_X     (0.194)
#define SLEEP_R1_Y     (-0.195)

#define SLEEP_R2_X     (0)
#define SLEEP_R2_Y     (-0.229)

#define SLEEP_R3_X     (-0.194)
#define SLEEP_R3_Y     (-0.195)

//-- go idle states --//
#define triple1_B 1
#define triple1_C 2
#define triple1_D 3
#define triple2_B 4
#define triple2_C 5
#define triple2_D 6
#define FINISH    7
#define GO_DOWN   8

#define pointA 0
#define pointB 1
#define pointC 2
#define pointD 3
//#define pointB2 4
//#define pointC2 5
//#define pointD2 6

struct vector_3D
{
  double x, y, z;
};

struct vector_2D
{
  double x, y;
};

class hmoving
{
public:
  hmoving(ros::NodeHandle *nh);
  void refresh_data();

private:
  ros::Subscriber command_sub;
  ros::Subscriber fkine_sub;
  ros::ServiceClient geom_constr_client;
  ros::Publisher pub;
  hexy_msgs::HexyRefVel ref_vel_msg;
  hexy_msgs::LegRefPos ref_pos_msg;
  geometry_constraints::geom_constr srv;

  void ref_vel_callback(const hexy_msgs::HexyRefVel::ConstPtr &msg);
  void fkine_callback(const hexy_msgs::LegCurPos::ConstPtr &msg);
  void state_switcher();
  void check_position();
  void go_idle();
  void go_sleep();
  void leg_moving();
  void body_moving();
  void walk_tripod_static();
  void remain_position();
  void prepare_for_idle();
  void prepare_for_sleep();
  void assign_pos(vector_3D &pos, double x, double y, double z);
  void assign_pos(vector_2D &pos, double x, double y);
  vector_3D assign_pos(double x, double y, double z);
  bool go_point_line(short point1, short point2, short legs, double incr);

  std::array<vector_3D, 6> vel_leg;
  vector_3D vel_body_transl;
  vector_3D vel_body_rotate;
  vector_2D vel_walk_dir;
  double vel_walk_rotate;
  short walk_type;

  std::array<vector_3D, 6> leg_cur_pos;
  std::array<vector_3D, 6> leg_ref_pos;

  short robot_command;
  short robot_state;

  bool isIdle, isSleep;
  bool idle_prepared, sleep_prepared, triple1_c_prep;

  //std::array<std::queue<vector_3D>, 6> leg_traj;
  std::array<std::array<vector_3D, 4>, 6> leg_traj;
  std::array<vector_3D, 6> ref_point;
  std::array<vector_3D, 6> error;
  short go_idle_state, go_sleep_state;
  bool leg_ready[6];

  double leg_vel;

  bool gazebo_sim;

  double t;
  bool just_on;
};

#endif // HMOVING_H
