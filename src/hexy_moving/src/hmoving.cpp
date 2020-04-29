#include "hmoving.h"

hmoving::hmoving(ros::NodeHandle *nh)
{
  just_on = true;
  gazebo_sim = 1;
  isIdle = 0;
  isSleep = 1;
  idle_prepared = 0;
  triple1_c_prep = 0;
  sleep_prepared = 0;
  go_idle_state = 0;
  robot_state = SLEEPING;
  leg_vel = 0.02;
  t = 0;
  for(int i = 0; i < 6; i++)
    assign_pos(leg_ref_pos[i], leg_cur_pos[i].x, leg_cur_pos[i].y, leg_cur_pos[i].z);

  pub = nh->advertise<hexy_msgs::LegRefPos>("leg_ref_pos", 1000);
  command_sub = nh->subscribe("robot_ref_vel", 1000, &hmoving::ref_vel_callback, this);
  fkine_sub = nh->subscribe("leg_cur_pos", 1000, &hmoving::fkine_callback, this);
  geom_constr_client = nh->serviceClient<geometry_constraints::geom_constr>("geometry_constraints");
}

void hmoving::ref_vel_callback(const hexy_msgs::HexyRefVel::ConstPtr &msg)
{
  for(int i = 0; i < 6; i++)
    assign_pos(vel_leg[i], msg->leg_x[i], msg->leg_y[i], msg->leg_z[i]);

  assign_pos(vel_body_transl, msg->body_transl[X], msg->body_transl[Y], msg->body_transl[Z]);
  assign_pos(vel_body_rotate, msg->body_rotate[X], msg->body_rotate[Y], msg->body_rotate[Z]);

  assign_pos(vel_walk_dir, msg->walk_dir[X], msg->walk_dir[Y]);
  walk_type = msg->walk_type;
  robot_command = msg->command;
}

void hmoving::fkine_callback(const hexy_msgs::LegCurPos::ConstPtr &msg)
{
  for(int i = 0; i < 6; i++)
    assign_pos(leg_cur_pos[i], msg->x[i], msg->y[i], msg->z[i]);
}

void hmoving::assign_pos(vector_3D &pos, double x, double y, double z)
{
  pos.x = x;
  pos.y = y;
  pos.z = z;
  //ROS_INFO("I'm int assign pos");
}

void hmoving::assign_pos(vector_2D &pos, double x, double y)
{
  pos.x = x;
  pos.y = y;
}

vector_3D hmoving::assign_pos(double x, double y, double z)
{
  vector_3D point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

bool hmoving::go_point_line(short point1, short point2, short legs, double incr)
{
  short n = 0; // number working legs
  short k = 0; // number points which were got
  for(int i = 0; i < 6; i++)
  {
    if (((legs >> i) & 1u) == 1 && leg_ready[i] == false)
    {
      n++;
      t += incr;
      if (abs(leg_ref_pos[i].x - leg_traj[i][point2].x) >  0.003 ||
          abs(leg_ref_pos[i].y - leg_traj[i][point2].y) >  0.003 ||
          abs(leg_ref_pos[i].z - leg_traj[i][point2].z) >  0.003)
      {
        leg_ref_pos[i].x = (leg_traj[i][point2].x - leg_traj[i][point1].x)*t + leg_traj[i][point1].x;
        leg_ref_pos[i].y = (leg_traj[i][point2].y - leg_traj[i][point1].y)*t + leg_traj[i][point1].y;
        leg_ref_pos[i].z = (leg_traj[i][point2].z - leg_traj[i][point1].z)*t + leg_traj[i][point1].z;
      }
      else
      {
        leg_ready[i] = true;
        k++;
      }
    }
  }

  if (n == k)
  {
    t = 0;
    for(int i = 0; i < 6; i++)
      leg_ready[i] = false;
    return true;
  }
  return false;
}

void hmoving::go_idle()
{
  if (!idle_prepared)
    prepare_for_idle();

  switch(go_idle_state)
  {
  case triple1_B:
    if (go_point_line(pointA, pointB, 0x15, leg_vel)) //legs 010101
      go_idle_state = triple1_C;
    break;
  case triple1_C:
    if (go_point_line(pointB, pointC, 0x15, leg_vel)) //legs 010101
      go_idle_state = triple1_D;
    break;
  case triple1_D:
    if (go_point_line(pointC, pointD, 0x15, leg_vel)) //legs 010101
      go_idle_state = triple2_B;
    break;
  case triple2_B:
    if (go_point_line(pointA, pointB, 0x2A, leg_vel)) //legs 101010
      go_idle_state = triple2_C;
    break;
  case triple2_C:
    if (go_point_line(pointB, pointC, 0x2A, leg_vel)) //legs 101010
      go_idle_state = triple2_D;
    break;
  case triple2_D:
    if (go_point_line(pointC, pointD, 0x2A, leg_vel)) //legs 101010
    {
      idle_prepared = false;
      isIdle = true;
      isSleep = false;
      go_idle_state = FINISH;
    }
    break;
  case FINISH:
    break;
  default:
    break;
  }
}

void hmoving::prepare_for_idle()
{
  /*   Trajectory of legs
   *   B ------------- C
   *    |             |
   *    |             |
   *   A ------------- D
   */
  // LEG L1
  leg_traj[LEG_L1][pointA] = assign_pos(leg_cur_pos[LEG_L1].x, leg_cur_pos[LEG_L1].y, leg_cur_pos[LEG_L1].z); // A
  leg_traj[LEG_L1][pointB] = assign_pos(leg_cur_pos[LEG_L1].x, leg_cur_pos[LEG_L1].y, L1_Z_MAX); // B1
  leg_traj[LEG_L1][pointC] = assign_pos(IDLE_L1_X, IDLE_L1_Y, L1_Z_MAX); // C1
  leg_traj[LEG_L1][pointD] = assign_pos(IDLE_L1_X, IDLE_L1_Y, IDLE_Z);  //D1

  // LEG L2
  leg_traj[LEG_L2][pointA] = assign_pos(leg_cur_pos[LEG_L2].x, leg_cur_pos[LEG_L2].y, leg_cur_pos[LEG_L2].z); // A
  leg_traj[LEG_L2][pointB] = assign_pos(leg_cur_pos[LEG_L2].x, leg_cur_pos[LEG_L2].y, L2_Z_MAX); // B2
  leg_traj[LEG_L2][pointC] = assign_pos(IDLE_L2_X, IDLE_L2_Y, L2_Z_MAX); // C2
  leg_traj[LEG_L2][pointD] = assign_pos(IDLE_L2_X, IDLE_L2_Y, IDLE_Z);  //D2

  // LEG L3
  leg_traj[LEG_L3][pointA] = assign_pos(leg_cur_pos[LEG_L3].x, leg_cur_pos[LEG_L3].y, leg_cur_pos[LEG_L3].z); // A
  leg_traj[LEG_L3][pointB] = assign_pos(leg_cur_pos[LEG_L3].x, leg_cur_pos[LEG_L3].y, L3_Z_MAX); // B
  leg_traj[LEG_L3][pointC] = assign_pos(IDLE_L3_X, IDLE_L3_Y, L3_Z_MAX); // C
  leg_traj[LEG_L3][pointD] = assign_pos(IDLE_L3_X, IDLE_L3_Y, IDLE_Z);  //D

  // LEG R1
  leg_traj[LEG_R1][pointA] = assign_pos(leg_cur_pos[LEG_R1].x, leg_cur_pos[LEG_R1].y, leg_cur_pos[LEG_R1].z); // A
  leg_traj[LEG_R1][pointB] = assign_pos(leg_cur_pos[LEG_R1].x, leg_cur_pos[LEG_R1].y, R1_Z_MAX); // B2
  leg_traj[LEG_R1][pointC] = assign_pos(IDLE_R1_X, IDLE_R1_Y, R1_Z_MAX); // C2
  leg_traj[LEG_R1][pointD] = assign_pos(IDLE_R1_X, IDLE_R1_Y, IDLE_Z);  //D2

  // LEG R2
  leg_traj[LEG_R2][pointA] = assign_pos(leg_cur_pos[LEG_R2].x, leg_cur_pos[LEG_R2].y, leg_cur_pos[LEG_R2].z); // A
  leg_traj[LEG_R2][pointB] = assign_pos(leg_cur_pos[LEG_R2].x, leg_cur_pos[LEG_R2].y, R2_Z_MAX); // B1
  leg_traj[LEG_R2][pointC] = assign_pos(IDLE_R2_X, IDLE_R2_Y, R2_Z_MAX); // C1
  leg_traj[LEG_R2][pointD] = assign_pos(IDLE_R2_X, IDLE_R2_Y, IDLE_Z);  //D1

  // LEG R3
  leg_traj[LEG_R3][pointA] = assign_pos(leg_cur_pos[LEG_R3].x, leg_cur_pos[LEG_R3].y, leg_cur_pos[LEG_R3].z); // A
  leg_traj[LEG_R3][pointB] = assign_pos(leg_cur_pos[LEG_R3].x, leg_cur_pos[LEG_R3].y, R3_Z_MAX); // B2
  leg_traj[LEG_R3][pointC] = assign_pos(IDLE_R3_X, IDLE_R3_Y, R3_Z_MAX); // C2
  leg_traj[LEG_R3][pointD] = assign_pos(IDLE_R3_X, IDLE_R3_Y, IDLE_Z);  //D2

  for (int i = 0; i < 6; i++)
    leg_ready[i] = false;

  for(int i = 0; i < 6; i++)
    leg_ref_pos[i].z = leg_cur_pos[i].z;

  go_idle_state = triple1_B;
  idle_prepared = '1';
}

void hmoving::go_sleep()
{
  if (!sleep_prepared)
    prepare_for_sleep();

  switch(go_sleep_state)
  {
  case triple1_B:
    if (go_point_line(pointA, pointB, 0x15, leg_vel)) //legs 010101
      go_sleep_state = triple1_C;
    break;
  case triple1_C:
    if (go_point_line(pointB, pointC, 0x15, leg_vel)) //legs 010101
      go_sleep_state = triple1_D;
    break;
  case triple1_D:
    if (go_point_line(pointC, pointD, 0x15, leg_vel)) //legs 010101
      go_sleep_state = triple2_B;
    break;
  case triple2_B:
    if (go_point_line(pointA, pointB, 0x2A, leg_vel)) //legs 101010
      go_sleep_state = triple2_C;
    break;
  case triple2_C:
    if (go_point_line(pointB, pointC, 0x2A, leg_vel)) //legs 101010
      go_sleep_state = triple2_D;
    break;
  case triple2_D:
    if (go_point_line(pointC, pointD, 0x2A, leg_vel)) //legs 101010
      go_sleep_state = GO_DOWN;
    break;
  case GO_DOWN:
    if (go_point_line(pointD, pointC, 0x3F, leg_vel)) //legs 101010
    {
      sleep_prepared = false;
      isSleep = true;
      isIdle = false;
      go_sleep_state = FINISH;
    }
    break;
  case FINISH:
    break;
  default:
    break;
  }
}

void hmoving::prepare_for_sleep()
{
  /*   Trajectory of legs
   *   B ------------- C
   *    |             |
   *    |             |
   *   A ------------- D
   */
  // LEG L1
  leg_traj[LEG_L1][pointA] = assign_pos(leg_cur_pos[LEG_L1].x, leg_cur_pos[LEG_L1].y, leg_cur_pos[LEG_L1].z); // A
  leg_traj[LEG_L1][pointB] = assign_pos(leg_cur_pos[LEG_L1].x, leg_cur_pos[LEG_L1].y, L1_Z_MAX); // B1
  leg_traj[LEG_L1][pointC] = assign_pos(L1_X_MIN, L1_Y_MIN, L1_Z_MAX); // C1
  leg_traj[LEG_L1][pointD] = assign_pos(L1_X_MIN, L1_Y_MIN, IDLE_Z);  //D1

  // LEG L2
  leg_traj[LEG_L2][pointA] = assign_pos(leg_cur_pos[LEG_L2].x, leg_cur_pos[LEG_L2].y, leg_cur_pos[LEG_L2].z); // A
  leg_traj[LEG_L2][pointB] = assign_pos(leg_cur_pos[LEG_L2].x, leg_cur_pos[LEG_L2].y, L2_Z_MAX); // B2
  leg_traj[LEG_L2][pointC] = assign_pos(0, L2_Y_MIN, L2_Z_MAX); // C2
  leg_traj[LEG_L2][pointD] = assign_pos(0, L2_Y_MIN, IDLE_Z);  //D2

  // LEG L3
  leg_traj[LEG_L3][pointA] = assign_pos(leg_cur_pos[LEG_L3].x, leg_cur_pos[LEG_L3].y, leg_cur_pos[LEG_L3].z); // A
  leg_traj[LEG_L3][pointB] = assign_pos(leg_cur_pos[LEG_L3].x, leg_cur_pos[LEG_L3].y, L3_Z_MAX); // B
  leg_traj[LEG_L3][pointC] = assign_pos(L3_X_MAX, L3_Y_MIN, L3_Z_MAX); // C
  leg_traj[LEG_L3][pointD] = assign_pos(L3_X_MAX, L3_Y_MIN, IDLE_Z);  //D

  // LEG R1
  leg_traj[LEG_R1][pointA] = assign_pos(leg_cur_pos[LEG_R1].x, leg_cur_pos[LEG_R1].y, leg_cur_pos[LEG_R1].z); // A
  leg_traj[LEG_R1][pointB] = assign_pos(leg_cur_pos[LEG_R1].x, leg_cur_pos[LEG_R1].y, R1_Z_MAX); // B2
  leg_traj[LEG_R1][pointC] = assign_pos(R1_X_MIN, R1_Y_MAX, R1_Z_MAX); // C2
  leg_traj[LEG_R1][pointD] = assign_pos(R1_X_MIN, R1_Y_MAX, IDLE_Z);  //D2

  // LEG R2
  leg_traj[LEG_R2][pointA] = assign_pos(leg_cur_pos[LEG_R2].x, leg_cur_pos[LEG_R2].y, leg_cur_pos[LEG_R2].z); // A
  leg_traj[LEG_R2][pointB] = assign_pos(leg_cur_pos[LEG_R2].x, leg_cur_pos[LEG_R2].y, R2_Z_MAX); // B1
  leg_traj[LEG_R2][pointC] = assign_pos(0, R2_Y_MAX, R2_Z_MAX); // C1
  leg_traj[LEG_R2][pointD] = assign_pos(0, R2_Y_MAX, IDLE_Z);  //D1

  // LEG R3
  leg_traj[LEG_R3][pointA] = assign_pos(leg_cur_pos[LEG_R3].x, leg_cur_pos[LEG_R3].y, leg_cur_pos[LEG_R3].z); // A
  leg_traj[LEG_R3][pointB] = assign_pos(leg_cur_pos[LEG_R3].x, leg_cur_pos[LEG_R3].y, R3_Z_MAX); // B2
  leg_traj[LEG_R3][pointC] = assign_pos(R3_X_MAX, R3_Y_MAX, R3_Z_MAX); // C2
  leg_traj[LEG_R3][pointD] = assign_pos(R3_X_MAX, R3_Y_MAX, IDLE_Z);  //D2

  for (int i = 0; i < 6; i++)
    leg_ready[i] = false;

  for(int i = 0; i < 6; i++)
    leg_ref_pos[i].z = leg_cur_pos[i].z;

  go_sleep_state = triple1_B;
  sleep_prepared = '1';
}

void hmoving::leg_moving()
{

}

void hmoving::body_moving()
{
    for(int i = 0; i < 6; i++)
    {
      leg_ref_pos[i].x -= vel_body_transl.x;
      leg_ref_pos[i].y -= vel_body_transl.y;
      leg_ref_pos[i].z -= vel_body_transl.z;

      leg_ref_pos[i].x = leg_ref_pos[i].x*cos(vel_body_rotate.z) - leg_ref_pos[i].y*sin(vel_body_rotate.z);
      leg_ref_pos[i].y = leg_ref_pos[i].x*sin(vel_body_rotate.z) + leg_ref_pos[i].y*cos(vel_body_rotate.z);

      leg_ref_pos[i].y = leg_ref_pos[i].y*cos(vel_body_rotate.x) -  leg_ref_pos[i].z*sin(vel_body_rotate.x);
      leg_ref_pos[i].z = leg_ref_pos[i].y*sin(vel_body_rotate.x) +  leg_ref_pos[i].z*cos(vel_body_rotate.x);

      leg_ref_pos[i].x = leg_ref_pos[i].x*cos(vel_body_rotate.y) +  leg_ref_pos[i].z*sin(vel_body_rotate.y);
      leg_ref_pos[i].z = -leg_ref_pos[i].x*sin(vel_body_rotate.y) +  leg_ref_pos[i].z*cos(vel_body_rotate.y);
    }
}

void hmoving::walk_tripod_static()
{

}

void hmoving::check_position()
{
  for(int i = 0; i < 6; i++)
  {
    srv.request.x[i] = leg_ref_pos[i].x;
    srv.request.y[i] = leg_ref_pos[i].y;
    srv.request.z[i] = leg_ref_pos[i].z;
  }
  if (geom_constr_client.call(srv))
  {
    for(int i = 0; i < 6; i++)
    {
      if (!srv.response.ok[i])
      {

        leg_ref_pos[i].x = srv.response.x_out[i];
        leg_ref_pos[i].y = srv.response.y_out[i];
        leg_ref_pos[i].z = srv.response.z_out[i];
      }
    }
  }
}

void hmoving::remain_position()
{
  double ref_z[6];

  if(gazebo_sim && just_on)
  {
    for(int i = 0; i < 6; i++)
        ref_z[i] = -0.1;
    just_on = false;
  }
  else
  {
    for(int i = 0; i < 6; i++)
        ref_z[i] = leg_cur_pos[i].z;
  }

  for(int i = 0; i < 6; i++)
    assign_pos(leg_ref_pos[i], leg_cur_pos[i].x, leg_cur_pos[i].y, ref_z[i]);
  //ROS_INFO("I'm in remain_position");
}

void hmoving::state_switcher()
{
  if (robot_command == GO_IDLE && isIdle == 0)
  {
    isSleep = 0;
    robot_state = GO_IDLE;
    ROS_INFO("'go idle' has been got");
  }
  else if (isIdle == 1)
    robot_state = MOVING;

  if (robot_command == GO_SLEEP && isSleep == 0)
  {
    robot_state = GO_SLEEP;
    isIdle = 0;
  }
  else if (isSleep == 1)
    robot_state = SLEEPING;
}

void hmoving::refresh_data()
{
  state_switcher();

  switch(robot_state)
  {
  case GO_IDLE:
    go_idle();
    //ROS_INFO("GO_IDLE");
    break;
  case GO_SLEEP:
    go_sleep();
    //ROS_INFO("GO_SLEEP");
    break;
  case MOVING:
    leg_moving();
    body_moving();
    walk_tripod_static();
    //ROS_INFO("MOVING");
    break;
  case SLEEPING:
    remain_position();
    //ROS_INFO("SLEEPING");
    break;
  default:
    break;
  }

  check_position();

  for(int i = 0; i < 6; i++)
  {
    ref_pos_msg.x[i] = leg_ref_pos[i].x;
    ref_pos_msg.y[i] = leg_ref_pos[i].y;
    ref_pos_msg.z[i] = leg_ref_pos[i].z;
  }

  pub.publish(ref_pos_msg);
  //  ROS_INFO("robot_state: [%d]", robot_state);
  //  ROS_INFO("ref: x: [%2f] y: [%2f] z: [%2f]", leg_ref_pos[LEG_L1].x, leg_ref_pos[LEG_L1].y, leg_ref_pos[LEG_L1].z);
  //  ROS_INFO("cur: x: [%2f] y: [%2f] z: [%2f]", leg_cur_pos[LEG_L1].x, leg_cur_pos[LEG_L1].y, leg_cur_pos[LEG_L1].z);
}
