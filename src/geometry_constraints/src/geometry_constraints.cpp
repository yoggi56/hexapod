#include "ros/ros.h"
#include <cmath>
#include "../../../devel/include/geometry_constraints/geom_constr.h"
#include <hexy_lib/hexy_lib.h>

using namespace std;

static double x_ok[6], y_ok[6], z_ok[6];

static double L1_legth, L2_legth, L3_legth;
static double body_width_small, body_width_big, body_length, body_heigth;

bool calc_constr(double x, double y, double z, int it)
{
  bool ok1 = false, ok3 = false, ok2 = false;
  double x_max = 0.0, y_max = 0.0, z_max = 0.0;
  double x_min = 0.0, y_min = 0.0, z_min = 0.0;

  // check number of a leg
  switch(it)
  {
  case LEG_L1:
    x_max = L1_X_MAX;
    y_max = L1_Y_MAX;
    z_max = L1_Z_MAX;
    x_min = L1_X_MIN;
    y_min = L1_Y_MIN;
    z_min = L1_Z_MIN;
    break;
  case LEG_L2:
    x_max = L2_X_MAX;
    y_max = L2_Y_MAX;
    z_max = L2_Z_MAX;
    x_min = L2_X_MIN;
    y_min = L2_Y_MIN;
    z_min = L2_Z_MIN;
    break;
  case LEG_L3:
    x_max = L3_X_MAX;
    y_max = L3_Y_MAX;
    z_max = L3_Z_MAX;
    x_min = L3_X_MIN;
    y_min = L3_Y_MIN;
    z_min = L3_Z_MIN;
    break;
  case LEG_R1:
    x_max = R1_X_MAX;
    y_max = R1_Y_MAX;
    z_max = R1_Z_MAX;
    x_min = R1_X_MIN;
    y_min = R1_Y_MIN;
    z_min = R1_Z_MIN;
    break;
  case LEG_R2:
    x_max = R2_X_MAX;
    y_max = R2_Y_MAX;
    z_max = R2_Z_MAX;
    x_min = R2_X_MIN;
    y_min = R2_Y_MIN;
    z_min = R2_Z_MIN;
    break;
  case LEG_R3:
    x_max = R3_X_MAX;
    y_max = R3_Y_MAX;
    z_max = R3_Z_MAX;
    x_min = R3_X_MIN;
    y_min = R3_Y_MIN;
    z_min = R3_Z_MIN;
    break;
  default:
    break;
  }

  // check X
  if (x >= x_min && x <= x_max)
  {
    x_ok[it] = x;
    ok1 = true;
  }
  else
  {
    ok1 = false;
    if (x > x_max)
      x_ok[it] = x_max;
    else if (x < x_min)
      x_ok[it] = x_min;
  }

  // check Y
  if (y >= y_min && y <= y_max)
  {
    y_ok[it] = y;
    ok2 = true;
  }
  else
  {
    ok2 = false;
    if (y > y_max)
      y_ok[it] = y_max;
    else if (y < y_min)
      y_ok[it] = y_min;
  }

  // check Z
  if (z >= z_min && z <= z_max)
  {
    z_ok[it] = z;
    ok3 = true;
  }
  else
  {
    ok3 = false;
    if (z > z_max)
      z_ok[it] = z_max;
    else if (z < z_min)
      z_ok[it] = z_min;
  }

  return ok1&ok2&ok3;
}

bool calc_R2_constr(double x, double y, double z)
{
  bool ok1 = false, ok2 = false, ok3 = false, ok4 = false;
  bool ok5 = false, ok6 = false, ok7 = false;

  // big sphere
  if ((pow(x,2) + pow(y+(body_width_big/2),2) + pow(z,2)) <= pow(0.231, 2))
    ok1 = true;

  // small sphere
  if ((pow(x,2) + pow(y+(body_width_big/2),2) + pow(z,2)) >= pow(0.106, 2))
    ok2 = true;

  // plane 3
  if (x <= body_length/2)
    ok3 = true;

  // plane 4
  if (x >= -body_length/2)
    ok4 = true;

  // plane 5
  if (z <= 0)
    ok5 = true;

  // plane 6
  if ((11.5*y + 1.005*z + 128.33) <= 0)
    ok6 = true;

  // plane 7
  if ((0.7*y + 1.12*z + 14.35) <= 0)
    ok7 = true;

  return ok1&ok2&ok3&ok4&ok5&ok6&ok7;
}

bool check_constr(geometry_constraints::geom_constr::Request  &req,
                  geometry_constraints::geom_constr::Response &res)
{
  ROS_INFO("INFO");
  for(int i = 0; i <= 5; i++)
  {
    res.ok[i] = calc_constr(req.x[i], req.y[i], req.z[i], i);
    res.x_out[i] = x_ok[i];
    res.y_out[i] = y_ok[i];
    res.z_out[i] = z_ok[i];

    ROS_INFO("request: x=%f, y=%f, z=%f", req.x[i], req.y[i], req.z[i]);
    ROS_INFO("sending back response: [%f] [%f] [%f] [%s]", x_ok[i], y_ok[i], z_ok[i], res.ok[i]?"True":"False");
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "geometry_constraints_server");
  ros::NodeHandle nh;

  // loading mechanical parameters

  ros::ServiceServer service = nh.advertiseService("geometry_constraints", check_constr);
  bool ok = false;
  if (nh.getParam("hexy/mech/L1_legth", L1_legth)) ok = true;
  if (nh.getParam("hexy/mech/L2_legth", L2_legth)) ok = true;
  if (nh.getParam("hexy/mech/L3_legth", L3_legth)) ok = true;
  if (nh.getParam("hexy/mech/body_width_small", body_width_small)) ok = true;
  if (nh.getParam("hexy/mech/body_width_big", body_width_big)) ok = true;
  if (nh.getParam("hexy/mech/body_length", body_length)) ok = true;
  if (nh.getParam("hexy/mech/body_heigth", body_heigth)) ok = true;

  if (!ok)
  {
    ROS_FATAL("leg_inv_kinematic: Some of the mechanical parameters weren't loaded. "
              "Check the file hexy_mechanical_parameters.yaml in the package hexy_lib.");
    return -1;
  }

  ROS_INFO("Ready to calculate geometry constraints");

  ros::spin();

  return 0;
}
