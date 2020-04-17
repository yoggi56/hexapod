#include "ros/ros.h"
#include <cmath>
#include "../../../devel/include/geometry_constraints/geom_constr.h"
#include <hexy_lib/hexy_lib.h>

//#define L1_LENGTH 0.034
//#define L2_LENGTH 0.095
//#define L3_LENGTH 0.134//0.142//0.13395
//#define BODY_WIDTH_S 0.135
//#define BODY_WIDTH_B 0.155
//#define BODY_LENGTH 0.220
//#define BODY_HEIGTH 0.04

//#define PI 3.14159265

//#define LEG_L1 0
//#define LEG_L2 1
//#define LEG_L3 2
//#define LEG_R1 3
//#define LEG_R2 4
//#define LEG_R3 5

//#define EP_X 0
//#define EP_Y 1
//#define EP_Z 2

//#define COXA  0
//#define FEMUR 1
//#define TIBIA 2

//#define H1_MIN    0
//#define H1_MAX (L1_LENGTH+L2_LENGTH+L3_LENGTH)
//#define A1_MAX (PI/3.0)
//#define H2_MIN 0.085
//#define H2_MAX (L2_LENGTH+L3_LENGTH)
//#define A2_MAX (120.0*PI/180.0)

//// --- xyz constraints for all the legs --- //
//#define L1_X_MAX 0.278
//#define L1_X_MIN 0.11
//#define L1_Y_MAX 0.27
//#define L1_Y_MIN 0.125
//#define L1_Z_MAX 0.0
//#define L1_Z_MIN -0.145

//#define L2_X_MAX 0.11
//#define L2_X_MIN -0.11
//#define L2_Y_MAX 0.297
//#define L2_Y_MIN 0.161
//#define L2_Z_MAX 0.0
//#define L2_Z_MIN -0.145

//#define L3_X_MAX -0.11
//#define L3_X_MIN -0.278
//#define L3_Y_MAX 0.27
//#define L3_Y_MIN 0.125
//#define L3_Z_MAX 0.0
//#define L3_Z_MIN -0.145

//#define R1_X_MAX 0.278
//#define R1_X_MIN 0.11
//#define R1_Y_MAX -0.125
//#define R1_Y_MIN -0.27
//#define R1_Z_MAX 0.0
//#define R1_Z_MIN -0.145

//#define R2_X_MAX 0.11
//#define R2_X_MIN -0.11
//#define R2_Y_MAX -0.161
//#define R2_Y_MIN -0.297
//#define R2_Z_MAX 0.0
//#define R2_Z_MIN -0.145

//#define R3_X_MAX -0.11
//#define R3_X_MIN -0.278
//#define R3_Y_MAX -0.125
//#define R3_Y_MIN -0.27
//#define R3_Z_MAX 0.0
//#define R3_Z_MIN -0.145
// --------------------------------------- //

using namespace std;

static double x_ok[6], y_ok[6], z_ok[6];

bool calc_constr(double x, double y, double z, int it)
{
  //  double P3Leg[3];
  //  double P03[4];
  //  double theta1;
  //  double h1, a1, h2, a2;
  //  bool ok;
  //  // transformation from body CoM to leg mount point
  //  P3Leg[0] = abs(y) - BODY_WIDTH_B/2;
  //  P3Leg[1] = x;
  //  P3Leg[2] = z;

  //  //calculate angle for coxa joint
  //  theta1 = -atan2(P3Leg[1], P3Leg[0]);

  //  // transforamtion from leg mount point to femur joint
  //  P03[0] = P3Leg[0] * cos(theta1) - P3Leg[1] * sin(theta1) - L1_LENGTH;
  //  P03[1] = P3Leg[0] * sin(theta1) + P3Leg[1] * cos(theta1);
  //  P03[2] = P3Leg[2];

  //  h1 = sqrt(pow(P3Leg[0], 2) + pow(P3Leg[1], 2));
  //  a1 = asin(P3Leg[1]/h1);

  //  h2 = sqrt(pow(P03[0], 2) + pow(P03[2], 2));
  //  a2 = asin(P03[0]/h1);

  //  if ((h1 >= H1_MIN && h1 <= H1_MAX) && (a1 <= A1_MAX && a1 >= -A1_MAX) &&
  //      ((h2 >= H2_MIN && h2 <= H2_MAX) && (a2 <= A2_MAX && a1 >= -A2_MAX)) &&
  //      y <= -BODY_WIDTH_B/2.0)
  //    ok = true;
  //  else
  //    ok = false;
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

  ros::ServiceServer service = nh.advertiseService("geometry_constraints", check_constr);
  ROS_INFO("Ready to calculate geomtry constraints");
  ros::spin();

  return 0;
}
