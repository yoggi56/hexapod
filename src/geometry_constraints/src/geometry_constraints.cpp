#include "ros/ros.h"
#include <cmath>
#include "../../../devel/include/geometry_constraints/geom_constr.h"

#define L1_LENGTH 0.034
#define L2_LENGTH 0.095
#define L3_LENGTH 0.134//0.142//0.13395
#define BODY_WIDTH_S 0.135
#define BODY_WIDTH_B 0.155
#define BODY_LENGTH 0.220
#define BODY_HEIGTH 0.04

#define PI 3.14159265

#define LEG_L1 0
#define LEG_L2 1
#define LEG_L3 2
#define LEG_R1 3
#define LEG_R2 4
#define LEG_R3 5

#define EP_X 0
#define EP_Y 1
#define EP_Z 2

#define COXA  0
#define FEMUR 1
#define TIBIA 2

#define H1_MIN    0
#define H1_MAX (L1_LENGTH+L2_LENGTH+L3_LENGTH)
#define A1_MAX (PI/3.0)
#define H2_MIN 0.085
#define H2_MAX (L2_LENGTH+L3_LENGTH)
#define A2_MAX (120.0*PI/180.0)

using namespace std;

static double res_pos[3];

bool calc_constr(double x, double y, double z)
{
  double P3Leg[3];
  double P03[4];
  double theta1;
  double h1, a1, h2, a2;
  bool ok1, ok2;
  // transformation from body CoM to leg mount point
  P3Leg[0] = abs(y) - BODY_WIDTH_B/2;
  P3Leg[1] = x;
  P3Leg[2] = z;

  //calculate angle for coxa joint
  theta1 = atan2(P3Leg[1], P3Leg[0]);

  // transforamtion from leg mount point to femur joint
  P03[0] = P3Leg[0] * cos(theta1) - P3Leg[1] * sin(theta1) - L1_LENGTH;
  P03[1] = P3Leg[0] * sin(theta1) + P3Leg[1] * cos(theta1);
  P03[2] = P3Leg[2];

  h1 = sqrt(pow(P3Leg[0], 2) + pow(P3Leg[1], 2));
  a1 = asin(P3Leg[1]/h1);

  h2 = sqrt(pow(P03[2], 2) + pow(P03[1], 2));
  a2 = asin(P03[1]/h1);

  if ((h1 >= H1_MIN && h1 <= H1_MAX) && (a1 <= A1_MAX && a1 >= -A1_MAX))
    ok1 = true;
  else
    ok1 = false;

  if ((h2 >= H2_MIN && h2 <= H2_MAX) && (a2 <= A2_MAX && a1 >= -A2_MAX))
    ok2 = true;
  else
    ok2 = false;

  res_pos[0] = x;
  res_pos[1] = y;
  res_pos[2] = z;
  return ok1 & ok2;
}

bool check_constr(geometry_constraints::geom_constr::Request  &req,
         geometry_constraints::geom_constr::Response &res)
{
  res.ok = calc_constr(req.x, req.y, req.z);
  res.x = res_pos[0];
  res.y = res_pos[1];
  res.z = res_pos[2];

  ROS_INFO("request: x=%f, y=%f, z=%f", req.x, req.y, req.z);
  ROS_INFO("sending back response: [%f], [%f], [%f], [%s]", res.x, res.y, res.z, res.ok?"True":"False");
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
