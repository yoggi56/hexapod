#include "ikine.h"

ikine::ikine()
{

}

vector < vector <double> > ikine::run(double const x[6], double const y[6], double const z[6])
{
  vector< std::vector<double> > result_joints(6, vector<double>(3));

  run_R2(x[LEG_R2], y[LEG_R2], z[LEG_R2]);

  result_joints[LEG_R2][COXA] = joints[LEG_R2][COXA];
  result_joints[LEG_R2][FEMUR] = joints[LEG_R2][FEMUR];
  result_joints[LEG_R2][TIBIA] = joints[LEG_R2][TIBIA];

  return result_joints;

}

void ikine::run_R2(double x, double y, double z)
{
  double P3Leg[3];
  double P03[4];
  double theta1_tmp;

  // transformation from body CoM to leg mount point
  P3Leg[0] = abs(y) - BODY_WIDTH_B/2;
  P3Leg[1] = x;
  P3Leg[2] = z;

  //calculate angle for coxa joint
  theta1_tmp = -atan2(P3Leg[1], P3Leg[0]);
  joints[LEG_R2][COXA] = -theta1_tmp;

  // transforamtion from leg mount point to femur joint
  P03[0] = P3Leg[0] * cos(theta1_tmp) - P3Leg[1] * sin(theta1_tmp);
  P03[1] = P3Leg[0] * sin(theta1_tmp) + P3Leg[1] * cos(theta1_tmp);
  P03[2] = P3Leg[2];

  //calculate angle for femur joint
  joints[LEG_R2][FEMUR] = (acos((pow(L2_LENGTH, 2) + pow((P03[0] - L1_LENGTH),2) + pow((P03[2]),2) - pow(L3_LENGTH,2)) /
      (2.0*L2_LENGTH*sqrt(pow((P03[0]-L1_LENGTH),2)+pow((P03[2]),2)))) -
      atan2(abs(P03[2]),(P03[0]-L1_LENGTH)));

  //calculate angle for tibia joint
  joints[LEG_R2][TIBIA] = -(acos((pow(L2_LENGTH,2) - pow((P03[0]-L1_LENGTH),2) - pow((P03[2]),2) + pow(L3_LENGTH,2))/(2.0*L2_LENGTH*L3_LENGTH)) - PI);

  //calculate angle constraints
  if (joints[LEG_R2][COXA] > 60.0*PI/180.0)
    joints[LEG_R2][COXA] = 60.0*PI/180.0;
  else if (joints[LEG_R2][COXA] < -60.0*PI/180.0)
    joints[LEG_R2][COXA] = -60.0*PI/180.0;

  if (joints[LEG_R2][FEMUR] > 70.0*PI/180.0)
    joints[LEG_R2][FEMUR] = 70.0*PI/180.0;
  else if (joints[LEG_R2][FEMUR] < -70.0*PI/180.0)
    joints[LEG_R2][FEMUR] = -70.0*PI/180.0;

  if (joints[LEG_R2][TIBIA] > 150.0*PI/180.0)
    joints[LEG_R2][TIBIA] = 150.0*PI/180.0;
  else if (joints[LEG_R2][TIBIA] < -150.0*PI/180.0)
    joints[LEG_R2][TIBIA] = -150.0*PI/180.0;
}
