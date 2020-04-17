#include "ikine.h"

ikine::ikine()
{

}

vector < vector <double> > ikine::run(double const x[6], double const y[6], double const z[6])
{
  vector< std::vector<double> > result_joints(6, vector<double>(3));

  //  run_L1(x[LEG_L1], y[LEG_L1], z[LEG_L1]);
  //  run_L2(x[LEG_L2], y[LEG_L2], z[LEG_L2]);
  //  run_L3(x[LEG_L3], y[LEG_L3], z[LEG_L3]);
  //  run_R1(x[LEG_R1], y[LEG_R1], z[LEG_R1]);
  //  run_R2(x[LEG_R2], y[LEG_R2], z[LEG_R2]);
  //  run_R3(x[LEG_R3], y[LEG_R3], z[LEG_R3]);

  for(int i = 0; i < 6; i++)
  {
    run_legs(i, x[i], y[i], z[i]);
    result_joints[i][COXA] = joints[i][COXA];
    result_joints[i][FEMUR] = joints[i][FEMUR];
    result_joints[i][TIBIA] = joints[i][TIBIA];
  }
  return result_joints;
}

void ikine::set_mechanical_params(double L1_l, double L2_l, double L3_l, double bWs, double bWb, double bL, double bH)
{
  L1_length = L1_l;
  L2_length = L2_l;
  L3_length = L3_l;
  body_width_small = bWs;
  body_width_big = bWb;
  body_length = bL;
  body_heigth = bH;
}

void ikine::run_legs(int leg_number, double x, double y, double z)
{
  double P3Leg[3];
  double P03[4];
  double theta1_tmp;

  // transformation from body CoM to leg mount point dependinding on the leg number
  double sin_pi_4 = sin(PI/4.0);
  switch(leg_number)
  {
  case LEG_L1:
    P3Leg[0] = sin_pi_4 * (x + y - (body_width_small + body_length)/2.0);
    P3Leg[1] = sin_pi_4 * (-x + y - (body_width_small - body_length)/2.0);
    break;
  case LEG_L2:
    P3Leg[0] = abs(y) - body_width_big/2;
    P3Leg[1] = -x;
    break;
  case LEG_L3:
    P3Leg[0] = -sin_pi_4 * (x - y + (body_width_small + body_length)/2.0);
    P3Leg[1] = -sin_pi_4 * (x + y - (body_width_small - body_length)/2.0);
    break;
  case LEG_R1:
    P3Leg[0] = sin_pi_4 * (x - y - (body_width_small + body_length)/2.0);
    P3Leg[1] = sin_pi_4 * (x + y + (body_width_small - body_length)/2.0);
    break;
  case LEG_R2:
    P3Leg[0] = abs(y) - body_width_big/2;
    P3Leg[1] = x;
    break;
  case LEG_R3:
    P3Leg[0] = -sin_pi_4 * (x + y + (body_width_small + body_length)/2.0);
    P3Leg[1] = sin_pi_4 * (x - y - (body_width_small - body_length)/2.0);
    break;
  default:
    break;
  }
  P3Leg[2] = z;

  //calculate angle for coxa joint
  theta1_tmp = -atan2(P3Leg[1], P3Leg[0]);
  if (leg_number < 3)
    joints[leg_number][COXA] = theta1_tmp;
  else
    joints[leg_number][COXA] = -theta1_tmp;

  // transforamtion from leg mount point to femur joint
  P03[0] = P3Leg[0] * cos(theta1_tmp) - P3Leg[1] * sin(theta1_tmp);
  P03[1] = P3Leg[0] * sin(theta1_tmp) + P3Leg[1] * cos(theta1_tmp);
  P03[2] = P3Leg[2];

  //calculate angle for femur joint
  joints[leg_number][FEMUR] = (acos((pow(L2_length, 2) + pow((P03[0] - L1_length),2) + pow((P03[2]),2) - pow(L3_length,2)) /
      (2.0*L2_length*sqrt(pow((P03[0]-L1_length),2)+pow((P03[2]),2)))) -
      atan2(abs(P03[2]),(P03[0]-L1_length)));

  //calculate angle for tibia joint
  joints[leg_number][TIBIA] = -(acos((pow(L2_length,2) - pow((P03[0]-L1_length),2) - pow((P03[2]),2) + pow(L3_length,2))/(2.0*L2_length*L3_length)) - PI);

  //calculate angle constraints
  if (joints[leg_number][COXA] > 60.0*PI/180.0)
    joints[leg_number][COXA] = 60.0*PI/180.0;
  else if (joints[leg_number][COXA] < -60.0*PI/180.0)
    joints[leg_number][COXA] = -60.0*PI/180.0;

  if (joints[leg_number][FEMUR] > 70.0*PI/180.0)
    joints[leg_number][FEMUR] = 70.0*PI/180.0;
  else if (joints[leg_number][FEMUR] < -70.0*PI/180.0)
    joints[leg_number][FEMUR] = -70.0*PI/180.0;

  if (joints[leg_number][TIBIA] > 150.0*PI/180.0)
    joints[leg_number][TIBIA] = 150.0*PI/180.0;
  else if (joints[leg_number][TIBIA] < -150.0*PI/180.0)
    joints[leg_number][TIBIA] = -150.0*PI/180.0;
}

//void ikine::run_L1(double x, double y, double z)
//{
//  double P3Leg[3];
//  double P03[4];
//  double theta1_tmp;

//  // transformation from body CoM to leg mount point
//  double sin_pi_4 = sin(PI/4.0);
//  P3Leg[0] = sin_pi_4 * (x + y - (BODY_WIDTH_S + BODY_LENGTH)/2.0);
//  P3Leg[1] = sin_pi_4 * (-x + y - (BODY_WIDTH_S - BODY_LENGTH)/2.0);
//  P3Leg[2] = z;

//  //calculate angle for coxa joint
//  theta1_tmp = -atan2(P3Leg[1], P3Leg[0]);
//  joints[LEG_L1][COXA] = theta1_tmp;

//  // transforamtion from leg mount point to femur joint
//  P03[0] = P3Leg[0] * cos(theta1_tmp) - P3Leg[1] * sin(theta1_tmp);
//  P03[1] = P3Leg[0] * sin(theta1_tmp) + P3Leg[1] * cos(theta1_tmp);
//  P03[2] = P3Leg[2];

//  //calculate angle for femur joint
//  joints[LEG_L1][FEMUR] = (acos((pow(L2_LENGTH, 2) + pow((P03[0] - L1_LENGTH),2) + pow((P03[2]),2) - pow(L3_LENGTH,2)) /
//      (2.0*L2_LENGTH*sqrt(pow((P03[0]-L1_LENGTH),2)+pow((P03[2]),2)))) -
//      atan2(abs(P03[2]),(P03[0]-L1_LENGTH)));

//  //calculate angle for tibia joint
//  joints[LEG_L1][TIBIA] = -(acos((pow(L2_LENGTH,2) - pow((P03[0]-L1_LENGTH),2) - pow((P03[2]),2) + pow(L3_LENGTH,2))/(2.0*L2_LENGTH*L3_LENGTH)) - PI);

//  //calculate angle constraints
//  if (joints[LEG_L1][COXA] > 60.0*PI/180.0)
//    joints[LEG_L1][COXA] = 60.0*PI/180.0;
//  else if (joints[LEG_L1][COXA] < -60.0*PI/180.0)
//    joints[LEG_L1][COXA] = -60.0*PI/180.0;

//  if (joints[LEG_L1][FEMUR] > 70.0*PI/180.0)
//    joints[LEG_L1][FEMUR] = 70.0*PI/180.0;
//  else if (joints[LEG_L1][FEMUR] < -70.0*PI/180.0)
//    joints[LEG_L1][FEMUR] = -70.0*PI/180.0;

//  if (joints[LEG_L1][TIBIA] > 150.0*PI/180.0)
//    joints[LEG_L1][TIBIA] = 150.0*PI/180.0;
//  else if (joints[LEG_L1][TIBIA] < -150.0*PI/180.0)
//    joints[LEG_L1][TIBIA] = -150.0*PI/180.0;
//}

//void ikine::run_L2(double x, double y, double z)
//{
//  double P3Leg[3];
//  double P03[4];
//  double theta1_tmp;

//  // transformation from body CoM to leg mount point
//  P3Leg[0] = y - BODY_WIDTH_B/2;
//  P3Leg[1] = -x;
//  P3Leg[2] = z;

//  //calculate angle for coxa joint
//  theta1_tmp = -atan2(P3Leg[1], P3Leg[0]);
//  joints[LEG_L2][COXA] = theta1_tmp;

//  // transforamtion from leg mount point to femur joint
//  P03[0] = P3Leg[0] * cos(theta1_tmp) - P3Leg[1] * sin(theta1_tmp);
//  P03[1] = P3Leg[0] * sin(theta1_tmp) + P3Leg[1] * cos(theta1_tmp);
//  P03[2] = P3Leg[2];

//  //calculate angle for femur joint
//  joints[LEG_L2][FEMUR] = (acos((pow(L2_LENGTH, 2) + pow((P03[0] - L1_LENGTH),2) + pow((P03[2]),2) - pow(L3_LENGTH,2)) /
//      (2.0*L2_LENGTH*sqrt(pow((P03[0]-L1_LENGTH),2)+pow((P03[2]),2)))) -
//      atan2(abs(P03[2]),(P03[0]-L1_LENGTH)));

//  //calculate angle for tibia joint
//  joints[LEG_L2][TIBIA] = -(acos((pow(L2_LENGTH,2) - pow((P03[0]-L1_LENGTH),2) - pow((P03[2]),2) + pow(L3_LENGTH,2))/(2.0*L2_LENGTH*L3_LENGTH)) - PI);

//  //calculate angle constraints
//  if (joints[LEG_L2][COXA] > 60.0*PI/180.0)
//    joints[LEG_L2][COXA] = 60.0*PI/180.0;
//  else if (joints[LEG_L2][COXA] < -60.0*PI/180.0)
//    joints[LEG_L2][COXA] = -60.0*PI/180.0;

//  if (joints[LEG_L2][FEMUR] > 70.0*PI/180.0)
//    joints[LEG_L2][FEMUR] = 70.0*PI/180.0;
//  else if (joints[LEG_L2][FEMUR] < -70.0*PI/180.0)
//    joints[LEG_L2][FEMUR] = -70.0*PI/180.0;

//  if (joints[LEG_L2][TIBIA] > 150.0*PI/180.0)
//    joints[LEG_L2][TIBIA] = 150.0*PI/180.0;
//  else if (joints[LEG_L2][TIBIA] < -150.0*PI/180.0)
//    joints[LEG_L2][TIBIA] = -150.0*PI/180.0;
//}

//void ikine::run_L3(double x, double y, double z)
//{
//  double P3Leg[3];
//  double P03[4];
//  double theta1_tmp;

//  // transformation from body CoM to leg mount point
//  double sin_pi_4 = sin(PI/4.0);
//  P3Leg[0] = -sin_pi_4 * (x - y + (BODY_WIDTH_S + BODY_LENGTH)/2.0);
//  P3Leg[1] = -sin_pi_4 * (x + y - (BODY_WIDTH_S - BODY_LENGTH)/2.0);
//  P3Leg[2] = z;

//  //calculate angle for coxa joint
//  theta1_tmp = -atan2(P3Leg[1], P3Leg[0]);
//  joints[LEG_L3][COXA] = theta1_tmp;

//  // transforamtion from leg mount point to femur joint
//  P03[0] = P3Leg[0] * cos(theta1_tmp) - P3Leg[1] * sin(theta1_tmp);
//  P03[1] = P3Leg[0] * sin(theta1_tmp) + P3Leg[1] * cos(theta1_tmp);
//  P03[2] = P3Leg[2];

//  //calculate angle for femur joint
//  joints[LEG_L3][FEMUR] = (acos((pow(L2_LENGTH, 2) + pow((P03[0] - L1_LENGTH),2) + pow((P03[2]),2) - pow(L3_LENGTH,2)) /
//      (2.0*L2_LENGTH*sqrt(pow((P03[0]-L1_LENGTH),2)+pow((P03[2]),2)))) -
//      atan2(abs(P03[2]),(P03[0]-L1_LENGTH)));

//  //calculate angle for tibia joint
//  joints[LEG_L3][TIBIA] = -(acos((pow(L2_LENGTH,2) - pow((P03[0]-L1_LENGTH),2) - pow((P03[2]),2) + pow(L3_LENGTH,2))/(2.0*L2_LENGTH*L3_LENGTH)) - PI);

//  //calculate angle constraints
//  if (joints[LEG_L3][COXA] > 60.0*PI/180.0)
//    joints[LEG_L3][COXA] = 60.0*PI/180.0;
//  else if (joints[LEG_L3][COXA] < -60.0*PI/180.0)
//    joints[LEG_L3][COXA] = -60.0*PI/180.0;

//  if (joints[LEG_L3][FEMUR] > 70.0*PI/180.0)
//    joints[LEG_L3][FEMUR] = 70.0*PI/180.0;
//  else if (joints[LEG_L3][FEMUR] < -70.0*PI/180.0)
//    joints[LEG_L3][FEMUR] = -70.0*PI/180.0;

//  if (joints[LEG_L3][TIBIA] > 150.0*PI/180.0)
//    joints[LEG_L3][TIBIA] = 150.0*PI/180.0;
//  else if (joints[LEG_L3][TIBIA] < -150.0*PI/180.0)
//    joints[LEG_L3][TIBIA] = -150.0*PI/180.0;
//}

//void ikine::run_R1(double x, double y, double z)
//{
//  double P3Leg[3];
//  double P03[4];
//  double theta1_tmp;

//  // transformation from body CoM to leg mount point
//  double sin_pi_4 = sin(PI/4.0);
//  P3Leg[0] = sin_pi_4 * (x - y - (BODY_WIDTH_S + BODY_LENGTH)/2.0);
//  P3Leg[1] = sin_pi_4 * (x + y + (BODY_WIDTH_S - BODY_LENGTH)/2.0);
//  P3Leg[2] = z;

//  //calculate angle for coxa joint
//  theta1_tmp = -atan2(P3Leg[1], P3Leg[0]);
//  joints[LEG_R1][COXA] = -theta1_tmp;

//  // transforamtion from leg mount point to femur joint
//  P03[0] = P3Leg[0] * cos(theta1_tmp) - P3Leg[1] * sin(theta1_tmp);
//  P03[1] = P3Leg[0] * sin(theta1_tmp) + P3Leg[1] * cos(theta1_tmp);
//  P03[2] = P3Leg[2];

//  //calculate angle for femur joint
//  joints[LEG_R1][FEMUR] = (acos((pow(L2_LENGTH, 2) + pow((P03[0] - L1_LENGTH),2) + pow((P03[2]),2) - pow(L3_LENGTH,2)) /
//      (2.0*L2_LENGTH*sqrt(pow((P03[0]-L1_LENGTH),2)+pow((P03[2]),2)))) -
//      atan2(abs(P03[2]),(P03[0]-L1_LENGTH)));

//  //calculate angle for tibia joint
//  joints[LEG_R1][TIBIA] = -(acos((pow(L2_LENGTH,2) - pow((P03[0]-L1_LENGTH),2) - pow((P03[2]),2) + pow(L3_LENGTH,2))/(2.0*L2_LENGTH*L3_LENGTH)) - PI);

//  //calculate angle constraints
//  if (joints[LEG_R1][COXA] > 60.0*PI/180.0)
//    joints[LEG_R1][COXA] = 60.0*PI/180.0;
//  else if (joints[LEG_R1][COXA] < -60.0*PI/180.0)
//    joints[LEG_R1][COXA] = -60.0*PI/180.0;

//  if (joints[LEG_R1][FEMUR] > 70.0*PI/180.0)
//    joints[LEG_R1][FEMUR] = 70.0*PI/180.0;
//  else if (joints[LEG_R1][FEMUR] < -70.0*PI/180.0)
//    joints[LEG_R1][FEMUR] = -70.0*PI/180.0;

//  if (joints[LEG_R1][TIBIA] > 150.0*PI/180.0)
//    joints[LEG_R1][TIBIA] = 150.0*PI/180.0;
//  else if (joints[LEG_R1][TIBIA] < -150.0*PI/180.0)
//    joints[LEG_R1][TIBIA] = -150.0*PI/180.0;
//}

//void ikine::run_R2(double x, double y, double z)
//{
//  double P3Leg[3];
//  double P03[4];
//  double theta1_tmp;

//  // transformation from body CoM to leg mount point
//  P3Leg[0] = abs(y) - BODY_WIDTH_B/2;
//  P3Leg[1] = x;
//  P3Leg[2] = z;

//  //calculate angle for coxa joint
//  theta1_tmp = -atan2(P3Leg[1], P3Leg[0]);
//  joints[LEG_R2][COXA] = -theta1_tmp;

//  // transforamtion from leg mount point to femur joint
//  P03[0] = P3Leg[0] * cos(theta1_tmp) - P3Leg[1] * sin(theta1_tmp);
//  P03[1] = P3Leg[0] * sin(theta1_tmp) + P3Leg[1] * cos(theta1_tmp);
//  P03[2] = P3Leg[2];

//  //calculate angle for femur joint
//  joints[LEG_R2][FEMUR] = (acos((pow(L2_LENGTH, 2) + pow((P03[0] - L1_LENGTH),2) + pow((P03[2]),2) - pow(L3_LENGTH,2)) /
//      (2.0*L2_LENGTH*sqrt(pow((P03[0]-L1_LENGTH),2)+pow((P03[2]),2)))) -
//      atan2(abs(P03[2]),(P03[0]-L1_LENGTH)));

//  //calculate angle for tibia joint
//  joints[LEG_R2][TIBIA] = -(acos((pow(L2_LENGTH,2) - pow((P03[0]-L1_LENGTH),2) - pow((P03[2]),2) + pow(L3_LENGTH,2))/(2.0*L2_LENGTH*L3_LENGTH)) - PI);

//  //calculate angle constraints
//  if (joints[LEG_R2][COXA] > 60.0*PI/180.0)
//    joints[LEG_R2][COXA] = 60.0*PI/180.0;
//  else if (joints[LEG_R2][COXA] < -60.0*PI/180.0)
//    joints[LEG_R2][COXA] = -60.0*PI/180.0;

//  if (joints[LEG_R2][FEMUR] > 70.0*PI/180.0)
//    joints[LEG_R2][FEMUR] = 70.0*PI/180.0;
//  else if (joints[LEG_R2][FEMUR] < -70.0*PI/180.0)
//    joints[LEG_R2][FEMUR] = -70.0*PI/180.0;

//  if (joints[LEG_R2][TIBIA] > 150.0*PI/180.0)
//    joints[LEG_R2][TIBIA] = 150.0*PI/180.0;
//  else if (joints[LEG_R2][TIBIA] < -150.0*PI/180.0)
//    joints[LEG_R2][TIBIA] = -150.0*PI/180.0;
//}

//void ikine::run_R3(double x, double y, double z)
//{
//  double P3Leg[3];
//  double P03[4];
//  double theta1_tmp;

//  // transformation from body CoM to leg mount point
//  double sin_pi_4 = sin(PI/4.0);
//  P3Leg[0] = -sin_pi_4 * (x + y + (BODY_WIDTH_S + BODY_LENGTH)/2.0);
//  P3Leg[1] = sin_pi_4 * (x - y - (BODY_WIDTH_S - BODY_LENGTH)/2.0);
//  P3Leg[2] = z;

//  //calculate angle for coxa joint
//  theta1_tmp = -atan2(P3Leg[1], P3Leg[0]);
//  joints[LEG_R3][COXA] = -theta1_tmp;

//  // transforamtion from leg mount point to femur joint
//  P03[0] = P3Leg[0] * cos(theta1_tmp) - P3Leg[1] * sin(theta1_tmp);
//  P03[1] = P3Leg[0] * sin(theta1_tmp) + P3Leg[1] * cos(theta1_tmp);
//  P03[2] = P3Leg[2];

//  //calculate angle for femur joint
//  joints[LEG_R3][FEMUR] = (acos((pow(L2_LENGTH, 2) + pow((P03[0] - L1_LENGTH),2) + pow((P03[2]),2) - pow(L3_LENGTH,2)) /
//      (2.0*L2_LENGTH*sqrt(pow((P03[0]-L1_LENGTH),2)+pow((P03[2]),2)))) -
//      atan2(abs(P03[2]),(P03[0]-L1_LENGTH)));

//  //calculate angle for tibia joint
//  joints[LEG_R3][TIBIA] = -(acos((pow(L2_LENGTH,2) - pow((P03[0]-L1_LENGTH),2) - pow((P03[2]),2) + pow(L3_LENGTH,2))/(2.0*L2_LENGTH*L3_LENGTH)) - PI);

//  //calculate angle constraints
//  if (joints[LEG_R3][COXA] > 60.0*PI/180.0)
//    joints[LEG_R3][COXA] = 60.0*PI/180.0;
//  else if (joints[LEG_R3][COXA] < -60.0*PI/180.0)
//    joints[LEG_R3][COXA] = -60.0*PI/180.0;

//  if (joints[LEG_R3][FEMUR] > 70.0*PI/180.0)
//    joints[LEG_R3][FEMUR] = 70.0*PI/180.0;
//  else if (joints[LEG_R3][FEMUR] < -70.0*PI/180.0)
//    joints[LEG_R3][FEMUR] = -70.0*PI/180.0;

//  if (joints[LEG_R3][TIBIA] > 150.0*PI/180.0)
//    joints[LEG_R3][TIBIA] = 150.0*PI/180.0;
//  else if (joints[LEG_R3][TIBIA] < -150.0*PI/180.0)
//    joints[LEG_R3][TIBIA] = -150.0*PI/180.0;
//}
