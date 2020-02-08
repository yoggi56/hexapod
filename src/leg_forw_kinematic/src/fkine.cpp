#include "fkine.h"

fkine::fkine()
{

}

vector < vector <double> > fkine::run(double const joints[6][3])
{
  vector< std::vector<double> > result_position(6, vector<double>(3));

  run_R2(joints[LEG_R2][COXA], joints[LEG_R2][FEMUR], joints[LEG_R2][TIBIA]);
  result_position[LEG_R2][COXA] = position[LEG_R2][COXA];
  result_position[LEG_R2][FEMUR] = position[LEG_R2][FEMUR];
  result_position[LEG_R2][TIBIA] = position[LEG_R2][TIBIA];

  return result_position;
}

void fkine::run_R2(double coxa, double femur, double tibia)
{
  position[LEG_R2][EP_X] = sin(coxa)*(L3_LENGTH*cos(femur - tibia) + cos(femur)*L2_LENGTH + L1_LENGTH);
                           //sin(u(1))*(L3*       sin(u(2)-u(3))     + cos(u(2)) *L2+         L1);
  position[LEG_R2][EP_Y] = cos(coxa)*(-L3_LENGTH*cos(femur - tibia) - cos(femur)*L2_LENGTH - L1_LENGTH) - BODY_WIDTH_B/2;
  position[LEG_R2][EP_Z] = L3_LENGTH*sin(femur - tibia) + sin(femur)*L2_LENGTH;

  //sin(u(1))*(L3*sin(u(2)-u(3))+cos(u(2))*L2+L1);
  //cos(u(1))*(-L3*sin(u(2)-u(3))-cos(u(2))*L2-L1)-bWb/2;
  //-L3*cos(u(2)-u(3))+sin(u(2))*L2;
}
