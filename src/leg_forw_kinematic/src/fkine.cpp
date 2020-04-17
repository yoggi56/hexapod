#include "fkine.h"

fkine::fkine()
{

}

vector < vector <double> > fkine::run(double const joints[6][3])
{
  vector< std::vector<double> > result_position(6, vector<double>(3));

  run_L1(joints[LEG_L1][COXA], joints[LEG_L1][FEMUR], joints[LEG_L1][TIBIA]);
  run_L2(joints[LEG_L2][COXA], joints[LEG_L2][FEMUR], joints[LEG_L2][TIBIA]);
  run_L3(joints[LEG_L3][COXA], joints[LEG_L3][FEMUR], joints[LEG_L3][TIBIA]);
  run_R1(joints[LEG_R1][COXA], joints[LEG_R1][FEMUR], joints[LEG_R1][TIBIA]);
  run_R2(joints[LEG_R2][COXA], joints[LEG_R2][FEMUR], joints[LEG_R2][TIBIA]);
  run_R3(joints[LEG_R3][COXA], joints[LEG_R3][FEMUR], joints[LEG_R3][TIBIA]);

  for(int i = 0; i < 6; i++)
  {
    result_position[i][COXA] = position[i][COXA];
    result_position[i][FEMUR] = position[i][FEMUR];
    result_position[i][TIBIA] = position[i][TIBIA];
  }
  return result_position;
}

void fkine::set_mechanical_params(double L1_l, double L2_l, double L3_l, double bWs, double bWb, double bL, double bH)
{
  L1_length = L1_l;
  L2_length = L2_l;
  L3_length = L3_l;
  body_width_small = bWs;
  body_width_big = bWb;
  body_length = bL;
  body_heigth = bH;
}

void fkine::run_L1(double coxa, double femur, double tibia)
{
  position[LEG_L1][EP_X] = body_length/2 + L3_length*((sqrt(2)*cos(coxa)*cos(femur - tibia))/2 - (sqrt(2)*sin(coxa)*cos(femur - tibia))/2) +
      (sqrt(2)*cos(coxa)*(L1_length + L2_length*cos(femur)))/2 - (sqrt(2)*sin(coxa)*(L1_length + L2_length*cos(femur)))/2;

  position[LEG_L1][EP_Y] = body_width_small/2 + L3_length*((sqrt(2)*cos(coxa)*cos(femur - tibia))/2 + (sqrt(2)*sin(coxa)*cos(femur - tibia))/2) +
      (sqrt(2)*cos(coxa)*(L1_length + L2_length*cos(femur)))/2 + (sqrt(2)*sin(coxa)*(L1_length + L2_length*cos(femur)))/2;

  position[LEG_L1][EP_Z] = L2_length*sin(femur) + L3_length*sin(femur - tibia);
}

void fkine::run_L2(double coxa, double femur, double tibia)
{
  position[LEG_L2][EP_X] = sin(coxa)*(L3_length*cos(femur - tibia) + cos(femur)*L2_length + L1_length);
  position[LEG_L2][EP_Y] = body_width_big/2 + cos(coxa)*(L1_length + L2_length*cos(femur)) + L3_length*cos(coxa)*cos(femur - tibia);
  position[LEG_L2][EP_Z] = L2_length*sin(femur) + L3_length*sin(femur - tibia);
}

void fkine::run_L3(double coxa, double femur, double tibia)
{
  position[LEG_L3][EP_X] = -body_length/2 - L3_length*((sqrt(2)*cos(coxa)*cos(femur - tibia))/2 + (sqrt(2)*sin(coxa)*cos(femur - tibia))/2) -
      (sqrt(2)*cos(coxa)*(L1_length + L2_length*cos(femur)))/2 - (sqrt(2)*sin(coxa)*(L1_length + L2_length*cos(femur)))/2;

  position[LEG_L3][EP_Y] = body_width_small/2 + L3_length*((sqrt(2)*cos(coxa)*cos(femur - tibia))/2 - (sqrt(2)*sin(coxa)*cos(femur - tibia))/2) +
      (sqrt(2)*cos(coxa)*(L1_length + L2_length*cos(femur)))/2 - (sqrt(2)*sin(coxa)*(L1_length + L2_length*cos(femur)))/2;

  position[LEG_L3][EP_Z] = L2_length*sin(femur) + L3_length*sin(femur - tibia);
}

void fkine::run_R1(double coxa, double femur, double tibia)
{
  position[LEG_R1][EP_X] = body_length/2 + L3_length*((sqrt(2)*cos(femur - tibia)*cos(coxa))/2 + (sqrt(2)*cos(femur - tibia)*sin(coxa))/2) +
      (sqrt(2)*cos(coxa)*(L1_length + L2_length*cos(femur)))/2 + (sqrt(2)*sin(coxa)*(L1_length + L2_length*cos(femur)))/2;

  position[LEG_R1][EP_Y] = (sqrt(2)*sin(coxa)*(L1_length + L2_length*cos(femur)))/2 -
      L3_length*((sqrt(2)*cos(femur - tibia)*cos(coxa))/2 - (sqrt(2)*cos(femur - tibia)*sin(coxa))/2) -
      (sqrt(2)*cos(coxa)*(L1_length + L2_length*cos(femur)))/2 - body_width_small/2;

  position[LEG_R1][EP_Z] = L2_length*sin(femur) + L3_length*sin(femur - tibia);
}

void fkine::run_R2(double coxa, double femur, double tibia)
{
  position[LEG_R2][EP_X] = sin(coxa)*(L3_length*cos(femur - tibia) + cos(femur)*L2_length + L1_length);
  //sin(u(1))*(L3*       sin(u(2)-u(3))     + cos(u(2)) *L2+         L1);
  position[LEG_R2][EP_Y] = cos(coxa)*(-L3_length*cos(femur - tibia) - cos(femur)*L2_length - L1_length) - body_width_big/2;
  position[LEG_R2][EP_Z] = L3_length*sin(femur - tibia) + sin(femur)*L2_length;

  //sin(u(1))*(L3*sin(u(2)-u(3))+cos(u(2))*L2+L1);
  //cos(u(1))*(-L3*sin(u(2)-u(3))-cos(u(2))*L2-L1)-bWb/2;
  //-L3*cos(u(2)-u(3))+sin(u(2))*L2;
}

void fkine::run_R3(double coxa, double femur, double tibia)
{
  position[LEG_R3][EP_X] = (sqrt(2)*sin(coxa)*(L1_length + L2_length*cos(femur)))/2 -
      L3_length*((sqrt(2)*cos(femur - tibia)*cos(coxa))/2 - (sqrt(2)*cos(femur - tibia)*sin(coxa))/2) -
      (sqrt(2)*cos(coxa)*(L1_length + L2_length*cos(femur)))/2 - body_length/2;

  position[LEG_R3][EP_Y] = -body_width_small/2 - L3_length*((sqrt(2)*cos(femur - tibia)*cos(coxa))/2 + (sqrt(2)*cos(femur - tibia)*sin(coxa))/2) -
      (sqrt(2)*cos(coxa)*(L1_length + L2_length*cos(femur)))/2 - (sqrt(2)*sin(coxa)*(L1_length + L2_length*cos(femur)))/2;

  position[LEG_R3][EP_Z] = L2_length*sin(femur) + L3_length*sin(femur - tibia);
}
