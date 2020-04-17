#ifndef FKINE_H
#define FKINE_H

#include <vector>
#include <cmath>
#include <hexy_lib/hexy_lib.h>

using namespace std;

class fkine
{
public:
  fkine();
  vector < vector <double> > run(double const joints[6][3]);
  void set_mechanical_params(double L1_l, double L2_l, double L3_l, double bWs, double bWb, double bL, double bH);

private:
  void run_L1(double coxa, double femur, double tibia);
  void run_L2(double coxa, double femur, double tibia);
  void run_L3(double coxa, double femur, double tibia);
  void run_R1(double coxa, double femur, double tibia);
  void run_R2(double coxa, double femur, double tibia);
  void run_R3(double coxa, double femur, double tibia);

  double position[6][3];
  double L1_length;
  double L2_length;
  double L3_length;
  double body_width_small;
  double body_width_big;
  double body_length;
  double body_heigth;
};

#endif // FKINE_H
