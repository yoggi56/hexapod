#ifndef IKINE_H
#define IKINE_H

#include <vector>
#include <cmath>
#include <hexy_lib/hexy_lib.h>

using namespace std;

class ikine
{
public:
  ikine();
  vector < vector <double> > run(double const x[6], double const y[6], double const z[6]);
  void set_mechanical_params(double L1_l, double L2_l, double L3_l, double bWs, double bWb, double bL, double bH);

private:
  void run_legs(int leg_number, double x, double y, double z);

  double joints[6][3];
  double L1_length;
  double L2_length;
  double L3_length;
  double body_width_small;
  double body_width_big;
  double body_length;
  double body_heigth;

};

#endif // IKINE_H
