#ifndef IKINE_H
#define IKINE_H

#include <vector>
#include <cmath>

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

using namespace std;

class ikine
{
public:
  ikine();
  vector < vector <double> > run(double const x[6], double const y[6], double const z[6]);

private:
  void run_R2(double x, double y, double z);

  double joints[6][3];

};

#endif // IKINE_H
