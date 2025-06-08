#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1556314892176535768) {
   out_1556314892176535768[0] = delta_x[0] + nom_x[0];
   out_1556314892176535768[1] = delta_x[1] + nom_x[1];
   out_1556314892176535768[2] = delta_x[2] + nom_x[2];
   out_1556314892176535768[3] = delta_x[3] + nom_x[3];
   out_1556314892176535768[4] = delta_x[4] + nom_x[4];
   out_1556314892176535768[5] = delta_x[5] + nom_x[5];
   out_1556314892176535768[6] = delta_x[6] + nom_x[6];
   out_1556314892176535768[7] = delta_x[7] + nom_x[7];
   out_1556314892176535768[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6496479561687713128) {
   out_6496479561687713128[0] = -nom_x[0] + true_x[0];
   out_6496479561687713128[1] = -nom_x[1] + true_x[1];
   out_6496479561687713128[2] = -nom_x[2] + true_x[2];
   out_6496479561687713128[3] = -nom_x[3] + true_x[3];
   out_6496479561687713128[4] = -nom_x[4] + true_x[4];
   out_6496479561687713128[5] = -nom_x[5] + true_x[5];
   out_6496479561687713128[6] = -nom_x[6] + true_x[6];
   out_6496479561687713128[7] = -nom_x[7] + true_x[7];
   out_6496479561687713128[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6407376786895322314) {
   out_6407376786895322314[0] = 1.0;
   out_6407376786895322314[1] = 0.0;
   out_6407376786895322314[2] = 0.0;
   out_6407376786895322314[3] = 0.0;
   out_6407376786895322314[4] = 0.0;
   out_6407376786895322314[5] = 0.0;
   out_6407376786895322314[6] = 0.0;
   out_6407376786895322314[7] = 0.0;
   out_6407376786895322314[8] = 0.0;
   out_6407376786895322314[9] = 0.0;
   out_6407376786895322314[10] = 1.0;
   out_6407376786895322314[11] = 0.0;
   out_6407376786895322314[12] = 0.0;
   out_6407376786895322314[13] = 0.0;
   out_6407376786895322314[14] = 0.0;
   out_6407376786895322314[15] = 0.0;
   out_6407376786895322314[16] = 0.0;
   out_6407376786895322314[17] = 0.0;
   out_6407376786895322314[18] = 0.0;
   out_6407376786895322314[19] = 0.0;
   out_6407376786895322314[20] = 1.0;
   out_6407376786895322314[21] = 0.0;
   out_6407376786895322314[22] = 0.0;
   out_6407376786895322314[23] = 0.0;
   out_6407376786895322314[24] = 0.0;
   out_6407376786895322314[25] = 0.0;
   out_6407376786895322314[26] = 0.0;
   out_6407376786895322314[27] = 0.0;
   out_6407376786895322314[28] = 0.0;
   out_6407376786895322314[29] = 0.0;
   out_6407376786895322314[30] = 1.0;
   out_6407376786895322314[31] = 0.0;
   out_6407376786895322314[32] = 0.0;
   out_6407376786895322314[33] = 0.0;
   out_6407376786895322314[34] = 0.0;
   out_6407376786895322314[35] = 0.0;
   out_6407376786895322314[36] = 0.0;
   out_6407376786895322314[37] = 0.0;
   out_6407376786895322314[38] = 0.0;
   out_6407376786895322314[39] = 0.0;
   out_6407376786895322314[40] = 1.0;
   out_6407376786895322314[41] = 0.0;
   out_6407376786895322314[42] = 0.0;
   out_6407376786895322314[43] = 0.0;
   out_6407376786895322314[44] = 0.0;
   out_6407376786895322314[45] = 0.0;
   out_6407376786895322314[46] = 0.0;
   out_6407376786895322314[47] = 0.0;
   out_6407376786895322314[48] = 0.0;
   out_6407376786895322314[49] = 0.0;
   out_6407376786895322314[50] = 1.0;
   out_6407376786895322314[51] = 0.0;
   out_6407376786895322314[52] = 0.0;
   out_6407376786895322314[53] = 0.0;
   out_6407376786895322314[54] = 0.0;
   out_6407376786895322314[55] = 0.0;
   out_6407376786895322314[56] = 0.0;
   out_6407376786895322314[57] = 0.0;
   out_6407376786895322314[58] = 0.0;
   out_6407376786895322314[59] = 0.0;
   out_6407376786895322314[60] = 1.0;
   out_6407376786895322314[61] = 0.0;
   out_6407376786895322314[62] = 0.0;
   out_6407376786895322314[63] = 0.0;
   out_6407376786895322314[64] = 0.0;
   out_6407376786895322314[65] = 0.0;
   out_6407376786895322314[66] = 0.0;
   out_6407376786895322314[67] = 0.0;
   out_6407376786895322314[68] = 0.0;
   out_6407376786895322314[69] = 0.0;
   out_6407376786895322314[70] = 1.0;
   out_6407376786895322314[71] = 0.0;
   out_6407376786895322314[72] = 0.0;
   out_6407376786895322314[73] = 0.0;
   out_6407376786895322314[74] = 0.0;
   out_6407376786895322314[75] = 0.0;
   out_6407376786895322314[76] = 0.0;
   out_6407376786895322314[77] = 0.0;
   out_6407376786895322314[78] = 0.0;
   out_6407376786895322314[79] = 0.0;
   out_6407376786895322314[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6513789890728588369) {
   out_6513789890728588369[0] = state[0];
   out_6513789890728588369[1] = state[1];
   out_6513789890728588369[2] = state[2];
   out_6513789890728588369[3] = state[3];
   out_6513789890728588369[4] = state[4];
   out_6513789890728588369[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6513789890728588369[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6513789890728588369[7] = state[7];
   out_6513789890728588369[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6547009325775891168) {
   out_6547009325775891168[0] = 1;
   out_6547009325775891168[1] = 0;
   out_6547009325775891168[2] = 0;
   out_6547009325775891168[3] = 0;
   out_6547009325775891168[4] = 0;
   out_6547009325775891168[5] = 0;
   out_6547009325775891168[6] = 0;
   out_6547009325775891168[7] = 0;
   out_6547009325775891168[8] = 0;
   out_6547009325775891168[9] = 0;
   out_6547009325775891168[10] = 1;
   out_6547009325775891168[11] = 0;
   out_6547009325775891168[12] = 0;
   out_6547009325775891168[13] = 0;
   out_6547009325775891168[14] = 0;
   out_6547009325775891168[15] = 0;
   out_6547009325775891168[16] = 0;
   out_6547009325775891168[17] = 0;
   out_6547009325775891168[18] = 0;
   out_6547009325775891168[19] = 0;
   out_6547009325775891168[20] = 1;
   out_6547009325775891168[21] = 0;
   out_6547009325775891168[22] = 0;
   out_6547009325775891168[23] = 0;
   out_6547009325775891168[24] = 0;
   out_6547009325775891168[25] = 0;
   out_6547009325775891168[26] = 0;
   out_6547009325775891168[27] = 0;
   out_6547009325775891168[28] = 0;
   out_6547009325775891168[29] = 0;
   out_6547009325775891168[30] = 1;
   out_6547009325775891168[31] = 0;
   out_6547009325775891168[32] = 0;
   out_6547009325775891168[33] = 0;
   out_6547009325775891168[34] = 0;
   out_6547009325775891168[35] = 0;
   out_6547009325775891168[36] = 0;
   out_6547009325775891168[37] = 0;
   out_6547009325775891168[38] = 0;
   out_6547009325775891168[39] = 0;
   out_6547009325775891168[40] = 1;
   out_6547009325775891168[41] = 0;
   out_6547009325775891168[42] = 0;
   out_6547009325775891168[43] = 0;
   out_6547009325775891168[44] = 0;
   out_6547009325775891168[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6547009325775891168[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6547009325775891168[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6547009325775891168[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6547009325775891168[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6547009325775891168[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6547009325775891168[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6547009325775891168[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6547009325775891168[53] = -9.8000000000000007*dt;
   out_6547009325775891168[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6547009325775891168[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6547009325775891168[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6547009325775891168[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6547009325775891168[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6547009325775891168[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6547009325775891168[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6547009325775891168[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6547009325775891168[62] = 0;
   out_6547009325775891168[63] = 0;
   out_6547009325775891168[64] = 0;
   out_6547009325775891168[65] = 0;
   out_6547009325775891168[66] = 0;
   out_6547009325775891168[67] = 0;
   out_6547009325775891168[68] = 0;
   out_6547009325775891168[69] = 0;
   out_6547009325775891168[70] = 1;
   out_6547009325775891168[71] = 0;
   out_6547009325775891168[72] = 0;
   out_6547009325775891168[73] = 0;
   out_6547009325775891168[74] = 0;
   out_6547009325775891168[75] = 0;
   out_6547009325775891168[76] = 0;
   out_6547009325775891168[77] = 0;
   out_6547009325775891168[78] = 0;
   out_6547009325775891168[79] = 0;
   out_6547009325775891168[80] = 1;
}
void h_25(double *state, double *unused, double *out_7013224615583518476) {
   out_7013224615583518476[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5803457646806109888) {
   out_5803457646806109888[0] = 0;
   out_5803457646806109888[1] = 0;
   out_5803457646806109888[2] = 0;
   out_5803457646806109888[3] = 0;
   out_5803457646806109888[4] = 0;
   out_5803457646806109888[5] = 0;
   out_5803457646806109888[6] = 1;
   out_5803457646806109888[7] = 0;
   out_5803457646806109888[8] = 0;
}
void h_24(double *state, double *unused, double *out_6448602700168441562) {
   out_6448602700168441562[0] = state[4];
   out_6448602700168441562[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3868394019091270834) {
   out_3868394019091270834[0] = 0;
   out_3868394019091270834[1] = 0;
   out_3868394019091270834[2] = 0;
   out_3868394019091270834[3] = 0;
   out_3868394019091270834[4] = 1;
   out_3868394019091270834[5] = 0;
   out_3868394019091270834[6] = 0;
   out_3868394019091270834[7] = 0;
   out_3868394019091270834[8] = 0;
   out_3868394019091270834[9] = 0;
   out_3868394019091270834[10] = 0;
   out_3868394019091270834[11] = 0;
   out_3868394019091270834[12] = 0;
   out_3868394019091270834[13] = 0;
   out_3868394019091270834[14] = 1;
   out_3868394019091270834[15] = 0;
   out_3868394019091270834[16] = 0;
   out_3868394019091270834[17] = 0;
}
void h_30(double *state, double *unused, double *out_8463856008212401643) {
   out_8463856008212401643[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1113232694685506867) {
   out_1113232694685506867[0] = 0;
   out_1113232694685506867[1] = 0;
   out_1113232694685506867[2] = 0;
   out_1113232694685506867[3] = 0;
   out_1113232694685506867[4] = 1;
   out_1113232694685506867[5] = 0;
   out_1113232694685506867[6] = 0;
   out_1113232694685506867[7] = 0;
   out_1113232694685506867[8] = 0;
}
void h_26(double *state, double *unused, double *out_4115924246241713836) {
   out_4115924246241713836[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8901783108029385504) {
   out_8901783108029385504[0] = 0;
   out_8901783108029385504[1] = 0;
   out_8901783108029385504[2] = 0;
   out_8901783108029385504[3] = 0;
   out_8901783108029385504[4] = 0;
   out_8901783108029385504[5] = 0;
   out_8901783108029385504[6] = 0;
   out_8901783108029385504[7] = 1;
   out_8901783108029385504[8] = 0;
}
void h_27(double *state, double *unused, double *out_8396733406681700455) {
   out_8396733406681700455[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1061530617114918044) {
   out_1061530617114918044[0] = 0;
   out_1061530617114918044[1] = 0;
   out_1061530617114918044[2] = 0;
   out_1061530617114918044[3] = 1;
   out_1061530617114918044[4] = 0;
   out_1061530617114918044[5] = 0;
   out_1061530617114918044[6] = 0;
   out_1061530617114918044[7] = 0;
   out_1061530617114918044[8] = 0;
}
void h_29(double *state, double *unused, double *out_3462182740551555998) {
   out_3462182740551555998[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1623464038999899051) {
   out_1623464038999899051[0] = 0;
   out_1623464038999899051[1] = 1;
   out_1623464038999899051[2] = 0;
   out_1623464038999899051[3] = 0;
   out_1623464038999899051[4] = 0;
   out_1623464038999899051[5] = 0;
   out_1623464038999899051[6] = 0;
   out_1623464038999899051[7] = 0;
   out_1623464038999899051[8] = 0;
}
void h_28(double *state, double *unused, double *out_3054866170795669892) {
   out_3054866170795669892[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3458934978069631523) {
   out_3458934978069631523[0] = 1;
   out_3458934978069631523[1] = 0;
   out_3458934978069631523[2] = 0;
   out_3458934978069631523[3] = 0;
   out_3458934978069631523[4] = 0;
   out_3458934978069631523[5] = 0;
   out_3458934978069631523[6] = 0;
   out_3458934978069631523[7] = 0;
   out_3458934978069631523[8] = 0;
}
void h_31(double *state, double *unused, double *out_7288418677868024365) {
   out_7288418677868024365[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5772811684929149460) {
   out_5772811684929149460[0] = 0;
   out_5772811684929149460[1] = 0;
   out_5772811684929149460[2] = 0;
   out_5772811684929149460[3] = 0;
   out_5772811684929149460[4] = 0;
   out_5772811684929149460[5] = 0;
   out_5772811684929149460[6] = 0;
   out_5772811684929149460[7] = 0;
   out_5772811684929149460[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_1556314892176535768) {
  err_fun(nom_x, delta_x, out_1556314892176535768);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6496479561687713128) {
  inv_err_fun(nom_x, true_x, out_6496479561687713128);
}
void car_H_mod_fun(double *state, double *out_6407376786895322314) {
  H_mod_fun(state, out_6407376786895322314);
}
void car_f_fun(double *state, double dt, double *out_6513789890728588369) {
  f_fun(state,  dt, out_6513789890728588369);
}
void car_F_fun(double *state, double dt, double *out_6547009325775891168) {
  F_fun(state,  dt, out_6547009325775891168);
}
void car_h_25(double *state, double *unused, double *out_7013224615583518476) {
  h_25(state, unused, out_7013224615583518476);
}
void car_H_25(double *state, double *unused, double *out_5803457646806109888) {
  H_25(state, unused, out_5803457646806109888);
}
void car_h_24(double *state, double *unused, double *out_6448602700168441562) {
  h_24(state, unused, out_6448602700168441562);
}
void car_H_24(double *state, double *unused, double *out_3868394019091270834) {
  H_24(state, unused, out_3868394019091270834);
}
void car_h_30(double *state, double *unused, double *out_8463856008212401643) {
  h_30(state, unused, out_8463856008212401643);
}
void car_H_30(double *state, double *unused, double *out_1113232694685506867) {
  H_30(state, unused, out_1113232694685506867);
}
void car_h_26(double *state, double *unused, double *out_4115924246241713836) {
  h_26(state, unused, out_4115924246241713836);
}
void car_H_26(double *state, double *unused, double *out_8901783108029385504) {
  H_26(state, unused, out_8901783108029385504);
}
void car_h_27(double *state, double *unused, double *out_8396733406681700455) {
  h_27(state, unused, out_8396733406681700455);
}
void car_H_27(double *state, double *unused, double *out_1061530617114918044) {
  H_27(state, unused, out_1061530617114918044);
}
void car_h_29(double *state, double *unused, double *out_3462182740551555998) {
  h_29(state, unused, out_3462182740551555998);
}
void car_H_29(double *state, double *unused, double *out_1623464038999899051) {
  H_29(state, unused, out_1623464038999899051);
}
void car_h_28(double *state, double *unused, double *out_3054866170795669892) {
  h_28(state, unused, out_3054866170795669892);
}
void car_H_28(double *state, double *unused, double *out_3458934978069631523) {
  H_28(state, unused, out_3458934978069631523);
}
void car_h_31(double *state, double *unused, double *out_7288418677868024365) {
  h_31(state, unused, out_7288418677868024365);
}
void car_H_31(double *state, double *unused, double *out_5772811684929149460) {
  H_31(state, unused, out_5772811684929149460);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
