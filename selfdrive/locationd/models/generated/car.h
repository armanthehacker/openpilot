#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_1556314892176535768);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6496479561687713128);
void car_H_mod_fun(double *state, double *out_6407376786895322314);
void car_f_fun(double *state, double dt, double *out_6513789890728588369);
void car_F_fun(double *state, double dt, double *out_6547009325775891168);
void car_h_25(double *state, double *unused, double *out_7013224615583518476);
void car_H_25(double *state, double *unused, double *out_5803457646806109888);
void car_h_24(double *state, double *unused, double *out_6448602700168441562);
void car_H_24(double *state, double *unused, double *out_3868394019091270834);
void car_h_30(double *state, double *unused, double *out_8463856008212401643);
void car_H_30(double *state, double *unused, double *out_1113232694685506867);
void car_h_26(double *state, double *unused, double *out_4115924246241713836);
void car_H_26(double *state, double *unused, double *out_8901783108029385504);
void car_h_27(double *state, double *unused, double *out_8396733406681700455);
void car_H_27(double *state, double *unused, double *out_1061530617114918044);
void car_h_29(double *state, double *unused, double *out_3462182740551555998);
void car_H_29(double *state, double *unused, double *out_1623464038999899051);
void car_h_28(double *state, double *unused, double *out_3054866170795669892);
void car_H_28(double *state, double *unused, double *out_3458934978069631523);
void car_h_31(double *state, double *unused, double *out_7288418677868024365);
void car_H_31(double *state, double *unused, double *out_5772811684929149460);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}