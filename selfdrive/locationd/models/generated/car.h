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
void car_err_fun(double *nom_x, double *delta_x, double *out_6074319553370797788);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2818683762402398463);
void car_H_mod_fun(double *state, double *out_4300185914266566871);
void car_f_fun(double *state, double dt, double *out_6274374552211164039);
void car_F_fun(double *state, double dt, double *out_6176046464892570561);
void car_h_25(double *state, double *unused, double *out_2206037968444420053);
void car_H_25(double *state, double *unused, double *out_5175494658605569851);
void car_h_24(double *state, double *unused, double *out_1844074687146334588);
void car_H_24(double *state, double *unused, double *out_8733420257407892987);
void car_h_30(double *state, double *unused, double *out_9173353133125452547);
void car_H_30(double *state, double *unused, double *out_7693827617112818478);
void car_h_26(double *state, double *unused, double *out_7823099266614956505);
void car_H_26(double *state, double *unused, double *out_1433991339731513627);
void car_h_27(double *state, double *unused, double *out_7207711236105265698);
void car_H_27(double *state, double *unused, double *out_5519064305312393567);
void car_h_29(double *state, double *unused, double *out_7482905298389771587);
void car_H_29(double *state, double *unused, double *out_8204058961427210662);
void car_h_28(double *state, double *unused, double *out_808669718691416552);
void car_H_28(double *state, double *unused, double *out_3121659944357680088);
void car_h_31(double *state, double *unused, double *out_1655645507984081599);
void car_H_31(double *state, double *unused, double *out_807783237498162151);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}