#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8696545149593538892);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2984200338259875378);
void pose_H_mod_fun(double *state, double *out_3068619037062819826);
void pose_f_fun(double *state, double dt, double *out_2052955553941554201);
void pose_F_fun(double *state, double dt, double *out_349240352665917142);
void pose_h_4(double *state, double *unused, double *out_1865703863787073070);
void pose_H_4(double *state, double *unused, double *out_2083899347976260909);
void pose_h_10(double *state, double *unused, double *out_6754350193537496476);
void pose_H_10(double *state, double *unused, double *out_4195416050396531315);
void pose_h_13(double *state, double *unused, double *out_1886809618332745398);
void pose_H_13(double *state, double *unused, double *out_1749856115326263115);
void pose_h_14(double *state, double *unused, double *out_7068262882355996527);
void pose_H_14(double *state, double *unused, double *out_998889084319111387);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}