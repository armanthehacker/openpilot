#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_3328892757986285011);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4963870370355840850);
void pose_H_mod_fun(double *state, double *out_7601866641244281513);
void pose_f_fun(double *state, double dt, double *out_8103375224917938147);
void pose_F_fun(double *state, double dt, double *out_1507526485982590861);
void pose_h_4(double *state, double *unused, double *out_1827372507797762722);
void pose_H_4(double *state, double *unused, double *out_598438242728943254);
void pose_h_10(double *state, double *unused, double *out_9090888505167823902);
void pose_H_10(double *state, double *unused, double *out_5725378386810357076);
void pose_h_13(double *state, double *unused, double *out_7964679880147664761);
void pose_H_13(double *state, double *unused, double *out_8209069451045644183);
void pose_h_14(double *state, double *unused, double *out_7516795339953614610);
void pose_H_14(double *state, double *unused, double *out_4561679099068427783);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}