#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8696545149593538892) {
   out_8696545149593538892[0] = delta_x[0] + nom_x[0];
   out_8696545149593538892[1] = delta_x[1] + nom_x[1];
   out_8696545149593538892[2] = delta_x[2] + nom_x[2];
   out_8696545149593538892[3] = delta_x[3] + nom_x[3];
   out_8696545149593538892[4] = delta_x[4] + nom_x[4];
   out_8696545149593538892[5] = delta_x[5] + nom_x[5];
   out_8696545149593538892[6] = delta_x[6] + nom_x[6];
   out_8696545149593538892[7] = delta_x[7] + nom_x[7];
   out_8696545149593538892[8] = delta_x[8] + nom_x[8];
   out_8696545149593538892[9] = delta_x[9] + nom_x[9];
   out_8696545149593538892[10] = delta_x[10] + nom_x[10];
   out_8696545149593538892[11] = delta_x[11] + nom_x[11];
   out_8696545149593538892[12] = delta_x[12] + nom_x[12];
   out_8696545149593538892[13] = delta_x[13] + nom_x[13];
   out_8696545149593538892[14] = delta_x[14] + nom_x[14];
   out_8696545149593538892[15] = delta_x[15] + nom_x[15];
   out_8696545149593538892[16] = delta_x[16] + nom_x[16];
   out_8696545149593538892[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2984200338259875378) {
   out_2984200338259875378[0] = -nom_x[0] + true_x[0];
   out_2984200338259875378[1] = -nom_x[1] + true_x[1];
   out_2984200338259875378[2] = -nom_x[2] + true_x[2];
   out_2984200338259875378[3] = -nom_x[3] + true_x[3];
   out_2984200338259875378[4] = -nom_x[4] + true_x[4];
   out_2984200338259875378[5] = -nom_x[5] + true_x[5];
   out_2984200338259875378[6] = -nom_x[6] + true_x[6];
   out_2984200338259875378[7] = -nom_x[7] + true_x[7];
   out_2984200338259875378[8] = -nom_x[8] + true_x[8];
   out_2984200338259875378[9] = -nom_x[9] + true_x[9];
   out_2984200338259875378[10] = -nom_x[10] + true_x[10];
   out_2984200338259875378[11] = -nom_x[11] + true_x[11];
   out_2984200338259875378[12] = -nom_x[12] + true_x[12];
   out_2984200338259875378[13] = -nom_x[13] + true_x[13];
   out_2984200338259875378[14] = -nom_x[14] + true_x[14];
   out_2984200338259875378[15] = -nom_x[15] + true_x[15];
   out_2984200338259875378[16] = -nom_x[16] + true_x[16];
   out_2984200338259875378[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3068619037062819826) {
   out_3068619037062819826[0] = 1.0;
   out_3068619037062819826[1] = 0.0;
   out_3068619037062819826[2] = 0.0;
   out_3068619037062819826[3] = 0.0;
   out_3068619037062819826[4] = 0.0;
   out_3068619037062819826[5] = 0.0;
   out_3068619037062819826[6] = 0.0;
   out_3068619037062819826[7] = 0.0;
   out_3068619037062819826[8] = 0.0;
   out_3068619037062819826[9] = 0.0;
   out_3068619037062819826[10] = 0.0;
   out_3068619037062819826[11] = 0.0;
   out_3068619037062819826[12] = 0.0;
   out_3068619037062819826[13] = 0.0;
   out_3068619037062819826[14] = 0.0;
   out_3068619037062819826[15] = 0.0;
   out_3068619037062819826[16] = 0.0;
   out_3068619037062819826[17] = 0.0;
   out_3068619037062819826[18] = 0.0;
   out_3068619037062819826[19] = 1.0;
   out_3068619037062819826[20] = 0.0;
   out_3068619037062819826[21] = 0.0;
   out_3068619037062819826[22] = 0.0;
   out_3068619037062819826[23] = 0.0;
   out_3068619037062819826[24] = 0.0;
   out_3068619037062819826[25] = 0.0;
   out_3068619037062819826[26] = 0.0;
   out_3068619037062819826[27] = 0.0;
   out_3068619037062819826[28] = 0.0;
   out_3068619037062819826[29] = 0.0;
   out_3068619037062819826[30] = 0.0;
   out_3068619037062819826[31] = 0.0;
   out_3068619037062819826[32] = 0.0;
   out_3068619037062819826[33] = 0.0;
   out_3068619037062819826[34] = 0.0;
   out_3068619037062819826[35] = 0.0;
   out_3068619037062819826[36] = 0.0;
   out_3068619037062819826[37] = 0.0;
   out_3068619037062819826[38] = 1.0;
   out_3068619037062819826[39] = 0.0;
   out_3068619037062819826[40] = 0.0;
   out_3068619037062819826[41] = 0.0;
   out_3068619037062819826[42] = 0.0;
   out_3068619037062819826[43] = 0.0;
   out_3068619037062819826[44] = 0.0;
   out_3068619037062819826[45] = 0.0;
   out_3068619037062819826[46] = 0.0;
   out_3068619037062819826[47] = 0.0;
   out_3068619037062819826[48] = 0.0;
   out_3068619037062819826[49] = 0.0;
   out_3068619037062819826[50] = 0.0;
   out_3068619037062819826[51] = 0.0;
   out_3068619037062819826[52] = 0.0;
   out_3068619037062819826[53] = 0.0;
   out_3068619037062819826[54] = 0.0;
   out_3068619037062819826[55] = 0.0;
   out_3068619037062819826[56] = 0.0;
   out_3068619037062819826[57] = 1.0;
   out_3068619037062819826[58] = 0.0;
   out_3068619037062819826[59] = 0.0;
   out_3068619037062819826[60] = 0.0;
   out_3068619037062819826[61] = 0.0;
   out_3068619037062819826[62] = 0.0;
   out_3068619037062819826[63] = 0.0;
   out_3068619037062819826[64] = 0.0;
   out_3068619037062819826[65] = 0.0;
   out_3068619037062819826[66] = 0.0;
   out_3068619037062819826[67] = 0.0;
   out_3068619037062819826[68] = 0.0;
   out_3068619037062819826[69] = 0.0;
   out_3068619037062819826[70] = 0.0;
   out_3068619037062819826[71] = 0.0;
   out_3068619037062819826[72] = 0.0;
   out_3068619037062819826[73] = 0.0;
   out_3068619037062819826[74] = 0.0;
   out_3068619037062819826[75] = 0.0;
   out_3068619037062819826[76] = 1.0;
   out_3068619037062819826[77] = 0.0;
   out_3068619037062819826[78] = 0.0;
   out_3068619037062819826[79] = 0.0;
   out_3068619037062819826[80] = 0.0;
   out_3068619037062819826[81] = 0.0;
   out_3068619037062819826[82] = 0.0;
   out_3068619037062819826[83] = 0.0;
   out_3068619037062819826[84] = 0.0;
   out_3068619037062819826[85] = 0.0;
   out_3068619037062819826[86] = 0.0;
   out_3068619037062819826[87] = 0.0;
   out_3068619037062819826[88] = 0.0;
   out_3068619037062819826[89] = 0.0;
   out_3068619037062819826[90] = 0.0;
   out_3068619037062819826[91] = 0.0;
   out_3068619037062819826[92] = 0.0;
   out_3068619037062819826[93] = 0.0;
   out_3068619037062819826[94] = 0.0;
   out_3068619037062819826[95] = 1.0;
   out_3068619037062819826[96] = 0.0;
   out_3068619037062819826[97] = 0.0;
   out_3068619037062819826[98] = 0.0;
   out_3068619037062819826[99] = 0.0;
   out_3068619037062819826[100] = 0.0;
   out_3068619037062819826[101] = 0.0;
   out_3068619037062819826[102] = 0.0;
   out_3068619037062819826[103] = 0.0;
   out_3068619037062819826[104] = 0.0;
   out_3068619037062819826[105] = 0.0;
   out_3068619037062819826[106] = 0.0;
   out_3068619037062819826[107] = 0.0;
   out_3068619037062819826[108] = 0.0;
   out_3068619037062819826[109] = 0.0;
   out_3068619037062819826[110] = 0.0;
   out_3068619037062819826[111] = 0.0;
   out_3068619037062819826[112] = 0.0;
   out_3068619037062819826[113] = 0.0;
   out_3068619037062819826[114] = 1.0;
   out_3068619037062819826[115] = 0.0;
   out_3068619037062819826[116] = 0.0;
   out_3068619037062819826[117] = 0.0;
   out_3068619037062819826[118] = 0.0;
   out_3068619037062819826[119] = 0.0;
   out_3068619037062819826[120] = 0.0;
   out_3068619037062819826[121] = 0.0;
   out_3068619037062819826[122] = 0.0;
   out_3068619037062819826[123] = 0.0;
   out_3068619037062819826[124] = 0.0;
   out_3068619037062819826[125] = 0.0;
   out_3068619037062819826[126] = 0.0;
   out_3068619037062819826[127] = 0.0;
   out_3068619037062819826[128] = 0.0;
   out_3068619037062819826[129] = 0.0;
   out_3068619037062819826[130] = 0.0;
   out_3068619037062819826[131] = 0.0;
   out_3068619037062819826[132] = 0.0;
   out_3068619037062819826[133] = 1.0;
   out_3068619037062819826[134] = 0.0;
   out_3068619037062819826[135] = 0.0;
   out_3068619037062819826[136] = 0.0;
   out_3068619037062819826[137] = 0.0;
   out_3068619037062819826[138] = 0.0;
   out_3068619037062819826[139] = 0.0;
   out_3068619037062819826[140] = 0.0;
   out_3068619037062819826[141] = 0.0;
   out_3068619037062819826[142] = 0.0;
   out_3068619037062819826[143] = 0.0;
   out_3068619037062819826[144] = 0.0;
   out_3068619037062819826[145] = 0.0;
   out_3068619037062819826[146] = 0.0;
   out_3068619037062819826[147] = 0.0;
   out_3068619037062819826[148] = 0.0;
   out_3068619037062819826[149] = 0.0;
   out_3068619037062819826[150] = 0.0;
   out_3068619037062819826[151] = 0.0;
   out_3068619037062819826[152] = 1.0;
   out_3068619037062819826[153] = 0.0;
   out_3068619037062819826[154] = 0.0;
   out_3068619037062819826[155] = 0.0;
   out_3068619037062819826[156] = 0.0;
   out_3068619037062819826[157] = 0.0;
   out_3068619037062819826[158] = 0.0;
   out_3068619037062819826[159] = 0.0;
   out_3068619037062819826[160] = 0.0;
   out_3068619037062819826[161] = 0.0;
   out_3068619037062819826[162] = 0.0;
   out_3068619037062819826[163] = 0.0;
   out_3068619037062819826[164] = 0.0;
   out_3068619037062819826[165] = 0.0;
   out_3068619037062819826[166] = 0.0;
   out_3068619037062819826[167] = 0.0;
   out_3068619037062819826[168] = 0.0;
   out_3068619037062819826[169] = 0.0;
   out_3068619037062819826[170] = 0.0;
   out_3068619037062819826[171] = 1.0;
   out_3068619037062819826[172] = 0.0;
   out_3068619037062819826[173] = 0.0;
   out_3068619037062819826[174] = 0.0;
   out_3068619037062819826[175] = 0.0;
   out_3068619037062819826[176] = 0.0;
   out_3068619037062819826[177] = 0.0;
   out_3068619037062819826[178] = 0.0;
   out_3068619037062819826[179] = 0.0;
   out_3068619037062819826[180] = 0.0;
   out_3068619037062819826[181] = 0.0;
   out_3068619037062819826[182] = 0.0;
   out_3068619037062819826[183] = 0.0;
   out_3068619037062819826[184] = 0.0;
   out_3068619037062819826[185] = 0.0;
   out_3068619037062819826[186] = 0.0;
   out_3068619037062819826[187] = 0.0;
   out_3068619037062819826[188] = 0.0;
   out_3068619037062819826[189] = 0.0;
   out_3068619037062819826[190] = 1.0;
   out_3068619037062819826[191] = 0.0;
   out_3068619037062819826[192] = 0.0;
   out_3068619037062819826[193] = 0.0;
   out_3068619037062819826[194] = 0.0;
   out_3068619037062819826[195] = 0.0;
   out_3068619037062819826[196] = 0.0;
   out_3068619037062819826[197] = 0.0;
   out_3068619037062819826[198] = 0.0;
   out_3068619037062819826[199] = 0.0;
   out_3068619037062819826[200] = 0.0;
   out_3068619037062819826[201] = 0.0;
   out_3068619037062819826[202] = 0.0;
   out_3068619037062819826[203] = 0.0;
   out_3068619037062819826[204] = 0.0;
   out_3068619037062819826[205] = 0.0;
   out_3068619037062819826[206] = 0.0;
   out_3068619037062819826[207] = 0.0;
   out_3068619037062819826[208] = 0.0;
   out_3068619037062819826[209] = 1.0;
   out_3068619037062819826[210] = 0.0;
   out_3068619037062819826[211] = 0.0;
   out_3068619037062819826[212] = 0.0;
   out_3068619037062819826[213] = 0.0;
   out_3068619037062819826[214] = 0.0;
   out_3068619037062819826[215] = 0.0;
   out_3068619037062819826[216] = 0.0;
   out_3068619037062819826[217] = 0.0;
   out_3068619037062819826[218] = 0.0;
   out_3068619037062819826[219] = 0.0;
   out_3068619037062819826[220] = 0.0;
   out_3068619037062819826[221] = 0.0;
   out_3068619037062819826[222] = 0.0;
   out_3068619037062819826[223] = 0.0;
   out_3068619037062819826[224] = 0.0;
   out_3068619037062819826[225] = 0.0;
   out_3068619037062819826[226] = 0.0;
   out_3068619037062819826[227] = 0.0;
   out_3068619037062819826[228] = 1.0;
   out_3068619037062819826[229] = 0.0;
   out_3068619037062819826[230] = 0.0;
   out_3068619037062819826[231] = 0.0;
   out_3068619037062819826[232] = 0.0;
   out_3068619037062819826[233] = 0.0;
   out_3068619037062819826[234] = 0.0;
   out_3068619037062819826[235] = 0.0;
   out_3068619037062819826[236] = 0.0;
   out_3068619037062819826[237] = 0.0;
   out_3068619037062819826[238] = 0.0;
   out_3068619037062819826[239] = 0.0;
   out_3068619037062819826[240] = 0.0;
   out_3068619037062819826[241] = 0.0;
   out_3068619037062819826[242] = 0.0;
   out_3068619037062819826[243] = 0.0;
   out_3068619037062819826[244] = 0.0;
   out_3068619037062819826[245] = 0.0;
   out_3068619037062819826[246] = 0.0;
   out_3068619037062819826[247] = 1.0;
   out_3068619037062819826[248] = 0.0;
   out_3068619037062819826[249] = 0.0;
   out_3068619037062819826[250] = 0.0;
   out_3068619037062819826[251] = 0.0;
   out_3068619037062819826[252] = 0.0;
   out_3068619037062819826[253] = 0.0;
   out_3068619037062819826[254] = 0.0;
   out_3068619037062819826[255] = 0.0;
   out_3068619037062819826[256] = 0.0;
   out_3068619037062819826[257] = 0.0;
   out_3068619037062819826[258] = 0.0;
   out_3068619037062819826[259] = 0.0;
   out_3068619037062819826[260] = 0.0;
   out_3068619037062819826[261] = 0.0;
   out_3068619037062819826[262] = 0.0;
   out_3068619037062819826[263] = 0.0;
   out_3068619037062819826[264] = 0.0;
   out_3068619037062819826[265] = 0.0;
   out_3068619037062819826[266] = 1.0;
   out_3068619037062819826[267] = 0.0;
   out_3068619037062819826[268] = 0.0;
   out_3068619037062819826[269] = 0.0;
   out_3068619037062819826[270] = 0.0;
   out_3068619037062819826[271] = 0.0;
   out_3068619037062819826[272] = 0.0;
   out_3068619037062819826[273] = 0.0;
   out_3068619037062819826[274] = 0.0;
   out_3068619037062819826[275] = 0.0;
   out_3068619037062819826[276] = 0.0;
   out_3068619037062819826[277] = 0.0;
   out_3068619037062819826[278] = 0.0;
   out_3068619037062819826[279] = 0.0;
   out_3068619037062819826[280] = 0.0;
   out_3068619037062819826[281] = 0.0;
   out_3068619037062819826[282] = 0.0;
   out_3068619037062819826[283] = 0.0;
   out_3068619037062819826[284] = 0.0;
   out_3068619037062819826[285] = 1.0;
   out_3068619037062819826[286] = 0.0;
   out_3068619037062819826[287] = 0.0;
   out_3068619037062819826[288] = 0.0;
   out_3068619037062819826[289] = 0.0;
   out_3068619037062819826[290] = 0.0;
   out_3068619037062819826[291] = 0.0;
   out_3068619037062819826[292] = 0.0;
   out_3068619037062819826[293] = 0.0;
   out_3068619037062819826[294] = 0.0;
   out_3068619037062819826[295] = 0.0;
   out_3068619037062819826[296] = 0.0;
   out_3068619037062819826[297] = 0.0;
   out_3068619037062819826[298] = 0.0;
   out_3068619037062819826[299] = 0.0;
   out_3068619037062819826[300] = 0.0;
   out_3068619037062819826[301] = 0.0;
   out_3068619037062819826[302] = 0.0;
   out_3068619037062819826[303] = 0.0;
   out_3068619037062819826[304] = 1.0;
   out_3068619037062819826[305] = 0.0;
   out_3068619037062819826[306] = 0.0;
   out_3068619037062819826[307] = 0.0;
   out_3068619037062819826[308] = 0.0;
   out_3068619037062819826[309] = 0.0;
   out_3068619037062819826[310] = 0.0;
   out_3068619037062819826[311] = 0.0;
   out_3068619037062819826[312] = 0.0;
   out_3068619037062819826[313] = 0.0;
   out_3068619037062819826[314] = 0.0;
   out_3068619037062819826[315] = 0.0;
   out_3068619037062819826[316] = 0.0;
   out_3068619037062819826[317] = 0.0;
   out_3068619037062819826[318] = 0.0;
   out_3068619037062819826[319] = 0.0;
   out_3068619037062819826[320] = 0.0;
   out_3068619037062819826[321] = 0.0;
   out_3068619037062819826[322] = 0.0;
   out_3068619037062819826[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_2052955553941554201) {
   out_2052955553941554201[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_2052955553941554201[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_2052955553941554201[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_2052955553941554201[3] = dt*state[12] + state[3];
   out_2052955553941554201[4] = dt*state[13] + state[4];
   out_2052955553941554201[5] = dt*state[14] + state[5];
   out_2052955553941554201[6] = state[6];
   out_2052955553941554201[7] = state[7];
   out_2052955553941554201[8] = state[8];
   out_2052955553941554201[9] = state[9];
   out_2052955553941554201[10] = state[10];
   out_2052955553941554201[11] = state[11];
   out_2052955553941554201[12] = state[12];
   out_2052955553941554201[13] = state[13];
   out_2052955553941554201[14] = state[14];
   out_2052955553941554201[15] = state[15];
   out_2052955553941554201[16] = state[16];
   out_2052955553941554201[17] = state[17];
}
void F_fun(double *state, double dt, double *out_349240352665917142) {
   out_349240352665917142[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_349240352665917142[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_349240352665917142[2] = 0;
   out_349240352665917142[3] = 0;
   out_349240352665917142[4] = 0;
   out_349240352665917142[5] = 0;
   out_349240352665917142[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_349240352665917142[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_349240352665917142[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_349240352665917142[9] = 0;
   out_349240352665917142[10] = 0;
   out_349240352665917142[11] = 0;
   out_349240352665917142[12] = 0;
   out_349240352665917142[13] = 0;
   out_349240352665917142[14] = 0;
   out_349240352665917142[15] = 0;
   out_349240352665917142[16] = 0;
   out_349240352665917142[17] = 0;
   out_349240352665917142[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_349240352665917142[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_349240352665917142[20] = 0;
   out_349240352665917142[21] = 0;
   out_349240352665917142[22] = 0;
   out_349240352665917142[23] = 0;
   out_349240352665917142[24] = 0;
   out_349240352665917142[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_349240352665917142[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_349240352665917142[27] = 0;
   out_349240352665917142[28] = 0;
   out_349240352665917142[29] = 0;
   out_349240352665917142[30] = 0;
   out_349240352665917142[31] = 0;
   out_349240352665917142[32] = 0;
   out_349240352665917142[33] = 0;
   out_349240352665917142[34] = 0;
   out_349240352665917142[35] = 0;
   out_349240352665917142[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_349240352665917142[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_349240352665917142[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_349240352665917142[39] = 0;
   out_349240352665917142[40] = 0;
   out_349240352665917142[41] = 0;
   out_349240352665917142[42] = 0;
   out_349240352665917142[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_349240352665917142[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_349240352665917142[45] = 0;
   out_349240352665917142[46] = 0;
   out_349240352665917142[47] = 0;
   out_349240352665917142[48] = 0;
   out_349240352665917142[49] = 0;
   out_349240352665917142[50] = 0;
   out_349240352665917142[51] = 0;
   out_349240352665917142[52] = 0;
   out_349240352665917142[53] = 0;
   out_349240352665917142[54] = 0;
   out_349240352665917142[55] = 0;
   out_349240352665917142[56] = 0;
   out_349240352665917142[57] = 1;
   out_349240352665917142[58] = 0;
   out_349240352665917142[59] = 0;
   out_349240352665917142[60] = 0;
   out_349240352665917142[61] = 0;
   out_349240352665917142[62] = 0;
   out_349240352665917142[63] = 0;
   out_349240352665917142[64] = 0;
   out_349240352665917142[65] = 0;
   out_349240352665917142[66] = dt;
   out_349240352665917142[67] = 0;
   out_349240352665917142[68] = 0;
   out_349240352665917142[69] = 0;
   out_349240352665917142[70] = 0;
   out_349240352665917142[71] = 0;
   out_349240352665917142[72] = 0;
   out_349240352665917142[73] = 0;
   out_349240352665917142[74] = 0;
   out_349240352665917142[75] = 0;
   out_349240352665917142[76] = 1;
   out_349240352665917142[77] = 0;
   out_349240352665917142[78] = 0;
   out_349240352665917142[79] = 0;
   out_349240352665917142[80] = 0;
   out_349240352665917142[81] = 0;
   out_349240352665917142[82] = 0;
   out_349240352665917142[83] = 0;
   out_349240352665917142[84] = 0;
   out_349240352665917142[85] = dt;
   out_349240352665917142[86] = 0;
   out_349240352665917142[87] = 0;
   out_349240352665917142[88] = 0;
   out_349240352665917142[89] = 0;
   out_349240352665917142[90] = 0;
   out_349240352665917142[91] = 0;
   out_349240352665917142[92] = 0;
   out_349240352665917142[93] = 0;
   out_349240352665917142[94] = 0;
   out_349240352665917142[95] = 1;
   out_349240352665917142[96] = 0;
   out_349240352665917142[97] = 0;
   out_349240352665917142[98] = 0;
   out_349240352665917142[99] = 0;
   out_349240352665917142[100] = 0;
   out_349240352665917142[101] = 0;
   out_349240352665917142[102] = 0;
   out_349240352665917142[103] = 0;
   out_349240352665917142[104] = dt;
   out_349240352665917142[105] = 0;
   out_349240352665917142[106] = 0;
   out_349240352665917142[107] = 0;
   out_349240352665917142[108] = 0;
   out_349240352665917142[109] = 0;
   out_349240352665917142[110] = 0;
   out_349240352665917142[111] = 0;
   out_349240352665917142[112] = 0;
   out_349240352665917142[113] = 0;
   out_349240352665917142[114] = 1;
   out_349240352665917142[115] = 0;
   out_349240352665917142[116] = 0;
   out_349240352665917142[117] = 0;
   out_349240352665917142[118] = 0;
   out_349240352665917142[119] = 0;
   out_349240352665917142[120] = 0;
   out_349240352665917142[121] = 0;
   out_349240352665917142[122] = 0;
   out_349240352665917142[123] = 0;
   out_349240352665917142[124] = 0;
   out_349240352665917142[125] = 0;
   out_349240352665917142[126] = 0;
   out_349240352665917142[127] = 0;
   out_349240352665917142[128] = 0;
   out_349240352665917142[129] = 0;
   out_349240352665917142[130] = 0;
   out_349240352665917142[131] = 0;
   out_349240352665917142[132] = 0;
   out_349240352665917142[133] = 1;
   out_349240352665917142[134] = 0;
   out_349240352665917142[135] = 0;
   out_349240352665917142[136] = 0;
   out_349240352665917142[137] = 0;
   out_349240352665917142[138] = 0;
   out_349240352665917142[139] = 0;
   out_349240352665917142[140] = 0;
   out_349240352665917142[141] = 0;
   out_349240352665917142[142] = 0;
   out_349240352665917142[143] = 0;
   out_349240352665917142[144] = 0;
   out_349240352665917142[145] = 0;
   out_349240352665917142[146] = 0;
   out_349240352665917142[147] = 0;
   out_349240352665917142[148] = 0;
   out_349240352665917142[149] = 0;
   out_349240352665917142[150] = 0;
   out_349240352665917142[151] = 0;
   out_349240352665917142[152] = 1;
   out_349240352665917142[153] = 0;
   out_349240352665917142[154] = 0;
   out_349240352665917142[155] = 0;
   out_349240352665917142[156] = 0;
   out_349240352665917142[157] = 0;
   out_349240352665917142[158] = 0;
   out_349240352665917142[159] = 0;
   out_349240352665917142[160] = 0;
   out_349240352665917142[161] = 0;
   out_349240352665917142[162] = 0;
   out_349240352665917142[163] = 0;
   out_349240352665917142[164] = 0;
   out_349240352665917142[165] = 0;
   out_349240352665917142[166] = 0;
   out_349240352665917142[167] = 0;
   out_349240352665917142[168] = 0;
   out_349240352665917142[169] = 0;
   out_349240352665917142[170] = 0;
   out_349240352665917142[171] = 1;
   out_349240352665917142[172] = 0;
   out_349240352665917142[173] = 0;
   out_349240352665917142[174] = 0;
   out_349240352665917142[175] = 0;
   out_349240352665917142[176] = 0;
   out_349240352665917142[177] = 0;
   out_349240352665917142[178] = 0;
   out_349240352665917142[179] = 0;
   out_349240352665917142[180] = 0;
   out_349240352665917142[181] = 0;
   out_349240352665917142[182] = 0;
   out_349240352665917142[183] = 0;
   out_349240352665917142[184] = 0;
   out_349240352665917142[185] = 0;
   out_349240352665917142[186] = 0;
   out_349240352665917142[187] = 0;
   out_349240352665917142[188] = 0;
   out_349240352665917142[189] = 0;
   out_349240352665917142[190] = 1;
   out_349240352665917142[191] = 0;
   out_349240352665917142[192] = 0;
   out_349240352665917142[193] = 0;
   out_349240352665917142[194] = 0;
   out_349240352665917142[195] = 0;
   out_349240352665917142[196] = 0;
   out_349240352665917142[197] = 0;
   out_349240352665917142[198] = 0;
   out_349240352665917142[199] = 0;
   out_349240352665917142[200] = 0;
   out_349240352665917142[201] = 0;
   out_349240352665917142[202] = 0;
   out_349240352665917142[203] = 0;
   out_349240352665917142[204] = 0;
   out_349240352665917142[205] = 0;
   out_349240352665917142[206] = 0;
   out_349240352665917142[207] = 0;
   out_349240352665917142[208] = 0;
   out_349240352665917142[209] = 1;
   out_349240352665917142[210] = 0;
   out_349240352665917142[211] = 0;
   out_349240352665917142[212] = 0;
   out_349240352665917142[213] = 0;
   out_349240352665917142[214] = 0;
   out_349240352665917142[215] = 0;
   out_349240352665917142[216] = 0;
   out_349240352665917142[217] = 0;
   out_349240352665917142[218] = 0;
   out_349240352665917142[219] = 0;
   out_349240352665917142[220] = 0;
   out_349240352665917142[221] = 0;
   out_349240352665917142[222] = 0;
   out_349240352665917142[223] = 0;
   out_349240352665917142[224] = 0;
   out_349240352665917142[225] = 0;
   out_349240352665917142[226] = 0;
   out_349240352665917142[227] = 0;
   out_349240352665917142[228] = 1;
   out_349240352665917142[229] = 0;
   out_349240352665917142[230] = 0;
   out_349240352665917142[231] = 0;
   out_349240352665917142[232] = 0;
   out_349240352665917142[233] = 0;
   out_349240352665917142[234] = 0;
   out_349240352665917142[235] = 0;
   out_349240352665917142[236] = 0;
   out_349240352665917142[237] = 0;
   out_349240352665917142[238] = 0;
   out_349240352665917142[239] = 0;
   out_349240352665917142[240] = 0;
   out_349240352665917142[241] = 0;
   out_349240352665917142[242] = 0;
   out_349240352665917142[243] = 0;
   out_349240352665917142[244] = 0;
   out_349240352665917142[245] = 0;
   out_349240352665917142[246] = 0;
   out_349240352665917142[247] = 1;
   out_349240352665917142[248] = 0;
   out_349240352665917142[249] = 0;
   out_349240352665917142[250] = 0;
   out_349240352665917142[251] = 0;
   out_349240352665917142[252] = 0;
   out_349240352665917142[253] = 0;
   out_349240352665917142[254] = 0;
   out_349240352665917142[255] = 0;
   out_349240352665917142[256] = 0;
   out_349240352665917142[257] = 0;
   out_349240352665917142[258] = 0;
   out_349240352665917142[259] = 0;
   out_349240352665917142[260] = 0;
   out_349240352665917142[261] = 0;
   out_349240352665917142[262] = 0;
   out_349240352665917142[263] = 0;
   out_349240352665917142[264] = 0;
   out_349240352665917142[265] = 0;
   out_349240352665917142[266] = 1;
   out_349240352665917142[267] = 0;
   out_349240352665917142[268] = 0;
   out_349240352665917142[269] = 0;
   out_349240352665917142[270] = 0;
   out_349240352665917142[271] = 0;
   out_349240352665917142[272] = 0;
   out_349240352665917142[273] = 0;
   out_349240352665917142[274] = 0;
   out_349240352665917142[275] = 0;
   out_349240352665917142[276] = 0;
   out_349240352665917142[277] = 0;
   out_349240352665917142[278] = 0;
   out_349240352665917142[279] = 0;
   out_349240352665917142[280] = 0;
   out_349240352665917142[281] = 0;
   out_349240352665917142[282] = 0;
   out_349240352665917142[283] = 0;
   out_349240352665917142[284] = 0;
   out_349240352665917142[285] = 1;
   out_349240352665917142[286] = 0;
   out_349240352665917142[287] = 0;
   out_349240352665917142[288] = 0;
   out_349240352665917142[289] = 0;
   out_349240352665917142[290] = 0;
   out_349240352665917142[291] = 0;
   out_349240352665917142[292] = 0;
   out_349240352665917142[293] = 0;
   out_349240352665917142[294] = 0;
   out_349240352665917142[295] = 0;
   out_349240352665917142[296] = 0;
   out_349240352665917142[297] = 0;
   out_349240352665917142[298] = 0;
   out_349240352665917142[299] = 0;
   out_349240352665917142[300] = 0;
   out_349240352665917142[301] = 0;
   out_349240352665917142[302] = 0;
   out_349240352665917142[303] = 0;
   out_349240352665917142[304] = 1;
   out_349240352665917142[305] = 0;
   out_349240352665917142[306] = 0;
   out_349240352665917142[307] = 0;
   out_349240352665917142[308] = 0;
   out_349240352665917142[309] = 0;
   out_349240352665917142[310] = 0;
   out_349240352665917142[311] = 0;
   out_349240352665917142[312] = 0;
   out_349240352665917142[313] = 0;
   out_349240352665917142[314] = 0;
   out_349240352665917142[315] = 0;
   out_349240352665917142[316] = 0;
   out_349240352665917142[317] = 0;
   out_349240352665917142[318] = 0;
   out_349240352665917142[319] = 0;
   out_349240352665917142[320] = 0;
   out_349240352665917142[321] = 0;
   out_349240352665917142[322] = 0;
   out_349240352665917142[323] = 1;
}
void h_4(double *state, double *unused, double *out_1865703863787073070) {
   out_1865703863787073070[0] = state[6] + state[9];
   out_1865703863787073070[1] = state[7] + state[10];
   out_1865703863787073070[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2083899347976260909) {
   out_2083899347976260909[0] = 0;
   out_2083899347976260909[1] = 0;
   out_2083899347976260909[2] = 0;
   out_2083899347976260909[3] = 0;
   out_2083899347976260909[4] = 0;
   out_2083899347976260909[5] = 0;
   out_2083899347976260909[6] = 1;
   out_2083899347976260909[7] = 0;
   out_2083899347976260909[8] = 0;
   out_2083899347976260909[9] = 1;
   out_2083899347976260909[10] = 0;
   out_2083899347976260909[11] = 0;
   out_2083899347976260909[12] = 0;
   out_2083899347976260909[13] = 0;
   out_2083899347976260909[14] = 0;
   out_2083899347976260909[15] = 0;
   out_2083899347976260909[16] = 0;
   out_2083899347976260909[17] = 0;
   out_2083899347976260909[18] = 0;
   out_2083899347976260909[19] = 0;
   out_2083899347976260909[20] = 0;
   out_2083899347976260909[21] = 0;
   out_2083899347976260909[22] = 0;
   out_2083899347976260909[23] = 0;
   out_2083899347976260909[24] = 0;
   out_2083899347976260909[25] = 1;
   out_2083899347976260909[26] = 0;
   out_2083899347976260909[27] = 0;
   out_2083899347976260909[28] = 1;
   out_2083899347976260909[29] = 0;
   out_2083899347976260909[30] = 0;
   out_2083899347976260909[31] = 0;
   out_2083899347976260909[32] = 0;
   out_2083899347976260909[33] = 0;
   out_2083899347976260909[34] = 0;
   out_2083899347976260909[35] = 0;
   out_2083899347976260909[36] = 0;
   out_2083899347976260909[37] = 0;
   out_2083899347976260909[38] = 0;
   out_2083899347976260909[39] = 0;
   out_2083899347976260909[40] = 0;
   out_2083899347976260909[41] = 0;
   out_2083899347976260909[42] = 0;
   out_2083899347976260909[43] = 0;
   out_2083899347976260909[44] = 1;
   out_2083899347976260909[45] = 0;
   out_2083899347976260909[46] = 0;
   out_2083899347976260909[47] = 1;
   out_2083899347976260909[48] = 0;
   out_2083899347976260909[49] = 0;
   out_2083899347976260909[50] = 0;
   out_2083899347976260909[51] = 0;
   out_2083899347976260909[52] = 0;
   out_2083899347976260909[53] = 0;
}
void h_10(double *state, double *unused, double *out_6754350193537496476) {
   out_6754350193537496476[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_6754350193537496476[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_6754350193537496476[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_4195416050396531315) {
   out_4195416050396531315[0] = 0;
   out_4195416050396531315[1] = 9.8100000000000005*cos(state[1]);
   out_4195416050396531315[2] = 0;
   out_4195416050396531315[3] = 0;
   out_4195416050396531315[4] = -state[8];
   out_4195416050396531315[5] = state[7];
   out_4195416050396531315[6] = 0;
   out_4195416050396531315[7] = state[5];
   out_4195416050396531315[8] = -state[4];
   out_4195416050396531315[9] = 0;
   out_4195416050396531315[10] = 0;
   out_4195416050396531315[11] = 0;
   out_4195416050396531315[12] = 1;
   out_4195416050396531315[13] = 0;
   out_4195416050396531315[14] = 0;
   out_4195416050396531315[15] = 1;
   out_4195416050396531315[16] = 0;
   out_4195416050396531315[17] = 0;
   out_4195416050396531315[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_4195416050396531315[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_4195416050396531315[20] = 0;
   out_4195416050396531315[21] = state[8];
   out_4195416050396531315[22] = 0;
   out_4195416050396531315[23] = -state[6];
   out_4195416050396531315[24] = -state[5];
   out_4195416050396531315[25] = 0;
   out_4195416050396531315[26] = state[3];
   out_4195416050396531315[27] = 0;
   out_4195416050396531315[28] = 0;
   out_4195416050396531315[29] = 0;
   out_4195416050396531315[30] = 0;
   out_4195416050396531315[31] = 1;
   out_4195416050396531315[32] = 0;
   out_4195416050396531315[33] = 0;
   out_4195416050396531315[34] = 1;
   out_4195416050396531315[35] = 0;
   out_4195416050396531315[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_4195416050396531315[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_4195416050396531315[38] = 0;
   out_4195416050396531315[39] = -state[7];
   out_4195416050396531315[40] = state[6];
   out_4195416050396531315[41] = 0;
   out_4195416050396531315[42] = state[4];
   out_4195416050396531315[43] = -state[3];
   out_4195416050396531315[44] = 0;
   out_4195416050396531315[45] = 0;
   out_4195416050396531315[46] = 0;
   out_4195416050396531315[47] = 0;
   out_4195416050396531315[48] = 0;
   out_4195416050396531315[49] = 0;
   out_4195416050396531315[50] = 1;
   out_4195416050396531315[51] = 0;
   out_4195416050396531315[52] = 0;
   out_4195416050396531315[53] = 1;
}
void h_13(double *state, double *unused, double *out_1886809618332745398) {
   out_1886809618332745398[0] = state[3];
   out_1886809618332745398[1] = state[4];
   out_1886809618332745398[2] = state[5];
}
void H_13(double *state, double *unused, double *out_1749856115326263115) {
   out_1749856115326263115[0] = 0;
   out_1749856115326263115[1] = 0;
   out_1749856115326263115[2] = 0;
   out_1749856115326263115[3] = 1;
   out_1749856115326263115[4] = 0;
   out_1749856115326263115[5] = 0;
   out_1749856115326263115[6] = 0;
   out_1749856115326263115[7] = 0;
   out_1749856115326263115[8] = 0;
   out_1749856115326263115[9] = 0;
   out_1749856115326263115[10] = 0;
   out_1749856115326263115[11] = 0;
   out_1749856115326263115[12] = 0;
   out_1749856115326263115[13] = 0;
   out_1749856115326263115[14] = 0;
   out_1749856115326263115[15] = 0;
   out_1749856115326263115[16] = 0;
   out_1749856115326263115[17] = 0;
   out_1749856115326263115[18] = 0;
   out_1749856115326263115[19] = 0;
   out_1749856115326263115[20] = 0;
   out_1749856115326263115[21] = 0;
   out_1749856115326263115[22] = 1;
   out_1749856115326263115[23] = 0;
   out_1749856115326263115[24] = 0;
   out_1749856115326263115[25] = 0;
   out_1749856115326263115[26] = 0;
   out_1749856115326263115[27] = 0;
   out_1749856115326263115[28] = 0;
   out_1749856115326263115[29] = 0;
   out_1749856115326263115[30] = 0;
   out_1749856115326263115[31] = 0;
   out_1749856115326263115[32] = 0;
   out_1749856115326263115[33] = 0;
   out_1749856115326263115[34] = 0;
   out_1749856115326263115[35] = 0;
   out_1749856115326263115[36] = 0;
   out_1749856115326263115[37] = 0;
   out_1749856115326263115[38] = 0;
   out_1749856115326263115[39] = 0;
   out_1749856115326263115[40] = 0;
   out_1749856115326263115[41] = 1;
   out_1749856115326263115[42] = 0;
   out_1749856115326263115[43] = 0;
   out_1749856115326263115[44] = 0;
   out_1749856115326263115[45] = 0;
   out_1749856115326263115[46] = 0;
   out_1749856115326263115[47] = 0;
   out_1749856115326263115[48] = 0;
   out_1749856115326263115[49] = 0;
   out_1749856115326263115[50] = 0;
   out_1749856115326263115[51] = 0;
   out_1749856115326263115[52] = 0;
   out_1749856115326263115[53] = 0;
}
void h_14(double *state, double *unused, double *out_7068262882355996527) {
   out_7068262882355996527[0] = state[6];
   out_7068262882355996527[1] = state[7];
   out_7068262882355996527[2] = state[8];
}
void H_14(double *state, double *unused, double *out_998889084319111387) {
   out_998889084319111387[0] = 0;
   out_998889084319111387[1] = 0;
   out_998889084319111387[2] = 0;
   out_998889084319111387[3] = 0;
   out_998889084319111387[4] = 0;
   out_998889084319111387[5] = 0;
   out_998889084319111387[6] = 1;
   out_998889084319111387[7] = 0;
   out_998889084319111387[8] = 0;
   out_998889084319111387[9] = 0;
   out_998889084319111387[10] = 0;
   out_998889084319111387[11] = 0;
   out_998889084319111387[12] = 0;
   out_998889084319111387[13] = 0;
   out_998889084319111387[14] = 0;
   out_998889084319111387[15] = 0;
   out_998889084319111387[16] = 0;
   out_998889084319111387[17] = 0;
   out_998889084319111387[18] = 0;
   out_998889084319111387[19] = 0;
   out_998889084319111387[20] = 0;
   out_998889084319111387[21] = 0;
   out_998889084319111387[22] = 0;
   out_998889084319111387[23] = 0;
   out_998889084319111387[24] = 0;
   out_998889084319111387[25] = 1;
   out_998889084319111387[26] = 0;
   out_998889084319111387[27] = 0;
   out_998889084319111387[28] = 0;
   out_998889084319111387[29] = 0;
   out_998889084319111387[30] = 0;
   out_998889084319111387[31] = 0;
   out_998889084319111387[32] = 0;
   out_998889084319111387[33] = 0;
   out_998889084319111387[34] = 0;
   out_998889084319111387[35] = 0;
   out_998889084319111387[36] = 0;
   out_998889084319111387[37] = 0;
   out_998889084319111387[38] = 0;
   out_998889084319111387[39] = 0;
   out_998889084319111387[40] = 0;
   out_998889084319111387[41] = 0;
   out_998889084319111387[42] = 0;
   out_998889084319111387[43] = 0;
   out_998889084319111387[44] = 1;
   out_998889084319111387[45] = 0;
   out_998889084319111387[46] = 0;
   out_998889084319111387[47] = 0;
   out_998889084319111387[48] = 0;
   out_998889084319111387[49] = 0;
   out_998889084319111387[50] = 0;
   out_998889084319111387[51] = 0;
   out_998889084319111387[52] = 0;
   out_998889084319111387[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_8696545149593538892) {
  err_fun(nom_x, delta_x, out_8696545149593538892);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2984200338259875378) {
  inv_err_fun(nom_x, true_x, out_2984200338259875378);
}
void pose_H_mod_fun(double *state, double *out_3068619037062819826) {
  H_mod_fun(state, out_3068619037062819826);
}
void pose_f_fun(double *state, double dt, double *out_2052955553941554201) {
  f_fun(state,  dt, out_2052955553941554201);
}
void pose_F_fun(double *state, double dt, double *out_349240352665917142) {
  F_fun(state,  dt, out_349240352665917142);
}
void pose_h_4(double *state, double *unused, double *out_1865703863787073070) {
  h_4(state, unused, out_1865703863787073070);
}
void pose_H_4(double *state, double *unused, double *out_2083899347976260909) {
  H_4(state, unused, out_2083899347976260909);
}
void pose_h_10(double *state, double *unused, double *out_6754350193537496476) {
  h_10(state, unused, out_6754350193537496476);
}
void pose_H_10(double *state, double *unused, double *out_4195416050396531315) {
  H_10(state, unused, out_4195416050396531315);
}
void pose_h_13(double *state, double *unused, double *out_1886809618332745398) {
  h_13(state, unused, out_1886809618332745398);
}
void pose_H_13(double *state, double *unused, double *out_1749856115326263115) {
  H_13(state, unused, out_1749856115326263115);
}
void pose_h_14(double *state, double *unused, double *out_7068262882355996527) {
  h_14(state, unused, out_7068262882355996527);
}
void pose_H_14(double *state, double *unused, double *out_998889084319111387) {
  H_14(state, unused, out_998889084319111387);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
