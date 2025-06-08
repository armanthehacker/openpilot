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
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3328892757986285011) {
   out_3328892757986285011[0] = delta_x[0] + nom_x[0];
   out_3328892757986285011[1] = delta_x[1] + nom_x[1];
   out_3328892757986285011[2] = delta_x[2] + nom_x[2];
   out_3328892757986285011[3] = delta_x[3] + nom_x[3];
   out_3328892757986285011[4] = delta_x[4] + nom_x[4];
   out_3328892757986285011[5] = delta_x[5] + nom_x[5];
   out_3328892757986285011[6] = delta_x[6] + nom_x[6];
   out_3328892757986285011[7] = delta_x[7] + nom_x[7];
   out_3328892757986285011[8] = delta_x[8] + nom_x[8];
   out_3328892757986285011[9] = delta_x[9] + nom_x[9];
   out_3328892757986285011[10] = delta_x[10] + nom_x[10];
   out_3328892757986285011[11] = delta_x[11] + nom_x[11];
   out_3328892757986285011[12] = delta_x[12] + nom_x[12];
   out_3328892757986285011[13] = delta_x[13] + nom_x[13];
   out_3328892757986285011[14] = delta_x[14] + nom_x[14];
   out_3328892757986285011[15] = delta_x[15] + nom_x[15];
   out_3328892757986285011[16] = delta_x[16] + nom_x[16];
   out_3328892757986285011[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4963870370355840850) {
   out_4963870370355840850[0] = -nom_x[0] + true_x[0];
   out_4963870370355840850[1] = -nom_x[1] + true_x[1];
   out_4963870370355840850[2] = -nom_x[2] + true_x[2];
   out_4963870370355840850[3] = -nom_x[3] + true_x[3];
   out_4963870370355840850[4] = -nom_x[4] + true_x[4];
   out_4963870370355840850[5] = -nom_x[5] + true_x[5];
   out_4963870370355840850[6] = -nom_x[6] + true_x[6];
   out_4963870370355840850[7] = -nom_x[7] + true_x[7];
   out_4963870370355840850[8] = -nom_x[8] + true_x[8];
   out_4963870370355840850[9] = -nom_x[9] + true_x[9];
   out_4963870370355840850[10] = -nom_x[10] + true_x[10];
   out_4963870370355840850[11] = -nom_x[11] + true_x[11];
   out_4963870370355840850[12] = -nom_x[12] + true_x[12];
   out_4963870370355840850[13] = -nom_x[13] + true_x[13];
   out_4963870370355840850[14] = -nom_x[14] + true_x[14];
   out_4963870370355840850[15] = -nom_x[15] + true_x[15];
   out_4963870370355840850[16] = -nom_x[16] + true_x[16];
   out_4963870370355840850[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_7601866641244281513) {
   out_7601866641244281513[0] = 1.0;
   out_7601866641244281513[1] = 0.0;
   out_7601866641244281513[2] = 0.0;
   out_7601866641244281513[3] = 0.0;
   out_7601866641244281513[4] = 0.0;
   out_7601866641244281513[5] = 0.0;
   out_7601866641244281513[6] = 0.0;
   out_7601866641244281513[7] = 0.0;
   out_7601866641244281513[8] = 0.0;
   out_7601866641244281513[9] = 0.0;
   out_7601866641244281513[10] = 0.0;
   out_7601866641244281513[11] = 0.0;
   out_7601866641244281513[12] = 0.0;
   out_7601866641244281513[13] = 0.0;
   out_7601866641244281513[14] = 0.0;
   out_7601866641244281513[15] = 0.0;
   out_7601866641244281513[16] = 0.0;
   out_7601866641244281513[17] = 0.0;
   out_7601866641244281513[18] = 0.0;
   out_7601866641244281513[19] = 1.0;
   out_7601866641244281513[20] = 0.0;
   out_7601866641244281513[21] = 0.0;
   out_7601866641244281513[22] = 0.0;
   out_7601866641244281513[23] = 0.0;
   out_7601866641244281513[24] = 0.0;
   out_7601866641244281513[25] = 0.0;
   out_7601866641244281513[26] = 0.0;
   out_7601866641244281513[27] = 0.0;
   out_7601866641244281513[28] = 0.0;
   out_7601866641244281513[29] = 0.0;
   out_7601866641244281513[30] = 0.0;
   out_7601866641244281513[31] = 0.0;
   out_7601866641244281513[32] = 0.0;
   out_7601866641244281513[33] = 0.0;
   out_7601866641244281513[34] = 0.0;
   out_7601866641244281513[35] = 0.0;
   out_7601866641244281513[36] = 0.0;
   out_7601866641244281513[37] = 0.0;
   out_7601866641244281513[38] = 1.0;
   out_7601866641244281513[39] = 0.0;
   out_7601866641244281513[40] = 0.0;
   out_7601866641244281513[41] = 0.0;
   out_7601866641244281513[42] = 0.0;
   out_7601866641244281513[43] = 0.0;
   out_7601866641244281513[44] = 0.0;
   out_7601866641244281513[45] = 0.0;
   out_7601866641244281513[46] = 0.0;
   out_7601866641244281513[47] = 0.0;
   out_7601866641244281513[48] = 0.0;
   out_7601866641244281513[49] = 0.0;
   out_7601866641244281513[50] = 0.0;
   out_7601866641244281513[51] = 0.0;
   out_7601866641244281513[52] = 0.0;
   out_7601866641244281513[53] = 0.0;
   out_7601866641244281513[54] = 0.0;
   out_7601866641244281513[55] = 0.0;
   out_7601866641244281513[56] = 0.0;
   out_7601866641244281513[57] = 1.0;
   out_7601866641244281513[58] = 0.0;
   out_7601866641244281513[59] = 0.0;
   out_7601866641244281513[60] = 0.0;
   out_7601866641244281513[61] = 0.0;
   out_7601866641244281513[62] = 0.0;
   out_7601866641244281513[63] = 0.0;
   out_7601866641244281513[64] = 0.0;
   out_7601866641244281513[65] = 0.0;
   out_7601866641244281513[66] = 0.0;
   out_7601866641244281513[67] = 0.0;
   out_7601866641244281513[68] = 0.0;
   out_7601866641244281513[69] = 0.0;
   out_7601866641244281513[70] = 0.0;
   out_7601866641244281513[71] = 0.0;
   out_7601866641244281513[72] = 0.0;
   out_7601866641244281513[73] = 0.0;
   out_7601866641244281513[74] = 0.0;
   out_7601866641244281513[75] = 0.0;
   out_7601866641244281513[76] = 1.0;
   out_7601866641244281513[77] = 0.0;
   out_7601866641244281513[78] = 0.0;
   out_7601866641244281513[79] = 0.0;
   out_7601866641244281513[80] = 0.0;
   out_7601866641244281513[81] = 0.0;
   out_7601866641244281513[82] = 0.0;
   out_7601866641244281513[83] = 0.0;
   out_7601866641244281513[84] = 0.0;
   out_7601866641244281513[85] = 0.0;
   out_7601866641244281513[86] = 0.0;
   out_7601866641244281513[87] = 0.0;
   out_7601866641244281513[88] = 0.0;
   out_7601866641244281513[89] = 0.0;
   out_7601866641244281513[90] = 0.0;
   out_7601866641244281513[91] = 0.0;
   out_7601866641244281513[92] = 0.0;
   out_7601866641244281513[93] = 0.0;
   out_7601866641244281513[94] = 0.0;
   out_7601866641244281513[95] = 1.0;
   out_7601866641244281513[96] = 0.0;
   out_7601866641244281513[97] = 0.0;
   out_7601866641244281513[98] = 0.0;
   out_7601866641244281513[99] = 0.0;
   out_7601866641244281513[100] = 0.0;
   out_7601866641244281513[101] = 0.0;
   out_7601866641244281513[102] = 0.0;
   out_7601866641244281513[103] = 0.0;
   out_7601866641244281513[104] = 0.0;
   out_7601866641244281513[105] = 0.0;
   out_7601866641244281513[106] = 0.0;
   out_7601866641244281513[107] = 0.0;
   out_7601866641244281513[108] = 0.0;
   out_7601866641244281513[109] = 0.0;
   out_7601866641244281513[110] = 0.0;
   out_7601866641244281513[111] = 0.0;
   out_7601866641244281513[112] = 0.0;
   out_7601866641244281513[113] = 0.0;
   out_7601866641244281513[114] = 1.0;
   out_7601866641244281513[115] = 0.0;
   out_7601866641244281513[116] = 0.0;
   out_7601866641244281513[117] = 0.0;
   out_7601866641244281513[118] = 0.0;
   out_7601866641244281513[119] = 0.0;
   out_7601866641244281513[120] = 0.0;
   out_7601866641244281513[121] = 0.0;
   out_7601866641244281513[122] = 0.0;
   out_7601866641244281513[123] = 0.0;
   out_7601866641244281513[124] = 0.0;
   out_7601866641244281513[125] = 0.0;
   out_7601866641244281513[126] = 0.0;
   out_7601866641244281513[127] = 0.0;
   out_7601866641244281513[128] = 0.0;
   out_7601866641244281513[129] = 0.0;
   out_7601866641244281513[130] = 0.0;
   out_7601866641244281513[131] = 0.0;
   out_7601866641244281513[132] = 0.0;
   out_7601866641244281513[133] = 1.0;
   out_7601866641244281513[134] = 0.0;
   out_7601866641244281513[135] = 0.0;
   out_7601866641244281513[136] = 0.0;
   out_7601866641244281513[137] = 0.0;
   out_7601866641244281513[138] = 0.0;
   out_7601866641244281513[139] = 0.0;
   out_7601866641244281513[140] = 0.0;
   out_7601866641244281513[141] = 0.0;
   out_7601866641244281513[142] = 0.0;
   out_7601866641244281513[143] = 0.0;
   out_7601866641244281513[144] = 0.0;
   out_7601866641244281513[145] = 0.0;
   out_7601866641244281513[146] = 0.0;
   out_7601866641244281513[147] = 0.0;
   out_7601866641244281513[148] = 0.0;
   out_7601866641244281513[149] = 0.0;
   out_7601866641244281513[150] = 0.0;
   out_7601866641244281513[151] = 0.0;
   out_7601866641244281513[152] = 1.0;
   out_7601866641244281513[153] = 0.0;
   out_7601866641244281513[154] = 0.0;
   out_7601866641244281513[155] = 0.0;
   out_7601866641244281513[156] = 0.0;
   out_7601866641244281513[157] = 0.0;
   out_7601866641244281513[158] = 0.0;
   out_7601866641244281513[159] = 0.0;
   out_7601866641244281513[160] = 0.0;
   out_7601866641244281513[161] = 0.0;
   out_7601866641244281513[162] = 0.0;
   out_7601866641244281513[163] = 0.0;
   out_7601866641244281513[164] = 0.0;
   out_7601866641244281513[165] = 0.0;
   out_7601866641244281513[166] = 0.0;
   out_7601866641244281513[167] = 0.0;
   out_7601866641244281513[168] = 0.0;
   out_7601866641244281513[169] = 0.0;
   out_7601866641244281513[170] = 0.0;
   out_7601866641244281513[171] = 1.0;
   out_7601866641244281513[172] = 0.0;
   out_7601866641244281513[173] = 0.0;
   out_7601866641244281513[174] = 0.0;
   out_7601866641244281513[175] = 0.0;
   out_7601866641244281513[176] = 0.0;
   out_7601866641244281513[177] = 0.0;
   out_7601866641244281513[178] = 0.0;
   out_7601866641244281513[179] = 0.0;
   out_7601866641244281513[180] = 0.0;
   out_7601866641244281513[181] = 0.0;
   out_7601866641244281513[182] = 0.0;
   out_7601866641244281513[183] = 0.0;
   out_7601866641244281513[184] = 0.0;
   out_7601866641244281513[185] = 0.0;
   out_7601866641244281513[186] = 0.0;
   out_7601866641244281513[187] = 0.0;
   out_7601866641244281513[188] = 0.0;
   out_7601866641244281513[189] = 0.0;
   out_7601866641244281513[190] = 1.0;
   out_7601866641244281513[191] = 0.0;
   out_7601866641244281513[192] = 0.0;
   out_7601866641244281513[193] = 0.0;
   out_7601866641244281513[194] = 0.0;
   out_7601866641244281513[195] = 0.0;
   out_7601866641244281513[196] = 0.0;
   out_7601866641244281513[197] = 0.0;
   out_7601866641244281513[198] = 0.0;
   out_7601866641244281513[199] = 0.0;
   out_7601866641244281513[200] = 0.0;
   out_7601866641244281513[201] = 0.0;
   out_7601866641244281513[202] = 0.0;
   out_7601866641244281513[203] = 0.0;
   out_7601866641244281513[204] = 0.0;
   out_7601866641244281513[205] = 0.0;
   out_7601866641244281513[206] = 0.0;
   out_7601866641244281513[207] = 0.0;
   out_7601866641244281513[208] = 0.0;
   out_7601866641244281513[209] = 1.0;
   out_7601866641244281513[210] = 0.0;
   out_7601866641244281513[211] = 0.0;
   out_7601866641244281513[212] = 0.0;
   out_7601866641244281513[213] = 0.0;
   out_7601866641244281513[214] = 0.0;
   out_7601866641244281513[215] = 0.0;
   out_7601866641244281513[216] = 0.0;
   out_7601866641244281513[217] = 0.0;
   out_7601866641244281513[218] = 0.0;
   out_7601866641244281513[219] = 0.0;
   out_7601866641244281513[220] = 0.0;
   out_7601866641244281513[221] = 0.0;
   out_7601866641244281513[222] = 0.0;
   out_7601866641244281513[223] = 0.0;
   out_7601866641244281513[224] = 0.0;
   out_7601866641244281513[225] = 0.0;
   out_7601866641244281513[226] = 0.0;
   out_7601866641244281513[227] = 0.0;
   out_7601866641244281513[228] = 1.0;
   out_7601866641244281513[229] = 0.0;
   out_7601866641244281513[230] = 0.0;
   out_7601866641244281513[231] = 0.0;
   out_7601866641244281513[232] = 0.0;
   out_7601866641244281513[233] = 0.0;
   out_7601866641244281513[234] = 0.0;
   out_7601866641244281513[235] = 0.0;
   out_7601866641244281513[236] = 0.0;
   out_7601866641244281513[237] = 0.0;
   out_7601866641244281513[238] = 0.0;
   out_7601866641244281513[239] = 0.0;
   out_7601866641244281513[240] = 0.0;
   out_7601866641244281513[241] = 0.0;
   out_7601866641244281513[242] = 0.0;
   out_7601866641244281513[243] = 0.0;
   out_7601866641244281513[244] = 0.0;
   out_7601866641244281513[245] = 0.0;
   out_7601866641244281513[246] = 0.0;
   out_7601866641244281513[247] = 1.0;
   out_7601866641244281513[248] = 0.0;
   out_7601866641244281513[249] = 0.0;
   out_7601866641244281513[250] = 0.0;
   out_7601866641244281513[251] = 0.0;
   out_7601866641244281513[252] = 0.0;
   out_7601866641244281513[253] = 0.0;
   out_7601866641244281513[254] = 0.0;
   out_7601866641244281513[255] = 0.0;
   out_7601866641244281513[256] = 0.0;
   out_7601866641244281513[257] = 0.0;
   out_7601866641244281513[258] = 0.0;
   out_7601866641244281513[259] = 0.0;
   out_7601866641244281513[260] = 0.0;
   out_7601866641244281513[261] = 0.0;
   out_7601866641244281513[262] = 0.0;
   out_7601866641244281513[263] = 0.0;
   out_7601866641244281513[264] = 0.0;
   out_7601866641244281513[265] = 0.0;
   out_7601866641244281513[266] = 1.0;
   out_7601866641244281513[267] = 0.0;
   out_7601866641244281513[268] = 0.0;
   out_7601866641244281513[269] = 0.0;
   out_7601866641244281513[270] = 0.0;
   out_7601866641244281513[271] = 0.0;
   out_7601866641244281513[272] = 0.0;
   out_7601866641244281513[273] = 0.0;
   out_7601866641244281513[274] = 0.0;
   out_7601866641244281513[275] = 0.0;
   out_7601866641244281513[276] = 0.0;
   out_7601866641244281513[277] = 0.0;
   out_7601866641244281513[278] = 0.0;
   out_7601866641244281513[279] = 0.0;
   out_7601866641244281513[280] = 0.0;
   out_7601866641244281513[281] = 0.0;
   out_7601866641244281513[282] = 0.0;
   out_7601866641244281513[283] = 0.0;
   out_7601866641244281513[284] = 0.0;
   out_7601866641244281513[285] = 1.0;
   out_7601866641244281513[286] = 0.0;
   out_7601866641244281513[287] = 0.0;
   out_7601866641244281513[288] = 0.0;
   out_7601866641244281513[289] = 0.0;
   out_7601866641244281513[290] = 0.0;
   out_7601866641244281513[291] = 0.0;
   out_7601866641244281513[292] = 0.0;
   out_7601866641244281513[293] = 0.0;
   out_7601866641244281513[294] = 0.0;
   out_7601866641244281513[295] = 0.0;
   out_7601866641244281513[296] = 0.0;
   out_7601866641244281513[297] = 0.0;
   out_7601866641244281513[298] = 0.0;
   out_7601866641244281513[299] = 0.0;
   out_7601866641244281513[300] = 0.0;
   out_7601866641244281513[301] = 0.0;
   out_7601866641244281513[302] = 0.0;
   out_7601866641244281513[303] = 0.0;
   out_7601866641244281513[304] = 1.0;
   out_7601866641244281513[305] = 0.0;
   out_7601866641244281513[306] = 0.0;
   out_7601866641244281513[307] = 0.0;
   out_7601866641244281513[308] = 0.0;
   out_7601866641244281513[309] = 0.0;
   out_7601866641244281513[310] = 0.0;
   out_7601866641244281513[311] = 0.0;
   out_7601866641244281513[312] = 0.0;
   out_7601866641244281513[313] = 0.0;
   out_7601866641244281513[314] = 0.0;
   out_7601866641244281513[315] = 0.0;
   out_7601866641244281513[316] = 0.0;
   out_7601866641244281513[317] = 0.0;
   out_7601866641244281513[318] = 0.0;
   out_7601866641244281513[319] = 0.0;
   out_7601866641244281513[320] = 0.0;
   out_7601866641244281513[321] = 0.0;
   out_7601866641244281513[322] = 0.0;
   out_7601866641244281513[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_8103375224917938147) {
   out_8103375224917938147[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_8103375224917938147[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_8103375224917938147[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_8103375224917938147[3] = dt*state[12] + state[3];
   out_8103375224917938147[4] = dt*state[13] + state[4];
   out_8103375224917938147[5] = dt*state[14] + state[5];
   out_8103375224917938147[6] = state[6];
   out_8103375224917938147[7] = state[7];
   out_8103375224917938147[8] = state[8];
   out_8103375224917938147[9] = state[9];
   out_8103375224917938147[10] = state[10];
   out_8103375224917938147[11] = state[11];
   out_8103375224917938147[12] = state[12];
   out_8103375224917938147[13] = state[13];
   out_8103375224917938147[14] = state[14];
   out_8103375224917938147[15] = state[15];
   out_8103375224917938147[16] = state[16];
   out_8103375224917938147[17] = state[17];
}
void F_fun(double *state, double dt, double *out_1507526485982590861) {
   out_1507526485982590861[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1507526485982590861[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1507526485982590861[2] = 0;
   out_1507526485982590861[3] = 0;
   out_1507526485982590861[4] = 0;
   out_1507526485982590861[5] = 0;
   out_1507526485982590861[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1507526485982590861[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1507526485982590861[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1507526485982590861[9] = 0;
   out_1507526485982590861[10] = 0;
   out_1507526485982590861[11] = 0;
   out_1507526485982590861[12] = 0;
   out_1507526485982590861[13] = 0;
   out_1507526485982590861[14] = 0;
   out_1507526485982590861[15] = 0;
   out_1507526485982590861[16] = 0;
   out_1507526485982590861[17] = 0;
   out_1507526485982590861[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1507526485982590861[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1507526485982590861[20] = 0;
   out_1507526485982590861[21] = 0;
   out_1507526485982590861[22] = 0;
   out_1507526485982590861[23] = 0;
   out_1507526485982590861[24] = 0;
   out_1507526485982590861[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1507526485982590861[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1507526485982590861[27] = 0;
   out_1507526485982590861[28] = 0;
   out_1507526485982590861[29] = 0;
   out_1507526485982590861[30] = 0;
   out_1507526485982590861[31] = 0;
   out_1507526485982590861[32] = 0;
   out_1507526485982590861[33] = 0;
   out_1507526485982590861[34] = 0;
   out_1507526485982590861[35] = 0;
   out_1507526485982590861[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1507526485982590861[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1507526485982590861[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1507526485982590861[39] = 0;
   out_1507526485982590861[40] = 0;
   out_1507526485982590861[41] = 0;
   out_1507526485982590861[42] = 0;
   out_1507526485982590861[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1507526485982590861[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1507526485982590861[45] = 0;
   out_1507526485982590861[46] = 0;
   out_1507526485982590861[47] = 0;
   out_1507526485982590861[48] = 0;
   out_1507526485982590861[49] = 0;
   out_1507526485982590861[50] = 0;
   out_1507526485982590861[51] = 0;
   out_1507526485982590861[52] = 0;
   out_1507526485982590861[53] = 0;
   out_1507526485982590861[54] = 0;
   out_1507526485982590861[55] = 0;
   out_1507526485982590861[56] = 0;
   out_1507526485982590861[57] = 1;
   out_1507526485982590861[58] = 0;
   out_1507526485982590861[59] = 0;
   out_1507526485982590861[60] = 0;
   out_1507526485982590861[61] = 0;
   out_1507526485982590861[62] = 0;
   out_1507526485982590861[63] = 0;
   out_1507526485982590861[64] = 0;
   out_1507526485982590861[65] = 0;
   out_1507526485982590861[66] = dt;
   out_1507526485982590861[67] = 0;
   out_1507526485982590861[68] = 0;
   out_1507526485982590861[69] = 0;
   out_1507526485982590861[70] = 0;
   out_1507526485982590861[71] = 0;
   out_1507526485982590861[72] = 0;
   out_1507526485982590861[73] = 0;
   out_1507526485982590861[74] = 0;
   out_1507526485982590861[75] = 0;
   out_1507526485982590861[76] = 1;
   out_1507526485982590861[77] = 0;
   out_1507526485982590861[78] = 0;
   out_1507526485982590861[79] = 0;
   out_1507526485982590861[80] = 0;
   out_1507526485982590861[81] = 0;
   out_1507526485982590861[82] = 0;
   out_1507526485982590861[83] = 0;
   out_1507526485982590861[84] = 0;
   out_1507526485982590861[85] = dt;
   out_1507526485982590861[86] = 0;
   out_1507526485982590861[87] = 0;
   out_1507526485982590861[88] = 0;
   out_1507526485982590861[89] = 0;
   out_1507526485982590861[90] = 0;
   out_1507526485982590861[91] = 0;
   out_1507526485982590861[92] = 0;
   out_1507526485982590861[93] = 0;
   out_1507526485982590861[94] = 0;
   out_1507526485982590861[95] = 1;
   out_1507526485982590861[96] = 0;
   out_1507526485982590861[97] = 0;
   out_1507526485982590861[98] = 0;
   out_1507526485982590861[99] = 0;
   out_1507526485982590861[100] = 0;
   out_1507526485982590861[101] = 0;
   out_1507526485982590861[102] = 0;
   out_1507526485982590861[103] = 0;
   out_1507526485982590861[104] = dt;
   out_1507526485982590861[105] = 0;
   out_1507526485982590861[106] = 0;
   out_1507526485982590861[107] = 0;
   out_1507526485982590861[108] = 0;
   out_1507526485982590861[109] = 0;
   out_1507526485982590861[110] = 0;
   out_1507526485982590861[111] = 0;
   out_1507526485982590861[112] = 0;
   out_1507526485982590861[113] = 0;
   out_1507526485982590861[114] = 1;
   out_1507526485982590861[115] = 0;
   out_1507526485982590861[116] = 0;
   out_1507526485982590861[117] = 0;
   out_1507526485982590861[118] = 0;
   out_1507526485982590861[119] = 0;
   out_1507526485982590861[120] = 0;
   out_1507526485982590861[121] = 0;
   out_1507526485982590861[122] = 0;
   out_1507526485982590861[123] = 0;
   out_1507526485982590861[124] = 0;
   out_1507526485982590861[125] = 0;
   out_1507526485982590861[126] = 0;
   out_1507526485982590861[127] = 0;
   out_1507526485982590861[128] = 0;
   out_1507526485982590861[129] = 0;
   out_1507526485982590861[130] = 0;
   out_1507526485982590861[131] = 0;
   out_1507526485982590861[132] = 0;
   out_1507526485982590861[133] = 1;
   out_1507526485982590861[134] = 0;
   out_1507526485982590861[135] = 0;
   out_1507526485982590861[136] = 0;
   out_1507526485982590861[137] = 0;
   out_1507526485982590861[138] = 0;
   out_1507526485982590861[139] = 0;
   out_1507526485982590861[140] = 0;
   out_1507526485982590861[141] = 0;
   out_1507526485982590861[142] = 0;
   out_1507526485982590861[143] = 0;
   out_1507526485982590861[144] = 0;
   out_1507526485982590861[145] = 0;
   out_1507526485982590861[146] = 0;
   out_1507526485982590861[147] = 0;
   out_1507526485982590861[148] = 0;
   out_1507526485982590861[149] = 0;
   out_1507526485982590861[150] = 0;
   out_1507526485982590861[151] = 0;
   out_1507526485982590861[152] = 1;
   out_1507526485982590861[153] = 0;
   out_1507526485982590861[154] = 0;
   out_1507526485982590861[155] = 0;
   out_1507526485982590861[156] = 0;
   out_1507526485982590861[157] = 0;
   out_1507526485982590861[158] = 0;
   out_1507526485982590861[159] = 0;
   out_1507526485982590861[160] = 0;
   out_1507526485982590861[161] = 0;
   out_1507526485982590861[162] = 0;
   out_1507526485982590861[163] = 0;
   out_1507526485982590861[164] = 0;
   out_1507526485982590861[165] = 0;
   out_1507526485982590861[166] = 0;
   out_1507526485982590861[167] = 0;
   out_1507526485982590861[168] = 0;
   out_1507526485982590861[169] = 0;
   out_1507526485982590861[170] = 0;
   out_1507526485982590861[171] = 1;
   out_1507526485982590861[172] = 0;
   out_1507526485982590861[173] = 0;
   out_1507526485982590861[174] = 0;
   out_1507526485982590861[175] = 0;
   out_1507526485982590861[176] = 0;
   out_1507526485982590861[177] = 0;
   out_1507526485982590861[178] = 0;
   out_1507526485982590861[179] = 0;
   out_1507526485982590861[180] = 0;
   out_1507526485982590861[181] = 0;
   out_1507526485982590861[182] = 0;
   out_1507526485982590861[183] = 0;
   out_1507526485982590861[184] = 0;
   out_1507526485982590861[185] = 0;
   out_1507526485982590861[186] = 0;
   out_1507526485982590861[187] = 0;
   out_1507526485982590861[188] = 0;
   out_1507526485982590861[189] = 0;
   out_1507526485982590861[190] = 1;
   out_1507526485982590861[191] = 0;
   out_1507526485982590861[192] = 0;
   out_1507526485982590861[193] = 0;
   out_1507526485982590861[194] = 0;
   out_1507526485982590861[195] = 0;
   out_1507526485982590861[196] = 0;
   out_1507526485982590861[197] = 0;
   out_1507526485982590861[198] = 0;
   out_1507526485982590861[199] = 0;
   out_1507526485982590861[200] = 0;
   out_1507526485982590861[201] = 0;
   out_1507526485982590861[202] = 0;
   out_1507526485982590861[203] = 0;
   out_1507526485982590861[204] = 0;
   out_1507526485982590861[205] = 0;
   out_1507526485982590861[206] = 0;
   out_1507526485982590861[207] = 0;
   out_1507526485982590861[208] = 0;
   out_1507526485982590861[209] = 1;
   out_1507526485982590861[210] = 0;
   out_1507526485982590861[211] = 0;
   out_1507526485982590861[212] = 0;
   out_1507526485982590861[213] = 0;
   out_1507526485982590861[214] = 0;
   out_1507526485982590861[215] = 0;
   out_1507526485982590861[216] = 0;
   out_1507526485982590861[217] = 0;
   out_1507526485982590861[218] = 0;
   out_1507526485982590861[219] = 0;
   out_1507526485982590861[220] = 0;
   out_1507526485982590861[221] = 0;
   out_1507526485982590861[222] = 0;
   out_1507526485982590861[223] = 0;
   out_1507526485982590861[224] = 0;
   out_1507526485982590861[225] = 0;
   out_1507526485982590861[226] = 0;
   out_1507526485982590861[227] = 0;
   out_1507526485982590861[228] = 1;
   out_1507526485982590861[229] = 0;
   out_1507526485982590861[230] = 0;
   out_1507526485982590861[231] = 0;
   out_1507526485982590861[232] = 0;
   out_1507526485982590861[233] = 0;
   out_1507526485982590861[234] = 0;
   out_1507526485982590861[235] = 0;
   out_1507526485982590861[236] = 0;
   out_1507526485982590861[237] = 0;
   out_1507526485982590861[238] = 0;
   out_1507526485982590861[239] = 0;
   out_1507526485982590861[240] = 0;
   out_1507526485982590861[241] = 0;
   out_1507526485982590861[242] = 0;
   out_1507526485982590861[243] = 0;
   out_1507526485982590861[244] = 0;
   out_1507526485982590861[245] = 0;
   out_1507526485982590861[246] = 0;
   out_1507526485982590861[247] = 1;
   out_1507526485982590861[248] = 0;
   out_1507526485982590861[249] = 0;
   out_1507526485982590861[250] = 0;
   out_1507526485982590861[251] = 0;
   out_1507526485982590861[252] = 0;
   out_1507526485982590861[253] = 0;
   out_1507526485982590861[254] = 0;
   out_1507526485982590861[255] = 0;
   out_1507526485982590861[256] = 0;
   out_1507526485982590861[257] = 0;
   out_1507526485982590861[258] = 0;
   out_1507526485982590861[259] = 0;
   out_1507526485982590861[260] = 0;
   out_1507526485982590861[261] = 0;
   out_1507526485982590861[262] = 0;
   out_1507526485982590861[263] = 0;
   out_1507526485982590861[264] = 0;
   out_1507526485982590861[265] = 0;
   out_1507526485982590861[266] = 1;
   out_1507526485982590861[267] = 0;
   out_1507526485982590861[268] = 0;
   out_1507526485982590861[269] = 0;
   out_1507526485982590861[270] = 0;
   out_1507526485982590861[271] = 0;
   out_1507526485982590861[272] = 0;
   out_1507526485982590861[273] = 0;
   out_1507526485982590861[274] = 0;
   out_1507526485982590861[275] = 0;
   out_1507526485982590861[276] = 0;
   out_1507526485982590861[277] = 0;
   out_1507526485982590861[278] = 0;
   out_1507526485982590861[279] = 0;
   out_1507526485982590861[280] = 0;
   out_1507526485982590861[281] = 0;
   out_1507526485982590861[282] = 0;
   out_1507526485982590861[283] = 0;
   out_1507526485982590861[284] = 0;
   out_1507526485982590861[285] = 1;
   out_1507526485982590861[286] = 0;
   out_1507526485982590861[287] = 0;
   out_1507526485982590861[288] = 0;
   out_1507526485982590861[289] = 0;
   out_1507526485982590861[290] = 0;
   out_1507526485982590861[291] = 0;
   out_1507526485982590861[292] = 0;
   out_1507526485982590861[293] = 0;
   out_1507526485982590861[294] = 0;
   out_1507526485982590861[295] = 0;
   out_1507526485982590861[296] = 0;
   out_1507526485982590861[297] = 0;
   out_1507526485982590861[298] = 0;
   out_1507526485982590861[299] = 0;
   out_1507526485982590861[300] = 0;
   out_1507526485982590861[301] = 0;
   out_1507526485982590861[302] = 0;
   out_1507526485982590861[303] = 0;
   out_1507526485982590861[304] = 1;
   out_1507526485982590861[305] = 0;
   out_1507526485982590861[306] = 0;
   out_1507526485982590861[307] = 0;
   out_1507526485982590861[308] = 0;
   out_1507526485982590861[309] = 0;
   out_1507526485982590861[310] = 0;
   out_1507526485982590861[311] = 0;
   out_1507526485982590861[312] = 0;
   out_1507526485982590861[313] = 0;
   out_1507526485982590861[314] = 0;
   out_1507526485982590861[315] = 0;
   out_1507526485982590861[316] = 0;
   out_1507526485982590861[317] = 0;
   out_1507526485982590861[318] = 0;
   out_1507526485982590861[319] = 0;
   out_1507526485982590861[320] = 0;
   out_1507526485982590861[321] = 0;
   out_1507526485982590861[322] = 0;
   out_1507526485982590861[323] = 1;
}
void h_4(double *state, double *unused, double *out_1827372507797762722) {
   out_1827372507797762722[0] = state[6] + state[9];
   out_1827372507797762722[1] = state[7] + state[10];
   out_1827372507797762722[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_598438242728943254) {
   out_598438242728943254[0] = 0;
   out_598438242728943254[1] = 0;
   out_598438242728943254[2] = 0;
   out_598438242728943254[3] = 0;
   out_598438242728943254[4] = 0;
   out_598438242728943254[5] = 0;
   out_598438242728943254[6] = 1;
   out_598438242728943254[7] = 0;
   out_598438242728943254[8] = 0;
   out_598438242728943254[9] = 1;
   out_598438242728943254[10] = 0;
   out_598438242728943254[11] = 0;
   out_598438242728943254[12] = 0;
   out_598438242728943254[13] = 0;
   out_598438242728943254[14] = 0;
   out_598438242728943254[15] = 0;
   out_598438242728943254[16] = 0;
   out_598438242728943254[17] = 0;
   out_598438242728943254[18] = 0;
   out_598438242728943254[19] = 0;
   out_598438242728943254[20] = 0;
   out_598438242728943254[21] = 0;
   out_598438242728943254[22] = 0;
   out_598438242728943254[23] = 0;
   out_598438242728943254[24] = 0;
   out_598438242728943254[25] = 1;
   out_598438242728943254[26] = 0;
   out_598438242728943254[27] = 0;
   out_598438242728943254[28] = 1;
   out_598438242728943254[29] = 0;
   out_598438242728943254[30] = 0;
   out_598438242728943254[31] = 0;
   out_598438242728943254[32] = 0;
   out_598438242728943254[33] = 0;
   out_598438242728943254[34] = 0;
   out_598438242728943254[35] = 0;
   out_598438242728943254[36] = 0;
   out_598438242728943254[37] = 0;
   out_598438242728943254[38] = 0;
   out_598438242728943254[39] = 0;
   out_598438242728943254[40] = 0;
   out_598438242728943254[41] = 0;
   out_598438242728943254[42] = 0;
   out_598438242728943254[43] = 0;
   out_598438242728943254[44] = 1;
   out_598438242728943254[45] = 0;
   out_598438242728943254[46] = 0;
   out_598438242728943254[47] = 1;
   out_598438242728943254[48] = 0;
   out_598438242728943254[49] = 0;
   out_598438242728943254[50] = 0;
   out_598438242728943254[51] = 0;
   out_598438242728943254[52] = 0;
   out_598438242728943254[53] = 0;
}
void h_10(double *state, double *unused, double *out_9090888505167823902) {
   out_9090888505167823902[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_9090888505167823902[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_9090888505167823902[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5725378386810357076) {
   out_5725378386810357076[0] = 0;
   out_5725378386810357076[1] = 9.8100000000000005*cos(state[1]);
   out_5725378386810357076[2] = 0;
   out_5725378386810357076[3] = 0;
   out_5725378386810357076[4] = -state[8];
   out_5725378386810357076[5] = state[7];
   out_5725378386810357076[6] = 0;
   out_5725378386810357076[7] = state[5];
   out_5725378386810357076[8] = -state[4];
   out_5725378386810357076[9] = 0;
   out_5725378386810357076[10] = 0;
   out_5725378386810357076[11] = 0;
   out_5725378386810357076[12] = 1;
   out_5725378386810357076[13] = 0;
   out_5725378386810357076[14] = 0;
   out_5725378386810357076[15] = 1;
   out_5725378386810357076[16] = 0;
   out_5725378386810357076[17] = 0;
   out_5725378386810357076[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5725378386810357076[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5725378386810357076[20] = 0;
   out_5725378386810357076[21] = state[8];
   out_5725378386810357076[22] = 0;
   out_5725378386810357076[23] = -state[6];
   out_5725378386810357076[24] = -state[5];
   out_5725378386810357076[25] = 0;
   out_5725378386810357076[26] = state[3];
   out_5725378386810357076[27] = 0;
   out_5725378386810357076[28] = 0;
   out_5725378386810357076[29] = 0;
   out_5725378386810357076[30] = 0;
   out_5725378386810357076[31] = 1;
   out_5725378386810357076[32] = 0;
   out_5725378386810357076[33] = 0;
   out_5725378386810357076[34] = 1;
   out_5725378386810357076[35] = 0;
   out_5725378386810357076[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5725378386810357076[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5725378386810357076[38] = 0;
   out_5725378386810357076[39] = -state[7];
   out_5725378386810357076[40] = state[6];
   out_5725378386810357076[41] = 0;
   out_5725378386810357076[42] = state[4];
   out_5725378386810357076[43] = -state[3];
   out_5725378386810357076[44] = 0;
   out_5725378386810357076[45] = 0;
   out_5725378386810357076[46] = 0;
   out_5725378386810357076[47] = 0;
   out_5725378386810357076[48] = 0;
   out_5725378386810357076[49] = 0;
   out_5725378386810357076[50] = 1;
   out_5725378386810357076[51] = 0;
   out_5725378386810357076[52] = 0;
   out_5725378386810357076[53] = 1;
}
void h_13(double *state, double *unused, double *out_7964679880147664761) {
   out_7964679880147664761[0] = state[3];
   out_7964679880147664761[1] = state[4];
   out_7964679880147664761[2] = state[5];
}
void H_13(double *state, double *unused, double *out_8209069451045644183) {
   out_8209069451045644183[0] = 0;
   out_8209069451045644183[1] = 0;
   out_8209069451045644183[2] = 0;
   out_8209069451045644183[3] = 1;
   out_8209069451045644183[4] = 0;
   out_8209069451045644183[5] = 0;
   out_8209069451045644183[6] = 0;
   out_8209069451045644183[7] = 0;
   out_8209069451045644183[8] = 0;
   out_8209069451045644183[9] = 0;
   out_8209069451045644183[10] = 0;
   out_8209069451045644183[11] = 0;
   out_8209069451045644183[12] = 0;
   out_8209069451045644183[13] = 0;
   out_8209069451045644183[14] = 0;
   out_8209069451045644183[15] = 0;
   out_8209069451045644183[16] = 0;
   out_8209069451045644183[17] = 0;
   out_8209069451045644183[18] = 0;
   out_8209069451045644183[19] = 0;
   out_8209069451045644183[20] = 0;
   out_8209069451045644183[21] = 0;
   out_8209069451045644183[22] = 1;
   out_8209069451045644183[23] = 0;
   out_8209069451045644183[24] = 0;
   out_8209069451045644183[25] = 0;
   out_8209069451045644183[26] = 0;
   out_8209069451045644183[27] = 0;
   out_8209069451045644183[28] = 0;
   out_8209069451045644183[29] = 0;
   out_8209069451045644183[30] = 0;
   out_8209069451045644183[31] = 0;
   out_8209069451045644183[32] = 0;
   out_8209069451045644183[33] = 0;
   out_8209069451045644183[34] = 0;
   out_8209069451045644183[35] = 0;
   out_8209069451045644183[36] = 0;
   out_8209069451045644183[37] = 0;
   out_8209069451045644183[38] = 0;
   out_8209069451045644183[39] = 0;
   out_8209069451045644183[40] = 0;
   out_8209069451045644183[41] = 1;
   out_8209069451045644183[42] = 0;
   out_8209069451045644183[43] = 0;
   out_8209069451045644183[44] = 0;
   out_8209069451045644183[45] = 0;
   out_8209069451045644183[46] = 0;
   out_8209069451045644183[47] = 0;
   out_8209069451045644183[48] = 0;
   out_8209069451045644183[49] = 0;
   out_8209069451045644183[50] = 0;
   out_8209069451045644183[51] = 0;
   out_8209069451045644183[52] = 0;
   out_8209069451045644183[53] = 0;
}
void h_14(double *state, double *unused, double *out_7516795339953614610) {
   out_7516795339953614610[0] = state[6];
   out_7516795339953614610[1] = state[7];
   out_7516795339953614610[2] = state[8];
}
void H_14(double *state, double *unused, double *out_4561679099068427783) {
   out_4561679099068427783[0] = 0;
   out_4561679099068427783[1] = 0;
   out_4561679099068427783[2] = 0;
   out_4561679099068427783[3] = 0;
   out_4561679099068427783[4] = 0;
   out_4561679099068427783[5] = 0;
   out_4561679099068427783[6] = 1;
   out_4561679099068427783[7] = 0;
   out_4561679099068427783[8] = 0;
   out_4561679099068427783[9] = 0;
   out_4561679099068427783[10] = 0;
   out_4561679099068427783[11] = 0;
   out_4561679099068427783[12] = 0;
   out_4561679099068427783[13] = 0;
   out_4561679099068427783[14] = 0;
   out_4561679099068427783[15] = 0;
   out_4561679099068427783[16] = 0;
   out_4561679099068427783[17] = 0;
   out_4561679099068427783[18] = 0;
   out_4561679099068427783[19] = 0;
   out_4561679099068427783[20] = 0;
   out_4561679099068427783[21] = 0;
   out_4561679099068427783[22] = 0;
   out_4561679099068427783[23] = 0;
   out_4561679099068427783[24] = 0;
   out_4561679099068427783[25] = 1;
   out_4561679099068427783[26] = 0;
   out_4561679099068427783[27] = 0;
   out_4561679099068427783[28] = 0;
   out_4561679099068427783[29] = 0;
   out_4561679099068427783[30] = 0;
   out_4561679099068427783[31] = 0;
   out_4561679099068427783[32] = 0;
   out_4561679099068427783[33] = 0;
   out_4561679099068427783[34] = 0;
   out_4561679099068427783[35] = 0;
   out_4561679099068427783[36] = 0;
   out_4561679099068427783[37] = 0;
   out_4561679099068427783[38] = 0;
   out_4561679099068427783[39] = 0;
   out_4561679099068427783[40] = 0;
   out_4561679099068427783[41] = 0;
   out_4561679099068427783[42] = 0;
   out_4561679099068427783[43] = 0;
   out_4561679099068427783[44] = 1;
   out_4561679099068427783[45] = 0;
   out_4561679099068427783[46] = 0;
   out_4561679099068427783[47] = 0;
   out_4561679099068427783[48] = 0;
   out_4561679099068427783[49] = 0;
   out_4561679099068427783[50] = 0;
   out_4561679099068427783[51] = 0;
   out_4561679099068427783[52] = 0;
   out_4561679099068427783[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_3328892757986285011) {
  err_fun(nom_x, delta_x, out_3328892757986285011);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4963870370355840850) {
  inv_err_fun(nom_x, true_x, out_4963870370355840850);
}
void pose_H_mod_fun(double *state, double *out_7601866641244281513) {
  H_mod_fun(state, out_7601866641244281513);
}
void pose_f_fun(double *state, double dt, double *out_8103375224917938147) {
  f_fun(state,  dt, out_8103375224917938147);
}
void pose_F_fun(double *state, double dt, double *out_1507526485982590861) {
  F_fun(state,  dt, out_1507526485982590861);
}
void pose_h_4(double *state, double *unused, double *out_1827372507797762722) {
  h_4(state, unused, out_1827372507797762722);
}
void pose_H_4(double *state, double *unused, double *out_598438242728943254) {
  H_4(state, unused, out_598438242728943254);
}
void pose_h_10(double *state, double *unused, double *out_9090888505167823902) {
  h_10(state, unused, out_9090888505167823902);
}
void pose_H_10(double *state, double *unused, double *out_5725378386810357076) {
  H_10(state, unused, out_5725378386810357076);
}
void pose_h_13(double *state, double *unused, double *out_7964679880147664761) {
  h_13(state, unused, out_7964679880147664761);
}
void pose_H_13(double *state, double *unused, double *out_8209069451045644183) {
  H_13(state, unused, out_8209069451045644183);
}
void pose_h_14(double *state, double *unused, double *out_7516795339953614610) {
  h_14(state, unused, out_7516795339953614610);
}
void pose_H_14(double *state, double *unused, double *out_4561679099068427783) {
  H_14(state, unused, out_4561679099068427783);
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
