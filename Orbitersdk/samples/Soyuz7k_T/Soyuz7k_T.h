//-----------------------------------------------------------------------------------------
//Copyright 2023 diogom
// 
//Permission is hereby granted, free of charge, to any person obtaining a copy of this softwareand associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and /or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :
//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// 
//-----------------------------------------------------------------------------------------


#ifndef __SOYUZ7K_T
#define __SOYUZ7K_T

#include "orbitersdk.h"

#define DPO_MODE 0
#define DO_MODE 1

#define BDUS_ON 1
#define BDUS_OFF 0

#define CLOCK_ON 0
#define CRONO_TOGGLE 1
#define CRONO_DIGIT 2
#define MET_MIN_DISP 3
#define MET_HOUR_DISP 4
#define MET_DAY_DISP 5
#define MET_CLEAR 6
#define SEC_ADVANCE 7
#define CLOCK_SET 8
#define CLOCK_SET_PRESSED 9
#define BLK_SKDU_ON 10
#define BLK_SKDU_OFF 11
#define CMD_SKDU_ON 12
#define CMD_SKDU_OFF 13
#define IRS_SPEED 14
#define IRS_RANGE_10 15
#define IRS_RANGE_100 16
#define IRS_SPEED_NEEDLE 17
#define IRS_RANGE_NEEDLE 18

#define CM_JETT_BUTTON 19
#define EMERG_UNDOCK_BUTTON 20

#define ELS_1 21
#define ELS_2 22
#define ELS_3 23
#define ELS_4 24
#define ELS_5 25
#define ELS_6 26
#define ELS_7 27
#define ELS_8 28
#define ELS_9 29
#define ELS_10 30
#define ELS_11 31
#define ELS_12 32
#define ELS_13 33
#define ELS_14 34
#define ELS_15 35
#define ELS_16 36
#define ELS_17 37
#define ELS_18 38
#define ELS_19 39
#define ELS_20 40
#define ELS_21 41
#define ELS_22 42
#define ELS_23 43
#define ELS_24 44
#define ELS_25 45
#define ELS_26 46
#define ELS_27 47
#define ELS_28 48
#define ELS_29 49
#define ELS_30 50
#define ELS_31 51
#define ELS_32 52
#define ELS_33 53
#define ELS_34 54
#define ELS_35 55
#define ELS_36 56

#define SOUND_BUTTON 57
#define ELS_BUTTON 58
#define IKP_BUTTON 59
#define MAIN_BATT_BUTTON 60
#define BCKP_BATT_BUTTON 61

#define KSU_LEFT_BUTTON 62
#define KSU_RIGHT_BUTTON 63
#define KSU_BOTH_BUTTON 64
#define KSU_OFF_BUTTON 65

#define KSUl_1 66
#define KSUl_2 67
#define KSUl_3 68
#define KSUl_4 69
#define KSUl_5 70
#define KSUl_6 71
#define KSUl_7 72
#define KSUl_8 73
#define KSUl_9 74
#define KSUl_10 75
#define KSUl_11 76
#define KSUl_12 77
#define KSUl_13 78
#define KSUl_14 79
#define KSUl_15 80
#define KSUl_16 81
#define KSUl_17 82
#define KSUl_18 83
#define KSUl_19 84
#define KSUl_20 85
#define KSUl_21 86
#define KSUl_22 87
#define KSUl_23 88
#define KSUl_24 89
#define KSUl_A 90
#define KSUl_B 91
#define KSUl_V 92
#define KSUl_G 93
#define KSUl_D 94
#define KSUl_E 95
#define KSUl_ZH 96
#define KSUl_I 97
#define KSUl_K 98
#define KSUl_L 99
#define KSUl_M 100
#define KSUl_N 101
#define KSUl_P 102
#define KSUl_R 103
#define KSUl_S 104
#define KSUl_T 105

#define KSUp_1 110
#define KSUp_2 111
#define KSUp_3 112
#define KSUp_4 113
#define KSUp_5 114
#define KSUp_6 115
#define KSUp_7 116
#define KSUp_8 117
#define KSUp_9 118
#define KSUp_10 119
#define KSUp_11 120
#define KSUp_12 121
#define KSUp_13 122
#define KSUp_14 123
#define KSUp_15 124
#define KSUp_16 125
#define KSUp_17 126
#define KSUp_18 127
#define KSUp_19 128
#define KSUp_20 129
#define KSUp_21 130
#define KSUp_22 131
#define KSUp_23 132
#define KSUp_24 133
#define KSUp_A 134
#define KSUp_B 135
#define KSUp_V 136
#define KSUp_G 137
#define KSUp_D 138
#define KSUp_E 139
#define KSUp_ZH 140
#define KSUp_I 141
#define KSUp_K 142
#define KSUp_L 143
#define KSUp_M 144
#define KSUp_N 145
#define KSUp_P 146
#define KSUp_R 147
#define KSUp_S 148
#define KSUp_T 149

#define KSUl_cyl 150				//KSU rotating cylinders
#define KSUp_cyl 151

#define BTSI_1 152					//BTSI counters
#define BTSI_2 153
#define BTSI_3 154
#define BTSI_4 155
#define BTSI_5 156
#define BTSI_6 157
#define BTSI_DKD 158				//BTSI DKD light

#define INK_PERIOD_KNB 159			//INK period counter adjust knob
#define INK_PERIOD_DIGIT_KNB 160	//INK period counter digit select knob
#define INK_PERIOD 161				//INK period counter
#define INK_SELECT_KNB 162			//INK mode select knob
#define INK_O_KNB 163				//globe orbital rotation adjust knob
#define INK_E_KNB 164				//globe daily rotation adjust knob
#define INK_VIT_KNB 165				//INK orbit counter knob
#define INK_VIT 166					//INK orbit counter
#define INK_UGOL_KNB 167			//INK angle counter knob
#define INK_UGOL 168				//INK angle counter
#define INK_MP 169					//Mesto Posadki light
#define INK_TENSVET 170
#define INK_UST 171

#define VZOR_CENTRAL 172
#define VZOR_PERIPH 173

#define BTSI_1_KNB 174
#define BTSI_2_KNB 175
#define BTSI_3_KNB 176
#define BTSI_4_KNB 177
#define BTSI_5_KNB 178
#define BTSI_6_KNB 179

#define INT_KNB 180
#define INT_NEEDLE 181
#define PREP_RB_BUTTON 182
#define NOPREP_RB_BUTTON 183

#define IKP_PANEL 184

#define VZOR_P_TOGGLE 185
#define VZOR_C_TOGGLE 186

#define INT_SCALE 187


// MESH GROUPS
// groups
static UINT GRP_LANTENNA = 29;
static UINT GRP_RANTENNA = 30;
static UINT grp_iglamain_fold[25] = { 16, 27, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 104, 118, 125, 126 };
static UINT grp_iglamain_gimbal[5] = { 16, 27, 66, 104, 125 };
static UINT GRP_IGLAMAIN_PORT = 49;
static UINT GRP_IGLAMAIN_STBD = 48;
static UINT grp_aft_igla[5] = { 32, 75, 76, 129, 131 };
static UINT grp_fwd_igla[14] = { 1, 2, 11, 12, 14, 15, 25, 36, 67, 68, 69, 101, 105, 106 };
static UINT grp_mid_igla[6] = { 5, 6, 26, 42, 102, 103 };
static UINT GRP_IGLA_SHIELD = 4;
static UINT grp_BO_lights[4] = { 10, 13, 81, 120 };
static UINT grp_BO_lights2[4] = { 127, 128, 131, 132 };
//static UINT grp_CM[25] = { 8, 9, 10, 11, 12, 13, 14, 21, 22, 23, 24, 48, 49, 65, 106, 123, 124, 125, 126, 127, 128, 129, 130, 131, 155 };
static UINT grp_rear_ant1[2] = { 180, 181 };
static UINT grp_rear_ant2[2] = { 183, 184 };

static UINT grp_ion1[3] = { 15, 122, 148 };
static UINT grp_ion2[3] = { 16, 123, 149 };
static UINT grp_ion3[3] = { 17, 124, 150 };
static UINT grp_ion4[3] = { 18, 125, 151 };

//static UINT grp_dock_probe[7] = { 8, 41, 99, 140, 141, 142, 143 };
static UINT grp_dock_probe[3] = { 8, 41, 99 };
static UINT GRP_DOCK_PROBE_SPRING = 82;
static UINT grp_latch1 = 140;
static UINT grp_latch2 = 141;
static UINT grp_latch3 = 142;
static UINT grp_latch4 = 143;

//static UINT grp_main_chute[5] = { 152, 153, 154, 155, 156 };
static UINT grp_main_chute[2] = { 155, 156 };
static UINT grp_drogue_chute[3] = { 157, 158, 159 };

// VC groups
static UINT grp_long = 139;
static UINT grp_lat = 137;
static UINT grp_clk_sec = 34;
static UINT grp_clk_min = 29;
static UINT grp_clk_hour = 28;
static UINT grp_clk_enable = 32;
static UINT grp_crono_sec = 40;
static UINT grp_crono_min = 38;
static UINT grp_crono_hour = 36;
static UINT grp_clock_set = 114;
static UINT grp_CM_jett = 9;
static UINT grp_emerg_undock = 10;
static UINT grp_skdu_on = 11;
static UINT grp_skdu_off = 12;
static UINT grp_block_skdu_on = 13;
static UINT grp_block_skdu_off = 14;
static UINT grp_KSU_left = 15;
static UINT grp_KSU_right = 16;
static UINT grp_KSU_both = 17;
static UINT grp_KSU_off = 18;
static UINT grp_prepRB_button = 19;
static UINT grp_noprepRB_button = 20;
static UINT grp_ELS_button = 21;
static UINT grp_IKP_button = 22;
static UINT grp_BB_button = 23;
static UINT grp_RB_button = 24;
static UINT grp_sound_button = 25;
static UINT grp_IRS_speed = 146;
static UINT grp_IRS_range = 147;
static UINT grp_KSU_cyl_left = 51;
static UINT grp_KSU_cyl_right = 102;
static UINT grp_globe_o_knb = 101;
static UINT grp_globe_e_knb = 100;
static UINT grp_globe = 79;
static UINT grp_ink_select[2] = { 88, 89 };
static UINT grp_period_select[2] = { 98, 99 };
static UINT grp_int_knob[2] = { 105, 106 };
static UINT grp_int_needle = 148;

static UINT grp_KSUl_A = 25;
static UINT grp_KSUl_B = 26;
static UINT grp_KSUl_V = 27;
static UINT grp_KSUl_G = 28;
static UINT grp_KSUl_D = 29;
static UINT grp_KSUl_E = 30;
static UINT grp_KSUl_ZH = 31;
static UINT grp_KSUl_I = 32;
static UINT grp_KSUl_K = 33;
static UINT grp_KSUl_L = 34;
static UINT grp_KSUl_M = 35;
static UINT grp_KSUl_N = 36;
static UINT grp_KSUl_P = 37;
static UINT grp_KSUl_R = 38;
static UINT grp_KSUl_S = 39;
static UINT grp_KSUl_T = 40;

static UINT grp_KSUp_A = 79;
static UINT grp_KSUp_B = 80;
static UINT grp_KSUp_V = 81;
static UINT grp_KSUp_G = 82;
static UINT grp_KSUp_D = 83;
static UINT grp_KSUp_E = 84;
static UINT grp_KSUp_ZH = 85;
static UINT grp_KSUp_I = 86;
static UINT grp_KSUp_K = 87;
static UINT grp_KSUp_L = 88;
static UINT grp_KSUp_M = 89;
static UINT grp_KSUp_N = 90;
static UINT grp_KSUp_P = 91;
static UINT grp_KSUp_R = 92;
static UINT grp_KSUp_S = 93;
static UINT grp_KSUp_T = 94;

static UINT grp_KSUl_1 = 0;
static UINT grp_KSUl_2 = 1;
static UINT grp_KSUl_3 = 2;
static UINT grp_KSUl_4 = 3;
static UINT grp_KSUl_5 = 4;
static UINT grp_KSUl_6 = 5;
static UINT grp_KSUl_7 = 6;
static UINT grp_KSUl_8 = 7;
static UINT grp_KSUl_9 = 8;
static UINT grp_KSUl_10 = 9;
static UINT grp_KSUl_11 = 10;
static UINT grp_KSUl_12 = 11;
static UINT grp_KSUl_13 = 12;
static UINT grp_KSUl_14 = 13;
static UINT grp_KSUl_15 = 14;
static UINT grp_KSUl_16 = 15;
static UINT grp_KSUl_17 = 16;
static UINT grp_KSUl_18 = 17;
static UINT grp_KSUl_19 = 18;
static UINT grp_KSUl_20 = 19;
static UINT grp_KSUl_21 = 20;
static UINT grp_KSUl_22 = 21;
static UINT grp_KSUl_23 = 22;
static UINT grp_KSUl_24 = 23;

static UINT grp_KSUp_1 = 54;
static UINT grp_KSUp_2 = 55;
static UINT grp_KSUp_3 = 56;
static UINT grp_KSUp_4 = 57;
static UINT grp_KSUp_5 = 58;
static UINT grp_KSUp_6 = 59;
static UINT grp_KSUp_7 = 60;
static UINT grp_KSUp_8 = 61;
static UINT grp_KSUp_9 = 62;
static UINT grp_KSUp_10 = 63;
static UINT grp_KSUp_11 = 64;
static UINT grp_KSUp_12 = 65;
static UINT grp_KSUp_13 = 66;
static UINT grp_KSUp_14 = 67;
static UINT grp_KSUp_15 = 68;
static UINT grp_KSUp_16 = 69;
static UINT grp_KSUp_17 = 70;
static UINT grp_KSUp_18 = 71;
static UINT grp_KSUp_19 = 72;
static UINT grp_KSUp_20 = 73;
static UINT grp_KSUp_21 = 74;
static UINT grp_KSUp_22 = 75;
static UINT grp_KSUp_23 = 76;
static UINT grp_KSUp_24 = 77;

static UINT grp_BTSI_knb_1[2] = { 118, 119 };
static UINT grp_BTSI_knb_2[2] = { 120, 121 };
static UINT grp_BTSI_knb_3[2] = { 122, 123 };
static UINT grp_BTSI_knb_4[2] = { 124, 125 };
static UINT grp_BTSI_knb_5[2] = { 126, 127 };
static UINT grp_BTSI_knb_6[2] = { 128, 129 };

// animation ranges
static float ANTENNA_RANGE = (float)(99.0 * RAD);
static float IGLAMAIN_DOWN_RANGE = (float)(113.0 * RAD);
static float IGLAMAIN_PORT_RANGE = (float)(-80.0 * RAD);
static float IGLAMAIN_STBD_RANGE = (float)(80.0 * RAD);
static float FOLD_AFT_IGLA_RANGE = (float)(-177.0 * RAD);
static float FOLD_FWD_IGLA_RANGE = (float)(-90.0 * RAD);
static float FOLD_MID_IGLA_RANGE = (float)(-110.0 * RAD);
static float FOLD_IGLA_SHIELD_RANGE = (float)(130.0 * RAD);
static float FOLD_ION_RANGE = (float)(-88.0 * RAD);
static float FOLD_BO_LIGHTS_RANGE = (float)(50.0 * RAD);
static float IGLA_MAIN_GIMBAL_RANGE = (float)(180.0 * RAD);
static float TWOPI_RANGE = (float)(2 * PI);
static float HALFPI_RANGE = (float)(PI / 2);
static float IRS_SPEED_RANGE = (float)(92 * RAD);
static float IRS_RANGE_RANGE = (float)(98.8 * RAD);
static float INK_SELECT_RANGE = (float)(47.4 * RAD);
static float INK_PERIOD_SELECT_RANGE = (float)(-43 * RAD);
static float INT_KNOB_RANGE = (float)(-315 * RAD);
static float INT_NEEDLE_RANGE = (float)(90.2 * RAD);


#endif