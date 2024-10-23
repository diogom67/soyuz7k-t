//-----------------------------------------------------------------------------------------
//Copyright 2024 diogom
// 
//Permission is hereby granted, free of charge, to any person obtaining a copy of this softwareand associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and /or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :
//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// 
//-----------------------------------------------------------------------------------------

#define STRICT
#define ORBITER_MODULE

#include "Orbitersdk.h"
#include <cstdio>
#include <cmath>
#include <string>
#include <stdio.h>
#include "gcAPI.h"
#include "Salyut4.h"
#include "Soyuz7k_T.h"
#include "XRSound.h"


// ==============================================================
// Some vessel parameters
// ==============================================================
const double  S7K_SIZE			= 5;					// mean radius [m]
const VECTOR3 S7K_CS			= {10.5,17.4,10.5};		// x,y,z cross sections [m^2]
const VECTOR3 S7K_CS_SA			= {3.9,3.83,3.82};		// x,y,z cross sections [m^2]
const VECTOR3 S7K_PMI			= {6.9,8.7,5.3};		// principal moments of inertia (mass-normalised) [m^2]
const VECTOR3 S7K_PMI_SA		= { 0.53,0.54,0.52 };	// principal moments of inertia (mass-normalised) [m^2]
//const VECTOR3 S7K_RD			= {0.025,0.025,0.02};	// {0.05,0.1,0.05};  // rotation drag coefficients
const VECTOR3 S7K_RD			= {0.001,0.001,0.001};	// rotation drag coefficients
const double  S7K_EMPTYMASS		= 6150.0;				// empty vessel mass [kg]
const double  S7K_FUELMASS		= 500.0;				// max fuel mass [kg]
const double  S7K_FUELDPOMASS	= 90.0;					// max fuel mass [kg]
const double  S7K_FUELDOMASS	= 30.0;					// max fuel mass [kg]
const double  S7K_FUELDORESMASS = 10.0;					// max fuel mass [kg]
const double  S7K_FUELSAMASS	= 40.0;					// max fuel mass [kg]
const double  S7K_ISP			= 2726.25;				// fuel-specific impulse [m/s]
const double  S7K_ISP_BACKUP	= 2648.7;				// fuel-specific impulse [m/s]
const double  S7K_SIOISP		= 1471.5;				// RCS fuel-specific impulse [m/s]
const VECTOR3 S7K_COP			= {0,0,0};				// centre of pressure for airfoils [m]
const double  S7K_VLIFT_C		= 2.0;					// chord length [m]
const double  S7K_VLIFT_S		= 2.0;					// wing area [m^2]
const double  S7K_VLIFT_A		= 2.5;					// wing aspect ratio
const double  S7K_HLIFT_C		= 2.0;					// chord length [m]
const double  S7K_HLIFT_S		= 1.5;					// wing area [m^2]
const double  S7K_HLIFT_A		= 2.0;					// wing aspect ratio

const double  S7K_MAXMAINTH  = 4090;             
const double  S7K_MAXBACKUPTH = 4030;
const double  S7K_MAXDPOTH   = 98;
const double  S7K_MAXDPOTH_2 = 196;
const double  S7K_MAXDOTH    = 9.8;
const double  S7K_MAXDOTH_2  = 19.6;
const double  S7K_MAXSADOTH  = 147;

const VECTOR3 S7K_DOCK_POS   = {0,0,3.6};				// docking port location [m]
const VECTOR3 S7K_DOCK_DIR   = {0,0,1};					// docking port approach direction
const VECTOR3 S7K_DOCK_ROT   = {0,1,0};					// docking port alignment direction
const double new_dock_factor = 0.435;

const double GLOBE_E_SPEED = 0.0000117713;				// 23h 35m 52s
const double ANT_SPEED = 0.5;
const double CYL_SPEED = 0.9;
const double DOCK_PROBE_SPEED = 0.00275;
const double LATCHES_SPEED = 0.008;
const double HOOKS_SPEED = 0.0055;
const double TEN_SEC_SPEED = 0.1;
const double MAIN_CHUTE_SPEED = 0.5;
const double ION_SPEED = 2;
const double VZOR_SPEED = 0.05;
const double DOCKOFFSET = 0.56; //og is 0.59
const double IGLA_RANGE = 30000;
const double MESH_SHIFT = 0.63;
const double BASE_POWER_DRAIN = 0.84;		// base power drain, kW
const double MAX_BB_POWER = 144000;			// BB capacity, kWs
const double MAX_RB_POWER = 13608;			// RB capacity, kWs
const double MAX_SA_POWER = 11664;			// SA BB capacity, kWs

// calculate main panel normal vector for rotations/translations 
const double PANEL_XROT = 12.5249 * RAD;
const VECTOR3 panel_normal = { 0, cos(PANEL_XROT), -sin(PANEL_XROT) };
const double att_hold_limit = 2.0;

VECTOR3 bscol = _V(1, 1, 1);

enum MySounds {
	ClockTick, // value 0
	ClockSwitch,
	ChronoButton,
	AlarmSound,
	ButtonIn,
	ButtonOut,
	ValveBlow
};

/*
// Define impact convex hull
static const DWORD ntdvtx = 12;
static TOUCHDOWNVTX tdvtx[ntdvtx] = {
	{_V( 0.707,  0.707, -0.5), 2e4, 1e3, 1.6, 1},
	{_V(-0.707,  0.707, -0.5), 2e4, 1e3, 3.0, 1},
	{_V( 0,  -1, -0.5), 2e4, 1e3, 3.0, 1},
	{_V(-0.5,-0.75,3  ), 2e4, 1e3, 3.0},
	{_V( 0.5,-0.75,3  ), 2e4, 1e3, 3.0},
	{_V(-2.6,-1.1,-1.9), 2e4, 1e3, 3.0},
	{_V( 2.6,-1.1,-1.9), 2e4, 1e3, 3.0},
	{_V(-1,   1.3, 0  ), 2e4, 1e3, 3.0},
	{_V( 1,   1.3, 0  ), 2e4, 1e3, 3.0},
	{_V(-1,   1.3,-2  ), 2e4, 1e3, 3.0},
	{_V( 1,   1.3,-2  ), 2e4, 1e3, 3.0},
	{_V( 0,   0.3,-3.8), 2e4, 1e3, 3.0}
};*/

// Define impact convex hull
static const DWORD ntdvtx = 12;
static TOUCHDOWNVTX tdvtx[ntdvtx] = {
	{_V(0, 0.2, -0.95), 2e4, 1e3, 3.0, 1},
	{_V(-0.1665, -0.1079, -0.95), 2e4, 1e3, 3.0, 1},
	{_V(0.1665, -0.1079, -0.95), 2e4, 1e3, 3.0, 1},
	{_V(0, 1.11, -0.699), 2e4, 1e3, 3.0},
	{_V(0, -1.11, -0.699), 2e4, 1e3, 3.0},
	{_V(-1.11, 0, -0.699), 2e4, 1e3, 3.0},
	{_V(1.11, 0, -0.699), 2e4, 1e3, 3.0},
	{_V(0, 0.938, 0.42), 2e4, 1e3, 3.0},
	{_V(0, -0.938, 0.42), 2e4, 1e3, 3.0},
	{_V(-0.938, 0, 0.42), 2e4, 1e3, 3.0},
	{_V(0.938, 0, 0.42), 2e4, 1e3, 3.0},
	{_V(0, 0, -0.96), 2e4, 1e3, 3.0}
};

// Calculate lift coefficient [Cl] as a function of aoa (angle of attack) over -Pi ... Pi
// Implemented here as a piecewise linear function
double LiftCoeff (double aoa)
{
	int i;
	const int nlift = 9;
	static const double AOA[nlift] = {-180*RAD,-60*RAD,-30*RAD,-1*RAD,15*RAD,20*RAD,25*RAD,60*RAD,180*RAD};
	static const double CL[nlift]  = {       0,      0,   -0.1,     0,   0.2,  0.25,   0.2,     0,      0};
	static const double SCL[nlift] = {(CL[1]-CL[0])/(AOA[1]-AOA[0]), (CL[2]-CL[1])/(AOA[2]-AOA[1]),
		                              (CL[3]-CL[2])/(AOA[3]-AOA[2]), (CL[4]-CL[3])/(AOA[4]-AOA[3]),
									  (CL[5]-CL[4])/(AOA[5]-AOA[4]), (CL[6]-CL[5])/(AOA[6]-AOA[5]),
									  (CL[7]-CL[6])/(AOA[7]-AOA[6]), (CL[8]-CL[7])/(AOA[8]-AOA[7])};
	for (i = 0; i < nlift-1 && AOA[i+1] < aoa; i++);
	return CL[i] + (aoa-AOA[i])*SCL[i];
}

// ==============================================================
// Soyuz7k_T class interface
// ==============================================================

class Soyuz7k_T: public VESSEL4 {
public:
	Soyuz7k_T (OBJHANDLE hVessel, int flightmodel);
	~Soyuz7k_T ();
	void clbkSetClassCaps (FILEHANDLE cfg);
	int clbkConsumeBufferedKey(DWORD key, bool down, char* kstate);
	void clbkPreStep(double SimT, double SimDT, double MJD);
	void clbkPostStep(double SimT, double SimDT, double MJD);
	void clbkLoadStateEx(FILEHANDLE scn, void* vs);
	void clbkSaveState(FILEHANDLE scn);
	void clbkPostCreation();
	bool clbkDrawHUD(int mode, const HUDPAINTSPEC * hps, oapi::Sketchpad * skp);
	int clbkConsumeDirectKey(char* kstate);
	bool clbkLoadVC(int id);
	bool clbkVCRedrawEvent(int id, int event, SURFHANDLE surf);
	bool clbkVCMouseEvent(int id, int event, VECTOR3& p);
	
private:
	void DefineAnimations(int mode);
	void SetRCSMode(int mode);
	void GetAngularVel();
	void GetAngularVelSUS();
	void JettisonModule();
	void ReconfigureSA();
	void IglaData(double SimDT);
	void Igla();
	void IglaOff();
	void Inertial(double MJD, double SimDT);
	void Uncage();
	void PVU(double SimDT, int scale);
	void AccelZ(double SimDT);
	void ClockHandling(double MJD);
	bool METBlt(int which, SURFHANDLE surf);
	bool ELSBlt(int id, SURFHANDLE surf);
	bool KSUBlt(SURFHANDLE surf);
	void BTSIBlt(SURFHANDLE surf, int counter);
	bool INKBlt(SURFHANDLE surf, int flag);
	bool IKPBlt(SURFHANDLE surf);
	void VzorUpdate(SURFHANDLE surf, int screen);
	void SUS(double SimDT);
	void AccelSUS(double SimDT);
	void Electrical(double SimDT);
	static void vliftPre (double aoa, double M, double Re, double *cl, double *cm, double *cd);
	static void vliftSUS (double aoa, double M, double Re, double *cl, double *cm, double *cd);
	static void vliftDrogue (double aoa, double M, double Re, double *cl, double *cm, double *cd);
	static void vliftNull (double aoa, double M, double Re, double *cl, double *cm, double *cd);
	static void hlift (double beta, double M, double Re, double *cl, double *cm, double *cd);
	static void hliftDrogue (double beta, double M, double Re, double *cl, double *cm, double *cd);
	static void hliftNull (double beta, double M, double Re, double *cl, double *cm, double *cd);
	int igla_gimbal_status, antennae_status, temp_ant_status, dock_probe_status, igla_sides_status, rcs_status, engine_status, pulsed_status, ori_ap_status, ori_ap_aux, ori_ap_roll, ori_ap_yaw, ori_ap_pitch, sun_ap_status, sun_ap_aux, sun_ap_roll;
	int sep_status, skin_status, relvel_sample, vzor_status, igla_status, main_chute_status, drogue_chute_status, thermal_status, clock_set_status, CM_jett, latches_status, hooks_status, ssvp_on, undock_commanded, ink_select_status, ink_period_status;
	int xflag = 0, yflag = 0, zflag = 0, killrot = 0, RO_detent[6], trans_lateral_flag, trans_vertical_flag;
	MGROUP_TRANSFORM *fold_Lantenna, *fold_Rantenna, *fold_iglamain_port, *fold_iglamain_stbd, *fold_iglamain_down, *fold_aft_igla, *fold_fwd_igla, *fold_mid_igla;
	MGROUP_TRANSFORM* fold_ion1, * fold_ion2, * fold_ion3, * fold_ion4, * fold_BO_lights, * fold_BO_lights2, * fold_igla_shield, *igla_main_gimbal;
	MGROUP_TRANSFORM *long_rot, *lat_rot, *clk_sec_rot, *clk_min_rot, *clk_hour_rot, *clk_enable_rot, *crono_sec_rot, *crono_min_rot, *crono_hour_rot, *IRS_speed_rot, *IRS_range_rot;
	MGROUP_TRANSFORM *KSU_cyl_left_rot, *KSU_cyl_right_rot;
	MGROUP_TRANSFORM *KSUl_A_tr, *KSUl_B_tr, *KSUl_V_tr, *KSUl_G_tr, *KSUl_D_tr, *KSUl_E_tr, *KSUl_ZH_tr, *KSUl_I_tr, *KSUl_K_tr, *KSUl_L_tr, *KSUl_M_tr, *KSUl_N_tr, *KSUl_P_tr, *KSUl_R_tr, *KSUl_S_tr, *KSUl_T_tr;
	MGROUP_TRANSFORM *KSUp_A_tr, *KSUp_B_tr, *KSUp_V_tr, *KSUp_G_tr, *KSUp_D_tr, *KSUp_E_tr, *KSUp_ZH_tr, *KSUp_I_tr, *KSUp_K_tr, *KSUp_L_tr, *KSUp_M_tr, *KSUp_N_tr, *KSUp_P_tr, *KSUp_R_tr, *KSUp_S_tr, *KSUp_T_tr;
	MGROUP_TRANSFORM *KSUl_1_tr, *KSUl_2_tr, *KSUl_3_tr, *KSUl_4_tr, *KSUl_5_tr, *KSUl_6_tr, *KSUl_7_tr, *KSUl_8_tr, *KSUl_9_tr, *KSUl_10_tr, *KSUl_11_tr, *KSUl_12_tr, *KSUl_13_tr, *KSUl_14_tr, *KSUl_15_tr, *KSUl_16_tr, *KSUl_17_tr, *KSUl_18_tr, *KSUl_19_tr, *KSUl_20_tr, *KSUl_21_tr, *KSUl_22_tr, *KSUl_23_tr, *KSUl_24_tr;
	MGROUP_TRANSFORM *KSUp_1_tr, *KSUp_2_tr, *KSUp_3_tr, *KSUp_4_tr, *KSUp_5_tr, *KSUp_6_tr, *KSUp_7_tr, *KSUp_8_tr, *KSUp_9_tr, *KSUp_10_tr, *KSUp_11_tr, *KSUp_12_tr, *KSUp_13_tr, *KSUp_14_tr, *KSUp_15_tr, *KSUp_16_tr, *KSUp_17_tr, *KSUp_18_tr, *KSUp_19_tr, *KSUp_20_tr, *KSUp_21_tr, *KSUp_22_tr, *KSUp_23_tr, *KSUp_24_tr;
	MGROUP_TRANSFORM *dock_probe, *clock_set_knob, *trans_CM_jett, *trans_block_skdu_on, *trans_block_skdu_off, *trans_skdu_on, *trans_skdu_off, *trans_ELS_button, *trans_sound_button, *trans_IKP_button, *trans_BB_button, *trans_RB_button, *trans_prepRB_button, *trans_noprepRB_button;
	MGROUP_TRANSFORM *trans_KSU_left_button, *trans_KSU_right_button, *trans_KSU_both_button, *trans_KSU_off_button, *trans_emerg_undock, *BTSI_knb_5_tr, *fold_rear_ant1, *fold_rear_ant2;
	MGROUP_TRANSFORM *dock_probe_spring, *main_chute_scale, *drogue_chute_scale, *latch1, *latch2, *latch3, *latch4, *globe_o, *o_knb, *e_knb, *ink_select_knb, *ink_period_knb, *int_knb, *int_needle;
	MGROUP_ROTATE *globe_e;
	UINT anim_antennae, anim_dock_probe, main_chute_deploy, drogue_chute_deploy, anim_clock_set, anim_block_skdu_on, anim_block_skdu_off, anim_skdu_on, anim_skdu_off, anim_ELS_button, anim_IKP_button, anim_CM_jett, anim_BB_button, anim_RB_button, anim_prepRB_button, anim_noprepRB_button;
	UINT anim_igla_gimbal, anim_igla_sides, anim_ion, anim_IRS_speed, anim_IRS_range, anim_KSU_left_button, anim_KSU_right_button, anim_KSU_both_button, anim_KSU_off_button, anim_emerg_undock;
	UINT anim_long, anim_lat, anim_clk_sec, anim_clk_min, anim_clk_hour, anim_clk_enable, anim_crono_sec, anim_crono_min, anim_crono_hour, anim_KSUl_cyl, anim_KSUp_cyl;
	UINT anim_KSUl_sysbutt[16], anim_KSUp_sysbutt[16], anim_KSUl_commbutt[24], anim_KSUp_commbutt[24], anim_latches, anim_globe_o, anim_globe_e, anim_o_knb, anim_e_knb, anim_ink_select_knb, anim_ink_period_knb;
	UINT anim_INT_knob, anim_INT_needle, anim_sound_button, anim_BTSI_knb_5;
	double igla_gimbal_proc, antennae_proc, ion_proc, temp_ant_proc, dock_probe_proc, igla_sides_proc, main_chute_proc, drogue_chute_proc, long_proc, lat_proc, clk_sec_proc, clk_min_proc, clk_hour_proc, clk_enable_proc;
	double crono_sec_proc, crono_min_proc, crono_hour_proc, IRS_speed_proc, IRS_range_proc, latches_proc, hooks_proc, undock_signal, globe_o_proc, globe_e_proc, o_knb_proc, e_knb_proc;
	double ion_err, IR_err, sun_err, sun_err_last, sun_err_plusx, sun_err_minusx, sun_err_plusz, sun_err_minusz;
	double tgt_range, tgt_range_rate, acc_SimDT, sun_mag, tgt_err, tgt_err_yaw, tgt_err_pitch, tgt_err_roll, last_tgt_err_yaw, last_tgt_err_pitch, last_tgt_err_roll, speed_target;
	double deltav, dv_accel, dv_accel_proc, accel_z, do_prop, do_res_prop, dpo_prop, main_prop, sa_prop, phi, theta, altitude, longitude, latitude, radius, last_MJD_clock, last_MJD_crono, last_MJD, delta_MJD, start_MJD, MET_MJD, ink_period;
	double intx, inty, intz, signal_start, globe_o_speed, orbit_count, ugol_pos_proc;
	double charge_current, BB_power, RB_power, SA_BB_power, BB_voltage, RB_voltage, SA_BB_voltage, BB_current, RB_current, SA_BB_current, power_modifier, BB_norm, RB_norm, SA_BB_norm, RB_button_proc, base_power, ikp_power, spotlight_power, ssvp_power, clock_power;
	double PVU_timer, refresh_timer, vzor_timer, btsi_5_counter, btsi_5_buffer;
	int PVU_on, PVU_minute, program_on, PVU_cmd_off, PVU_cmd_seq, BTSI_knb_5_digit, btsi_5_read, manual_settings, thermals_off;
	int BB_status, RB_status, SA_BB_status, INT_status, no_power, combined_power, recharge, VClight_status, prepRB_status, BB_check, RB_check, IKP_check;
	int accel_z_status, crono_digit_changed, block_skdu_on, block_skdu_off, command_skdu_on, command_skdu_off, ELS_check, KSUl_curr_col, KSUp_curr_col, KSUl_cyl_rotating, KSUp_cyl_rotating;
	int ink_o_flag, ink_e_flag, ink_ugol_flag, ink_vit_flag, ugol_pos, last_ugol_pos;
	unsigned int IRS_speed_scale, IRS_range_scale;
	int last_ELS_state[36] = { 0 };
	int first_ELS_load[36] = { 0 };
	int last_KSU_state[16*16] = { 0 };
	int first_KSU_load[16*16] = { 0 };
	ATTACHMENTHANDLE attach1;
	DOCKHANDLE hDock;
	THRUSTER_HANDLE th_main[2], th_retro[2], th_trans_fwd[2], th_DPO[10], th_DO[8], th_group[5], th_SA[6], th_land, th_correct[6];
	PROPELLANT_HANDLE hpr;
	PROPELLANT_HANDLE hpr_dpo, hpr_do, hpr_do_res, hpr_sa, hpr_land;
	BEACONLIGHTSPEC beac1, beac2, beac3, beac4, beac6;
	VECTOR3 beac1_pos, beac2_pos, beac3_pos, beac4_pos, beac5_pos, beac6_pos, avel, panel_normal, globe_e_axis;
	VECTOR3 last_omega, tgt_relpos, tgt_relvel, last_tgtpos, last_tgt_relvel, sun_localpos, sun_unitvect, tgt_unitvect, tgt_globpos, tgt_globPRef, tgt_relPRef, LOS_motion;
	SpotLight *spotlight1, *spotlight2;
	LightEmitter *spotlightVC;
	UINT mesh[18], VC_mesh_idx[4]; 
	MESHHANDLE VC_mesh, VC_panel_mesh, VC_KSU_mesh, VC_Vzor_mesh;
	MESHHANDLE hMesh[18];
	SURFHANDLE surf_int, dyn_tex_first, dyn_tex_second, surf_dynamic_first, surf_dynamic_second, surf_dynamic_third, dyn_tex_KSU_unlit, dyn_tex_KSU_lit, surf_dynamic_KSUl, surf_dynamic_KSUp, hSurfVzor, hSurfVzor2, hSurfVzor3, vzTex, vzTex2, dyn_tex_IKP;
	OBJHANDLE hAttached, hTarget, hMother, hVessel;
	Salyut4 *target_Vessel;
	//PSTREAM_HANDLE stream_DO_0_0, stream_DO_0_1, stream_DO_1_0, stream_DO_1_1, stream_DO_6_0, stream_DO_6_1, stream_DO_7_0, stream_DO_7_1, stream_DO_16_0, stream_DO_16_1, stream_DO_17_0, stream_DO_17_1;
	int igla_ap_pitch, igla_ap_yaw, igla_ap_roll, igla_ap_aux_pitch, igla_ap_aux_yaw, igla_ap_aux_roll, igla_ap_aux_vert, igla_ap_aux_lat, igla_ap_vert, igla_ap_lat, ori_ap_aux_yaw, ori_ap_aux_pitch, ori_ap_aux_roll;
	int target_oriented, burn_seq, igla_idle, orb_ori_dir, first_BTSI_DKD_load, last_BTSI_DKD_state, first_MP_load, last_MP_state, igla_LOS_burn, SUS_status, mouse_downl;
	int clock_enable, crono_enable, MET_day, MET_hour, MET_minute, KSU_left_enable, KSU_right_enable, BDUS_status, BDUS_integr, inertial_status, fuel_alarm, avaria, modes_off, man_appr, ballistic, AKSP_status, vzor_c_on, vzor_p_on;
	double side_null_fact = 0.2915;
	double vert_null_fact = 0.4502;
	double side_thrust, vert_thrust, igla_4_limit, curr_slip, last_slip, curr_pitch, last_pitch, curr_roll, last_roll;
	double timer = 0;
	double SUS_timestep_accum_x, SUS_timestep_accum_y, SUS_timestep_accum_z, Ux, x_duration, Uy, y_duration, Uz, z_duration;
	//double K0y = 1.0/6.0;
	//double K0z = 1.0/4.0;
	double K1x = 1.0/2.0;
	//double K1y = 1.0/1.4;
	//double K1z = 1.0/2.0;
	double K2x = 1.0/4.3;
	//double K2y = 1.0/3.0;
	//double K2z = 1.0/3.0;
	double thrust_rot, thrust_trans_dpo, thrust_main, mesh_shift_z, mesh_shift_y;
	double KSUl_cyl_proc, KSUp_cyl_proc, KSUl_rot_tgt, KSUp_rot_tgt, KSUl_rot_acc, KSUp_rot_acc, roll_drift_vector, accel_SUS;
	char targetName[10];
	int rotation_command[6]; //pitchup - pitchdown - yawleft - yawright - rollleft - rollright
	int DEBUG, btsi_5_flag, IKP_update, KSU_update, INK_update, MET_update, backup_DO;
	XRSound *m_pXRSound;
	AIRFOILHANDLE airfoil_h, airfoil_v;
	ORBITPARAM orbparam;
	ELEMENTS el;
	CAMERAHANDLE hCam_vzor_nad, hCam_vzor_fwd, hCam_vzor_periph;
	PARTICLESTREAMSPEC landing_contrail = {
		0, 2.0, 3, 200.0, 0.25, 6.0, 11, 10.0, PARTICLESTREAMSPEC::DIFFUSE,
		PARTICLESTREAMSPEC::LVL_PSQRT, 0, 0.7,
		PARTICLESTREAMSPEC::ATM_PLOG, 1e-6, 0.1
	};
	PARTICLESTREAMSPEC landing_exhaust = {
		0, 1.0, 40, 250.0, 0.04, 0.2, 20, 6.0, PARTICLESTREAMSPEC::EMISSIVE,
		PARTICLESTREAMSPEC::LVL_SQRT, 1, 1,
		PARTICLESTREAMSPEC::ATM_FLAT, 1, 1
	};
	PARTICLESTREAMSPEC main_exhaust = {
		0, 0.9, 60.0, S7K_ISP, 0.0, 0.03, 1300.0, 2.0, PARTICLESTREAMSPEC::EMISSIVE,
		PARTICLESTREAMSPEC::LVL_PSQRT, 0.0, 0.5,
		PARTICLESTREAMSPEC::ATM_FLAT, 0.15, 0.15,
		NULL
	};
	PARTICLESTREAMSPEC DPO_exhaust = {
		0, 0.01, 60.0, S7K_SIOISP, 0.0, 0.01, 600.0, 2.0, PARTICLESTREAMSPEC::EMISSIVE,
		PARTICLESTREAMSPEC::LVL_PSQRT, 0.0, 0.5,
		PARTICLESTREAMSPEC::ATM_FLAT, 0.15, 0.15,
		NULL
	};
	PARTICLESTREAMSPEC DO_exhaust = {
		0, 0.001, 60.0, S7K_SIOISP, 0.0, 0.01, 400.0, 2.0, PARTICLESTREAMSPEC::EMISSIVE,
		PARTICLESTREAMSPEC::LVL_PSQRT, 0.0, 0.5,
		PARTICLESTREAMSPEC::ATM_FLAT, 0.15, 0.15,
		NULL
	};
};

Soyuz7k_T::Soyuz7k_T (OBJHANDLE hVessel, int flightmodel)
: VESSEL4 (hVessel, flightmodel)
{
	DefineAnimations(0);
	Soyuz7k_T::hVessel = hVessel;
}

Soyuz7k_T::~Soyuz7k_T ()
{
	delete m_pXRSound;
	delete KSU_cyl_left_rot;
	delete KSU_cyl_right_rot;
	delete fold_Lantenna, fold_Rantenna, fold_iglamain_port, fold_iglamain_stbd, fold_iglamain_down, fold_aft_igla, fold_fwd_igla, fold_mid_igla;
	delete fold_ion1, fold_ion2, fold_ion3, fold_ion4, fold_BO_lights, fold_BO_lights2, fold_igla_shield, igla_main_gimbal;
	delete long_rot, lat_rot, clk_sec_rot, clk_min_rot, clk_hour_rot, clk_enable_rot, crono_sec_rot, crono_min_rot, crono_hour_rot, IRS_speed_rot, IRS_range_rot;
	delete KSUl_A_tr, KSUl_B_tr, KSUl_V_tr, KSUl_G_tr, KSUl_D_tr, KSUl_E_tr, KSUl_ZH_tr, KSUl_I_tr, KSUl_K_tr, KSUl_L_tr, KSUl_M_tr, KSUl_N_tr, KSUl_P_tr, KSUl_R_tr, KSUl_S_tr, KSUl_T_tr;
	delete KSUp_A_tr, KSUp_B_tr, KSUp_V_tr, KSUp_G_tr, KSUp_D_tr, KSUp_E_tr, KSUp_ZH_tr, KSUp_I_tr, KSUp_K_tr, KSUp_L_tr, KSUp_M_tr, KSUp_N_tr, KSUp_P_tr, KSUp_R_tr, KSUp_S_tr, KSUp_T_tr;
	delete KSUl_1_tr, KSUl_2_tr, KSUl_3_tr, KSUl_4_tr, KSUl_5_tr, KSUl_6_tr, KSUl_7_tr, KSUl_8_tr, KSUl_9_tr, KSUl_10_tr, KSUl_11_tr, KSUl_12_tr, KSUl_13_tr, KSUl_14_tr, KSUl_15_tr, KSUl_16_tr, KSUl_17_tr, KSUl_18_tr, KSUl_19_tr, KSUl_20_tr, KSUl_21_tr, KSUl_22_tr, KSUl_23_tr, KSUl_24_tr;
	delete KSUp_1_tr, KSUp_2_tr, KSUp_3_tr, KSUp_4_tr, KSUp_5_tr, KSUp_6_tr, KSUp_7_tr, KSUp_8_tr, KSUp_9_tr, KSUp_10_tr, KSUp_11_tr, KSUp_12_tr, KSUp_13_tr, KSUp_14_tr, KSUp_15_tr, KSUp_16_tr, KSUp_17_tr, KSUp_18_tr, KSUp_19_tr, KSUp_20_tr, KSUp_21_tr, KSUp_22_tr, KSUp_23_tr, KSUp_24_tr;
	delete dock_probe, clock_set_knob, trans_CM_jett, trans_block_skdu_on, trans_block_skdu_off, trans_skdu_on, trans_skdu_off, trans_ELS_button, trans_IKP_button, trans_BB_button, trans_RB_button, trans_prepRB_button, trans_noprepRB_button;
	delete trans_KSU_left_button, trans_KSU_right_button, trans_KSU_both_button, trans_KSU_off_button, trans_emerg_undock, trans_sound_button, BTSI_knb_5_tr, fold_rear_ant1, fold_rear_ant2;
	delete dock_probe_spring, main_chute_scale, drogue_chute_scale, latch1, latch2, latch3, latch4, globe_o, globe_e, o_knb, e_knb, ink_select_knb, ink_period_knb, int_knb, int_needle;
	if (vzTex)	oapiReleaseTexture(vzTex);
	if (vzTex2)	oapiReleaseTexture(vzTex2);
	if (hCam_vzor_nad)	gcDeleteCustomCamera(hCam_vzor_nad);
	if (hCam_vzor_fwd)	gcDeleteCustomCamera(hCam_vzor_fwd);
	if (hCam_vzor_periph)	gcDeleteCustomCamera(hCam_vzor_periph);
}


// animation transformation definitions
// coordinates for animation reference and axis
static VECTOR3 LANTENNA_REF = { 1.325,0,-1.57 };
static VECTOR3 LANTENNA_AXIS = { 0,1,0 };
static VECTOR3 RANTENNA_REF = { -1.325,0,-1.57 };
static VECTOR3 RANTENNA_AXIS = { 0,-1,0 };
static VECTOR3 IGLAMAIN_DOWN_REF = { 0, 1.08, 1.74 };
static VECTOR3 IGLAMAIN_DOWN_AXIS = { -1, 0, 0 };
static VECTOR3 IGLAMAIN_PORT_REF = { -0.105, 1.65, 1.38 };
static VECTOR3 IGLAMAIN_PORT_AXIS = { 0, -0.912, 0.41 };
static VECTOR3 IGLAMAIN_STBD_REF = { 0.105, 1.65, 1.38 };
static VECTOR3 IGLAMAIN_STBD_AXIS = { 0, -0.912, 0.41 };
static VECTOR3 FOLD_AFT_IGLA_REF = { -1.082, -0.357, -3.5 };
static VECTOR3 FOLD_AFT_IGLA_AXIS = { 0.707, -0.707, 0 };
static VECTOR3 FOLD_FWD_IGLA_REF = { 0.668, 0, 3.515 };
static VECTOR3 FOLD_FWD_IGLA_AXIS = { 0, -1, 0 };
static VECTOR3 FOLD_MID_IGLA_REF = { -1.115, 0, 1.953 };
static VECTOR3 FOLD_MID_IGLA_AXIS = { 0, -1, 0 };
static VECTOR3 FOLD_IGLA_SHIELD_REF = { 0, 0.93, 3.02 };
static VECTOR3 FOLD_IGLA_SHIELD_AXIS = { -1, 0, 0 };
static VECTOR3 FOLD_ION1_REF = { -0.715, -1.05, -3.42 };
static VECTOR3 FOLD_ION2_REF = { 1.05, -0.715, -3.42 };
static VECTOR3 FOLD_ION3_REF = { 0.715, 1.05, -3.42 };
static VECTOR3 FOLD_ION4_REF = { -1.05, 0.715, -3.42 };
static VECTOR3 FOLD_ION_AXIS = { 0, 0, -1 };
static VECTOR3 FOLD_BO_LIGHTS_REF = { -0.28, -0.76, 1.35 };
static VECTOR3 FOLD_BO_LIGHTS2_REF = { -0.286447, 0.751905, 1.35344 };
static VECTOR3 FOLD_BO_LIGHTS_AXIS = { -0.981, 0.196, 0 };
static VECTOR3 FOLD_BO_LIGHTS2_AXIS = { -0.981, -0.196, 0 };
static VECTOR3 IGLA_MAIN_GIMBAL_REF = { 0, 3, 1.241 };
static VECTOR3 IGLA_MAIN_GIMBAL_AXIS = { -1, 0, 0 };
static VECTOR3 FOLD_REAR_ANT1_REF = { -0.213133, 1.16588, -3.43256 };
static VECTOR3 FOLD_REAR_ANT1_AXIS = { 0.978534, 0.206087, 0 };
static VECTOR3 FOLD_REAR_ANT2_REF = { -0.214334, -1.1618, -3.43062 };
static VECTOR3 FOLD_REAR_ANT2_AXIS = { 0.976062, -0.217495, 0 };

// VC objects
static VECTOR3 PAN_NORM_AXIS = panel_normal;
static VECTOR3 LONG_REF = { -0.352198, -0.550398, 0.591709 };
static VECTOR3 LONG_AXIS = { 0, -panel_normal.z, panel_normal.y };
static VECTOR3 LAT_REF = { -0.437891, -0.568983, 0.509842 };
static VECTOR3 LAT_AXIS = { -1, 0, 0 };
static VECTOR3 CLK_SEC_REF = { -0.106048, -0.569951, 0.332911 };
static VECTOR3 CLK_MIN_REF = { -0.106035, -0.570678, 0.33304 };
static VECTOR3 CLK_HOUR_REF = { -0.106014, -0.57143, 0.333212 };
static VECTOR3 CLK_ENABLE_REF = { -0.214873, -0.579814, 0.28954 };
static VECTOR3 CRONO_SEC_REF = { -0.154545, -0.575583, 0.313064 };
static VECTOR3 CRONO_MIN_REF = { -0.18114, -0.575583, 0.313064 };
static VECTOR3 CRONO_HOUR_REF = { -0.207751, -0.575583, 0.313064 };
static VECTOR3 IRS_SPEED_REF = { 0.053282, -0.57373, 0.339595 };
static VECTOR3 IRS_RANGE_REF = { 0.052049, -0.582744, 0.299022 };
static VECTOR3 CYL_ROT_AXIS = { 0, 0.295135, 0.955455 };
static VECTOR3 CYL_LEFT_REF = { -0.585846, -0.425616, 0.454723 };
static VECTOR3 CYL_RIGHT_REF = { 0.602587, -0.425798, 0.454856 };
static VECTOR3 KSUl_SYSBUTT_NORM = { -0.0149049, 0.955367, -0.295044 };
static VECTOR3 KSUl_MISCBUTT_NORM = { 0.475674, 0.840444, -0.259592 };
static VECTOR3 KSUp_SYSBUTT_NORM = { 0.0247207, 0.955162, -0.295048 };
static VECTOR3 KSUp_MISCBUTT_NORM = { -0.47489, 0.840831, -0.259774 };
static VECTOR3 GLOBE_REF = { -0.354703, -0.582185, 0.513579 };
static VECTOR3 O_KNB_REF = { -0.259515, -0.540599, 0.44688 };
static VECTOR3 E_KNB_REF = { -0.259515, -0.530935, 0.444733 };
static VECTOR3 INK_SELECT_KNB_REF = { -0.263337, -0.515346, 0.559348 };
static VECTOR3 INK_PERIOD_KNB_REF = { -0.215552, -0.520568, 0.515959 };
static VECTOR3 INK_VIT_KNB_REF = { -0.178357, -0.532508, 0.471323 };
static VECTOR3 INK_UGOL_KNB_REF = { -0.1827, -0.539757, 0.438694 };
static VECTOR3 INK_TENSVET_KNB_REF = { -0.405258, -0.541814, 0.429432 };
static VECTOR3 INK_UST_KNB_REF = { -0.305634, -0.541814, 0.429432 };
static VECTOR3 GLOBE_E_AXIS_INIT = { 0.61896, -0.76673, 0.170334 };
static VECTOR3 INT_KNB_REF = { -0.412873, -0.574465, 0.297633 };
static VECTOR3 INT_NEEDLE_REF = { -0.412873, -0.581298, 0.299151 };

// scale
static VECTOR3 DOCK_PROBE_SHIFT = { 0, 0, 0.347 };
static VECTOR3 DOCK_PROBE_SPRING_REF = { 0, 0, 4.02 };
static VECTOR3 DOCK_PROBE_SPRING_SCALE = { 1, 1, -90 };
static VECTOR3 CLOCK_SET_SHIFT = PAN_NORM_AXIS * -0.0015;
static VECTOR3 BUTTONS_SHIFT = PAN_NORM_AXIS * -0.004;
static VECTOR3 KSUl_SYS_BUTTONS_SHIFT = KSUl_SYSBUTT_NORM * -0.004;
static VECTOR3 KSUl_MISC_BUTTONS_SHIFT = KSUl_MISCBUTT_NORM * -0.0025;
static VECTOR3 KSUp_SYS_BUTTONS_SHIFT = KSUp_SYSBUTT_NORM * -0.004;
static VECTOR3 KSUp_MISC_BUTTONS_SHIFT = KSUp_MISCBUTT_NORM * -0.0025;
static VECTOR3 BTSI_KNB_SHIFT = { 0.010569, 0, 0};

static VECTOR3 MAIN_CHUTE_REF = { 0.000, 0.000, -0.84 };
static VECTOR3 DROGUE_CHUTE_REF = { 0.000, 0.000, -0.84 };
static VECTOR3 MAIN_CHUTE_SCALE = { 65, 65, 45 };
static VECTOR3 DROGUE_CHUTE_SCALE = { 16, 16, 12 };


void Soyuz7k_T::clbkSaveState(FILEHANDLE scn) {

	char cbuf[256];

	// default vessel parameters
	VESSEL4::clbkSaveState(scn);

	// store animation settings
	sprintf(cbuf, "%d %lf %d %lf %d %lf %d %lf", antennae_status, antennae_proc, igla_sides_status, igla_sides_proc, thermal_status, ion_proc, main_chute_status, main_chute_proc);
	oapiWriteScenario_string(scn, "ANIM", cbuf);

	// store docking probe status
	sprintf(cbuf, "%d %lf %d %lf %d %lf", dock_probe_status, dock_probe_proc, latches_status, latches_proc, hooks_status, hooks_proc);
	oapiWriteScenario_string(scn, "PROBE", cbuf);

	// store DPO/DO config
	int rcs_store;
	if (rcs_status == 0 || rcs_status == 3) rcs_store = 3;
	else if (rcs_status == 2 || rcs_status == 1) rcs_store = 1;
	else rcs_store = 6;
	sprintf(cbuf, "%d", rcs_store);
	oapiWriteScenario_string(scn, "RCS", cbuf);

	// store Igla target name
	sprintf(cbuf, "%s", targetName);
	oapiWriteScenario_string(scn, "TGT", cbuf);

	// store current module config status
	sprintf(cbuf, "%d %d %d", sep_status, skin_status, fuel_alarm);
	oapiWriteScenario_string(scn, "MOD", cbuf);

	// store launch time - static
	sprintf(cbuf, "%lf", start_MJD);
	oapiWriteScenario_string(scn, "MJD", cbuf);

	// store KSU selected columns
	sprintf(cbuf, "%d %d", KSUl_curr_col, KSUp_curr_col);
	oapiWriteScenario_string(scn, "KSU", cbuf);

	// store Orient mode state
	sprintf(cbuf, "%d %d %d %d %d %lf %lf %lf %d", ori_ap_status, ori_ap_aux, ori_ap_roll, ori_ap_aux_yaw, ori_ap_aux_pitch, last_slip, last_pitch, last_roll, orb_ori_dir);
	oapiWriteScenario_string(scn, "ORNT", cbuf);

	// store Solar mode state
	sprintf(cbuf, "%d %d %d", sun_ap_status, sun_ap_aux, sun_ap_roll);
	oapiWriteScenario_string(scn, "SUN", cbuf);

	// store Inertial mode state
	sprintf(cbuf, "%d %lf %lf %lf %d", inertial_status, intx, inty, intz, BDUS_status);
	oapiWriteScenario_string(scn, "INRT", cbuf);

	// store PVU state
	sprintf(cbuf, "%d %d %d %lf %d %lf", program_on, PVU_cmd_seq, PVU_cmd_off, PVU_timer, accel_z_status, accel_z);
	oapiWriteScenario_string(scn, "PVU", cbuf);

	// store whether docking assembly has been jettisoned or not
	sprintf(cbuf, "%d", CM_jett);
	oapiWriteScenario_string(scn, "CM", cbuf);

	// store globe and its knobs' initial position
	sprintf(cbuf, "%lf %lf %lf %lf", globe_o_proc, globe_e_proc, o_knb_proc, o_knb_proc);
	oapiWriteScenario_string(scn, "GLOBE", cbuf);

	// store INK parameters
	sprintf(cbuf, "%lf %lf %lf %d", ink_period, orbit_count, ugol_pos_proc, ink_select_status);
	oapiWriteScenario_string(scn, "INK", cbuf);
	
	// store INT parameters
	sprintf(cbuf, "%d", INT_status);
	oapiWriteScenario_string(scn, "INT", cbuf);

	// store camera (vzor) parameters
	sprintf(cbuf, "%d", vzor_status);
	oapiWriteScenario_string(scn, "CAM", cbuf);

	// store power parameters

	BB_norm = BB_power / MAX_BB_POWER;
	RB_norm = RB_power / MAX_RB_POWER;
	SA_BB_norm = SA_BB_power / MAX_SA_POWER; 
	sprintf(cbuf, "%lf %d %lf %d %lf", BB_norm, BB_status, RB_norm, RB_status, SA_BB_norm);
	oapiWriteScenario_string(scn, "PWR", cbuf);

	// store BTSI data and related
	sprintf(cbuf, "%d %lf %d %d %lf", manual_settings, btsi_5_counter, btsi_5_flag, btsi_5_read, dv_accel);
	oapiWriteScenario_string(scn, "BTSI", cbuf);
}

void Soyuz7k_T::clbkLoadStateEx(FILEHANDLE scn, void* vs) {

	char* line;

	while (oapiReadScenario_nextline(scn, line)) {
		// load animation status
		if (!_strnicmp(line, "ANIM", 4)) {
			sscanf(line + 4, "%d%lf%d%lf%d%lf%d%lf", &antennae_status, &antennae_proc, &igla_sides_status, &igla_sides_proc, &thermal_status, &ion_proc, &main_chute_status, &main_chute_proc);
		}
		// load dock probe status
		else if (!_strnicmp(line, "PROBE", 5)) {
			sscanf(line + 5, "%d%lf%d%lf%d%lf", &dock_probe_status, &dock_probe_proc, &latches_status, &latches_proc, &hooks_status, &hooks_proc);
		}
		// load DPO/DO status
		else if (!_strnicmp(line, "RCS", 3)) {
			sscanf(line + 3, "%d", &rcs_status);
		}
		// load Igla target name
		else if (!_strnicmp(line, "TGT", 3)) {
			sscanf(line + 3, "%s", targetName);
		}
		// load module config status
		else if (!_strnicmp(line, "MOD", 3)) {
			sscanf(line + 3, "%d%d%d", &sep_status, &skin_status, &fuel_alarm);
		}
		// load launch time
		else if (!_strnicmp(line, "MJD", 3)) {
			sscanf(line + 3, "%lf", &start_MJD);
		}
		// load KSU selected columns
		else if (!_strnicmp(line, "KSU", 3)) {
			sscanf(line + 3, "%d%d", &KSUl_curr_col, &KSUp_curr_col);
		}
		// load Orient mode state
		else if (!_strnicmp(line, "ORNT", 4)) {
			sscanf(line + 4, "%d%d%d%d%d%lf%lf%lf%d", &ori_ap_status, &ori_ap_aux, &ori_ap_roll, &ori_ap_aux_yaw, &ori_ap_aux_pitch, &last_slip, &last_pitch, &last_roll, &orb_ori_dir);
		}
		// load Sun mode state
		else if (!_strnicmp(line, "SUN", 3)) {
			sscanf(line + 3, "%d%d%d", &sun_ap_status, &sun_ap_aux, &sun_ap_roll);
		}
		// load Inertial mode state
		else if (!_strnicmp(line, "INRT", 4)) {
			sscanf(line + 4, "%d%lf%lf%lf%d", &inertial_status, &intx, &inty, &intz, &BDUS_status);
		}
		// load PVU state
		else if (!_strnicmp(line, "PVU", 3)) {
			sscanf(line + 3, "%d%d%d%lf%d%lf", &program_on, &PVU_cmd_seq, &PVU_cmd_off, &PVU_timer, &accel_z_status, &accel_z);
		}
		// load docking assembly status
		else if (!_strnicmp(line, "CM", 2)) {
			sscanf(line + 2, "%d", &CM_jett);
		}
		// load globe starting conditions
		else if (!_strnicmp(line, "GLOBE", 5)) {
			sscanf(line + 5, "%lf%lf%lf%lf", &globe_o_proc, &globe_e_proc, &o_knb_proc, &o_knb_proc);
		}
		// load ink starting conditions
		else if (!_strnicmp(line, "INK", 3)) {
			sscanf(line + 3, "%lf%lf%lf%d", &ink_period, &orbit_count, &ugol_pos_proc, &ink_select_status);
		}
		else if (!_strnicmp(line, "INT", 3)) {
			sscanf(line + 3, "%d", &INT_status);
		}
		else if (!_strnicmp(line, "CAM", 3)) {
			sscanf(line + 3, "%d", &vzor_status);
		}
		else if (!_strnicmp(line, "PWR", 3)) {
			sscanf(line + 3, "%lf%d%lf%d%lf", &BB_norm, &BB_status, &RB_norm, &RB_status, &SA_BB_norm);
		}
		else if (!_strnicmp(line, "BTSI", 4)) {
			sscanf(line + 3, "%d%lf%d%d%lf", &manual_settings, &btsi_5_counter, &btsi_5_flag, &btsi_5_read, &dv_accel);
		}
		else {
			ParseScenarioLineEx(line, vs);
			// unrecognised option - pass to Orbiter's generic parser
		}
	}
}

void Soyuz7k_T::clbkPostCreation() {

	// Set battery states
	BB_power = BB_norm * MAX_BB_POWER;
	RB_power = RB_norm * MAX_RB_POWER;
	SA_BB_power = SA_BB_norm * MAX_SA_POWER;
	if (BB_norm > 1) BB_power = MAX_BB_POWER;
	if (RB_norm > 1) RB_power = MAX_RB_POWER;
	if (SA_BB_norm > 1) SA_BB_power = MAX_SA_POWER;
	
	// IKP power from current state
	if (!program_on) ikp_power = 0.0108;		// kW, 400 mA at 27 V
	else ikp_power = 0.0162;

	// Clock power from current state
	if (clock_enable) clock_power = 0.0002;
	if (crono_enable == 1) clock_power += 0.0002;

	// External lighting power from current state
	if (beac1.active == true) spotlight_power = 0.11;

	if (rcs_status == 6) {
		SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
		SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
		SetThrusterGroupLevel(THGROUP_ATT_UP, 0);
		SetThrusterGroupLevel(THGROUP_ATT_DOWN, 0);
		SetThrusterGroupLevel(THGROUP_ATT_LEFT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_FORWARD, 0);
		SetThrusterGroupLevel(THGROUP_ATT_BACK, 0);
		SetThrusterGroupLevel(THGROUP_RETRO, 0);
		DelThrusterGroup(THGROUP_ATT_YAWLEFT, false);
		DelThrusterGroup(THGROUP_ATT_YAWRIGHT, false);
		DelThrusterGroup(THGROUP_ATT_PITCHUP, false);
		DelThrusterGroup(THGROUP_ATT_PITCHDOWN, false);
		DelThrusterGroup(THGROUP_ATT_BANKLEFT, false);
		DelThrusterGroup(THGROUP_ATT_BANKRIGHT, false);
		DelThrusterGroup(THGROUP_ATT_UP, false);
		DelThrusterGroup(THGROUP_ATT_DOWN, false);
		DelThrusterGroup(THGROUP_ATT_LEFT, false);
		DelThrusterGroup(THGROUP_ATT_RIGHT, false);
		DelThrusterGroup(THGROUP_ATT_FORWARD, false);
		DelThrusterGroup(THGROUP_ATT_BACK, false);
		DelThrusterGroup(THGROUP_RETRO, false);
	}

	// in case CM has been jettisoned previously
	if (CM_jett == 1) {
		DelDock(hDock);
		hDock = NULL;
		DelAnimation(anim_dock_probe);
		DelMesh(mesh[2], true);
		VECTOR3 ofs = _V(0, 0, mesh_shift_z);
		hMesh[2] = oapiLoadMeshGlobal("Soyuz_7k\\T_BO_noCM");
		mesh[2] = AddMesh(hMesh[2], &ofs);
	}

	// set up starting module status
	if (sep_status == 1 || sep_status == 2) {
		DelMesh(mesh[2]);
		SetSize(S7K_SIZE - 1.5);
		SetEmptyMass(S7K_EMPTYMASS - 1350);
		if (CM_jett == 0) DelDock(hDock);
		ShiftCG(_V(0, 0, -0.5));
		DelBeacon(&beac1);
		DelBeacon(&beac2);
		DelBeacon(&beac6);
		DelLightEmitter(spotlight1);
		DelLightEmitter(spotlight2);
		DelAttachment(attach1);
		DelAnimation(anim_igla_sides);
		DelAnimation(anim_igla_gimbal);
		if (CM_jett == 0) DelAnimation(anim_dock_probe);
	}
	else if (sep_status == 3 || sep_status == 4) {
		DelMesh(mesh[2]);
		if (CM_jett == 0) DelDock(hDock);
		ClearBeacons();
		DelLightEmitter(spotlight1);
		DelLightEmitter(spotlight2);
		DelAttachment(attach1);
		DelAnimation(anim_igla_sides);
		DelAnimation(anim_ion);
		if (CM_jett == 0) DelAnimation(anim_dock_probe);
		DelMesh(mesh[0]);
		DelMesh(mesh[3]);
		DelMesh(mesh[4]);
		DelMesh(mesh[5]);
		DelMesh(mesh[6]);
		DelMesh(mesh[7]);
		DelMesh(mesh[8]);
		DelMesh(mesh[9]);
		DelMesh(mesh[10]);
		DelMesh(mesh[11]);
		DelMesh(mesh[12]);
		ShiftCG(_V(0, 0, -0.5));
		SA_BB_status = 1;
		ReconfigureSA();
		if (skin_status == 2) skin_status = 1;
	}
	else if (sep_status == 5 || sep_status == 6) {
		DelMesh(mesh[2]);
		if (CM_jett == 0) DelDock(hDock);
		ClearBeacons();
		DelLightEmitter(spotlight1);
		DelLightEmitter(spotlight2);
		DelAttachment(attach1);
		DelAnimation(anim_igla_sides);
		DelAnimation(anim_ion);
		if (CM_jett == 0) DelAnimation(anim_dock_probe);
		DelMesh(mesh[0]);
		DelMesh(mesh[3]);
		DelMesh(mesh[4]);
		DelMesh(mesh[5]);
		DelMesh(mesh[6]);
		DelMesh(mesh[7]);
		DelMesh(mesh[8]);
		DelMesh(mesh[9]);
		DelMesh(mesh[10]);
		DelMesh(mesh[11]);
		DelMesh(mesh[12]);
		ReconfigureSA();
		DelMesh(mesh[13]);
		SetEmptyMass(GetEmptyMass() - 500);
		if (skin_status == 2) skin_status = 1;
	}
	else if (hDock != NULL) {
		hTarget = oapiGetVesselByName(targetName);
		if (hTarget != NULL)target_Vessel = (Salyut4*)oapiGetVesselInterface(hTarget);

		OBJHANDLE hMother = GetDockStatus(hDock);
		if (hMother != NULL && dock_probe_status == 0) {
			hooks_status = 2;
			hooks_proc = 1;
		}

		// Set current docking animation state
		if (hMother == NULL)
		{
			VECTOR3 new_dock_pos = S7K_DOCK_POS;
			new_dock_pos.z = new_dock_pos.z + (dock_probe_proc * new_dock_factor) + DOCKOFFSET;
			SetDockParams(hDock, new_dock_pos, _V(0, 0, 1), _V(0, 1, 0));
		}
		else if (dock_probe_status > 0 && dock_probe_proc > 0.18)
		{
			VESSEL* v = oapiGetVesselInterface(hMother);
			int nDock;
			nDock = v->DockCount();
			for (DWORD j = 0; j < nDock; j++)
			{
				DOCKHANDLE hMotherDock = v->GetDockHandle(j);
				OBJHANDLE hChild = v->GetDockStatus(hMotherDock);
				if (hChild == GetHandle())
				{
					VECTOR3 mtr_am;
					v->GetAngularVel(mtr_am);
					Undock(0);
					VECTOR3 new_dock_pos = S7K_DOCK_POS;
					new_dock_pos.z = new_dock_pos.z + (dock_probe_proc * new_dock_factor) + DOCKOFFSET;
					SetDockParams(hDock, new_dock_pos, _V(0, 0, 1), _V(0, 1, 0));
					Dock(hMother, 0, j, 2);
					v->SetAngularVel(mtr_am);
					break;
				}
			}

		} 
	}
	// load dynamic texture for VC updating
	dyn_tex_first = oapiLoadTexture("Sirius_7k\\Sirius7k_Dynamic_Displays.dds", true);
	dyn_tex_second = oapiLoadTexture("Sirius_7k\\Sirius7k_Dynamic_Displays_ELS.dds", true);
	dyn_tex_KSU_unlit = oapiLoadTexture("Sirius_7k\\Sirius7k_Dynamic_Displays_KSU.dds", true);
	dyn_tex_KSU_lit = oapiLoadTexture("Sirius_7k\\Sirius7k_Dynamic_Displays_KSU_lit.dds", true);
	dyn_tex_IKP = oapiLoadTexture("Sirius_7k\\Sirius7k_IKP.dds", true);

	// load starting KSU status
	KSUl_cyl_proc = (double)KSUl_curr_col * 0.0625;
	KSUp_cyl_proc = (double)KSUp_curr_col * 0.0625;

	SetAnimation(anim_KSUl_sysbutt[KSUl_curr_col], 1.0);
	SetAnimation(anim_KSUp_sysbutt[KSUp_curr_col], 1.0);

	SetAnimation(anim_KSUl_cyl, KSUl_cyl_proc);
	SetAnimation(anim_KSUp_cyl, KSUp_cyl_proc);

	// set current selected KSU button state
	if (KSU_left_enable == 0 && KSU_right_enable == 0) SetAnimation(anim_KSU_off_button, 1.0);
	else if (KSU_left_enable == 1 && KSU_right_enable == 0) SetAnimation(anim_KSU_left_button, 1.0);
	else if (KSU_left_enable == 0 && KSU_right_enable == 1) SetAnimation(anim_KSU_right_button, 1.0);
	else SetAnimation(anim_KSU_both_button, 1.0);

	// globe initial set
	globe_o_speed = 1.001 / (ink_period * 60);
	SetAnimation(anim_globe_o, globe_o_proc);

	// correct globe e animation axis with rotation matrix around x
	double angGlobeO = 2 * PI * globe_o_proc;
	globe_e_axis.x = GLOBE_E_AXIS_INIT.x;
	globe_e_axis.y = GLOBE_E_AXIS_INIT.y * cos(angGlobeO) - GLOBE_E_AXIS_INIT.z * sin(angGlobeO);
	globe_e_axis.z = GLOBE_E_AXIS_INIT.y * sin(angGlobeO) + GLOBE_E_AXIS_INIT.z * cos(angGlobeO);
	globe_e->axis = globe_e_axis;

	SetAnimation(anim_globe_e, globe_e_proc);

	ugol_pos = round(ugol_pos_proc * 360);
	if (ugol_pos == 360) ugol_pos = 0;

	if (ink_select_status == 0) SetAnimation(anim_ink_select_knb, 0.0);
	else if (ink_select_status == 1) SetAnimation(anim_ink_select_knb, 0.5);
	else SetAnimation(anim_ink_select_knb, 1.0);

	SetAnimation(anim_INT_knob, 1);

	// Force one single battery on - TEMP
	if (BB_status && RB_status) RB_status = 0;

	if (RB_status) {
		SetAnimation(anim_prepRB_button, 1.0);
		SetAnimation(anim_noprepRB_button, 0.0);
	}
	else {
		SetAnimation(anim_prepRB_button, 0.0);
		SetAnimation(anim_noprepRB_button, 0.0);
	}

	//initialise connection to graphics client
	gcInitialize();
	// custom cameras for vzor if connection established
	if (gcEnabled() && sep_status < 3) {
		hSurfVzor = oapiCreateSurfaceEx(1024, 1024, OAPISURFACE_RENDER3D | OAPISURFACE_TEXTURE | OAPISURFACE_RENDERTARGET | OAPISURFACE_NOMIPMAPS );	//surface for nadir vzor cam
		hSurfVzor2 = oapiCreateSurfaceEx(1024, 1024, OAPISURFACE_RENDER3D | OAPISURFACE_TEXTURE | OAPISURFACE_RENDERTARGET | OAPISURFACE_NOMIPMAPS );	//surface for forward vzor cam
		hSurfVzor3 = oapiCreateSurfaceEx(1024, 1024, OAPISURFACE_RENDER3D | OAPISURFACE_TEXTURE | OAPISURFACE_RENDERTARGET | OAPISURFACE_NOMIPMAPS);	//surface for peripheral vzor cam
		oapiClearSurface(hSurfVzor);
		oapiClearSurface(hSurfVzor2);
		oapiClearSurface(hSurfVzor3);
		//setup cameras position and nadir direction 
		VECTOR3 campos = { 0.280216, -1.47587, 0.5 + mesh_shift_z };
		VECTOR3 camdir = { 0, -0.994522, 0.104528 };
		VECTOR3 camup = { 0, 0.104528, 0.994522 };
		hCam_vzor_nad = gcSetupCustomCamera(hCam_vzor_nad, GetHandle(), campos, camdir, camup, 7.5 * RAD, hSurfVzor);
		hCam_vzor_periph = gcSetupCustomCamera(hCam_vzor_periph, GetHandle(), campos, camdir, camup, 77 * RAD, hSurfVzor3);
		//setup forward direction
		camdir = { 0, 0, 1 };
		camup = { 0, 1, 0 };
		hCam_vzor_fwd = gcSetupCustomCamera(hCam_vzor_fwd, GetHandle(), campos, camdir, camup, 7.5 * RAD, hSurfVzor2);

		//enable camera according to current vzor status
		if (vzor_status == 3) {
			gcCustomCameraOnOff(hCam_vzor_nad, true);
			gcCustomCameraOnOff(hCam_vzor_fwd, false);
		}
		else if (vzor_status == 2) {
			gcCustomCameraOnOff(hCam_vzor_fwd, true);
			gcCustomCameraOnOff(hCam_vzor_nad, false);
		}
		gcCustomCameraOnOff(hCam_vzor_periph, true);
		//get texture handles for the central screen and peripheral screens textures to blit to
		vzTex = oapiGetTextureHandle(VC_Vzor_mesh, 2);
		vzTex2 = oapiGetTextureHandle(VC_Vzor_mesh, 3);

		//turn it off if landed
		//if you turn it on and see through a fairing, your problem mate, can't fix
		VESSELSTATUS2 status;
		GetStatusEx(&status);
		if (status.status) {
			vzor_c_on = 2;
			vzor_p_on = 2;
			gcCustomCameraOnOff(hCam_vzor_periph, false);
			gcCustomCameraOnOff(hCam_vzor_fwd, false);
			gcCustomCameraOnOff(hCam_vzor_nad, false);
		}
		
	}

	if (vzor_status == 3 && sep_status < 3) {				//switch to vzor nadir
		SetCameraDefaultDirection(_V(0, -0.994522, 0.104528));
		oapiCameraSetCockpitDir(0, 0, false);
		vzor_timer = 1;
	}
	else if (vzor_status == 2 && sep_status < 3) {		//switch to vzor fwd
		SetCameraDefaultDirection(_V(0, 0, 1));
		oapiCameraSetCockpitDir(0, 0, false);
		vzor_timer = 0;
	}

	// Sound setup:
	// Create sound engine instance for this vessel
	m_pXRSound = XRSound::CreateInstance(this); 

	// Disable some default sounds
	m_pXRSound->SetDefaultSoundEnabled(XRSound::CabinAmbienceGroup, false);
	m_pXRSound->SetDefaultSoundEnabled(XRSound::MachCalloutsGroup, false);
	m_pXRSound->SetDefaultSoundEnabled(XRSound::AltitudeCalloutsGroup, false);
	m_pXRSound->SetDefaultSoundEnabled(XRSound::DockingDistanceCalloutsGroup, false);

	// VC sounds 
	m_pXRSound->LoadWav(ClockTick, "Sound\\Soyuz_7k\\clock_halfsec_tick.mp3", XRSound::InternalOnly);
	m_pXRSound->LoadWav(ClockSwitch, "Sound\\Soyuz_7k\\clock_switch.mp3", XRSound::InternalOnly);
	m_pXRSound->LoadWav(ChronoButton, "Sound\\Soyuz_7k\\chrono_button.mp3", XRSound::InternalOnly);
	m_pXRSound->LoadWav(AlarmSound, "Sound\\Soyuz_7k\\7k_alarm.mp3", XRSound::InternalOnly);
	m_pXRSound->LoadWav(ButtonIn, "Sound\\Soyuz_7k\\button_in.mp3", XRSound::InternalOnly);
	m_pXRSound->LoadWav(ButtonOut, "Sound\\Soyuz_7k\\button_out.mp3", XRSound::InternalOnly);
	m_pXRSound->LoadWav(ValveBlow, "XRSound\\Default\\Thump.wav", XRSound::InternalOnly);
}

bool Soyuz7k_T::METBlt(int which, SURFHANDLE surf) {
	// handles processing for updating Mission Elapsed Time display

	if (which == 0 && MET_update > 0) {
		oapiBltPanelAreaBackground(MET_MIN_DISP, surf);
		int div = MET_minute / 10;
		int mod = MET_minute % 10;

		if (div == 0) {
			oapiBlt(surf, dyn_tex_first, 8, 8, 135, 327, 17, 21);
		}
		else {
			oapiBlt(surf, dyn_tex_first, 8, 8, (20 + 29 * (div - 1)), 288, 17, 21);
		}

		if (mod == 0) {
			oapiBlt(surf, dyn_tex_first, 42, 8, 135, 327, 17, 21);
		}
		else if (mod == 1) {
			oapiBlt(surf, dyn_tex_first, 42, 8, 117, 488, 17, 21);
		}
		else if (mod < 6) {
			oapiBlt(surf, dyn_tex_first, 42, 8, (49 + 29 * (mod - 2)), 288, 17, 21);
		}
		else {
			oapiBlt(surf, dyn_tex_first, 42, 8, (20 + 28 * (mod - 6)), 327, 17, 21);
		}
		MET_update--;
		return true;
	}
	else if (which == 1 && MET_update > 0) {
		oapiBltPanelAreaBackground(MET_HOUR_DISP, surf);
		if (MET_hour == 0) {
			oapiBlt(surf, dyn_tex_first, 19, 7, 74, 488, 28, 21);
		}
		else if (MET_hour < 6) {
			oapiBlt(surf, dyn_tex_first, 19, 7, (14 + 29 * (MET_hour - 1)), 288, 28, 21);
		}
		else if (MET_hour < 10) {
			oapiBlt(surf, dyn_tex_first, 19, 7, (15 + 28 * (MET_hour - 6)), 327, 28, 21);
		}
		else if (MET_hour < 14) {
			oapiBlt(surf, dyn_tex_first, 19, 7, (8 + 34 * (MET_hour - 10)), 366, 28, 21);
		}
		else if (MET_hour < 18) {
			oapiBlt(surf, dyn_tex_first, 19, 7, (8 + 34 * (MET_hour - 14)), 405, 28, 21);
		}
		else if (MET_hour < 22) {
			oapiBlt(surf, dyn_tex_first, 19, 7, (8 + 34 * (MET_hour - 18)), 445, 28, 21);
		}
		else {
			oapiBlt(surf, dyn_tex_first, 19, 7, (7 + 34 * (MET_hour - 22)), 488, 28, 21);
		}
		MET_update--;
		return true;
	}
	else if (MET_update > 0) {
		oapiBltPanelAreaBackground(MET_DAY_DISP, surf);
		int div = MET_day / 10;
		int mod = MET_day % 10;

		if (div == 0) {
			oapiBlt(surf, dyn_tex_first, 8, 8, 135, 327, 17, 21);
		}
		else if (div < 6) {
			oapiBlt(surf, dyn_tex_first, 8, 8, (20 + 29 * (div - 1)), 288, 17, 21);
		}
		else {
			oapiBlt(surf, dyn_tex_first, 8, 8, (20 + 28 * (mod - 6)), 327, 17, 21);
		}

		if (mod == 0) {
			oapiBlt(surf, dyn_tex_first, 42, 8, 135, 327, 17, 21);
		}
		else if (mod == 1) {
			oapiBlt(surf, dyn_tex_first, 42, 8, 117, 488, 17, 21);
		}
		else if (mod < 6) {
			oapiBlt(surf, dyn_tex_first, 42, 8, (49 + 29 * (mod - 2)), 288, 17, 21);
		}
		else {
			oapiBlt(surf, dyn_tex_first, 42, 8, (20 + 28 * (mod - 6)), 327, 17, 21);
		}
		MET_update--;
		return true;
	}
	return false;
}

bool Soyuz7k_T::ELSBlt(int id, SURFHANDLE surf) {
	// handles processing for ELS updating
	
	if (id == ELS_1) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 13, 295, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 13, 4, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_2) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 4, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && undock_signal > 0) || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 295, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && undock_signal == 0) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 4, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_3) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 295, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 4, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_4) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 295, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 4, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_5) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 13, 101, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && avaria != 0) || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 13, 392, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && avaria == 0) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 13, 101, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_6) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 392, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 101, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_7) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 392, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 101, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_8) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 101, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && thermals_off) || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 392, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && !thermals_off) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 101, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_9) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 13, 489, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 13, 198, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_10) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 198, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && RB_status) || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 489, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && !RB_status) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 198, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_11) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 489, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 198, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_12) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 489, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 198, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_13) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 854, 4, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && ballistic) || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 854, 295, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && !ballistic) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 854, 4, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_14) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1063, 4, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && ((BB_status && BB_voltage <= 24.3) || (RB_status && RB_voltage <= 24.3) || (SA_BB_status && SA_BB_voltage <= 24.3))) || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1063, 295, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && ((BB_status && BB_voltage > 24.3) || (RB_status && RB_voltage > 24.3) || (SA_BB_status && SA_BB_voltage > 24.3))) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1063, 4, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_15) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1272, 295, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1272, 4, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_16) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1481, 295, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1481, 4, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_17) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 854, 101, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && (KSU_left_enable == 1 || KSU_right_enable == 1)) || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 854, 392, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && KSU_left_enable == 0 && KSU_right_enable == 0) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 854, 101, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_18) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1063, 295, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1063, 101, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_19) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1272, 101, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && main_chute_status != 0) && sep_status > 2 || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1272, 392, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && main_chute_status == 0) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1 && sep_status > 2)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1272, 101, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_20) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1481, 101, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && altitude <= 15) && sep_status > 2 || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1481, 392, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && (altitude > 15 || sep_status < 3)) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1 && sep_status > 2)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1481, 101, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_21) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 854, 489, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 854, 198, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_22) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1063, 489, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1063, 198, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_23) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1272, 489, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1272, 198, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_24) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1481, 198, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && altitude <= 5500) && sep_status > 2 || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1481, 489, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && (altitude > 5500 || sep_status < 3)) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1 && sep_status > 2)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1481, 198, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_25) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 13, 880, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 13, 589, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_26) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 880, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 589, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_27) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 880, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 589, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_28) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 589, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (hAttached == NULL && last_ELS_state[id - ELS_1] == 0 && ((ion_err * DEG <= 3 && orb_ori_dir == 0) || (ion_err * DEG >= 177 && orb_ori_dir == 1))) || (hAttached == NULL && last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 880, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && ((ion_err * DEG > 3 && orb_ori_dir == 0) || (ion_err * DEG < 177 && orb_ori_dir == 1))) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 589, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_29) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 13, 977, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 13, 686, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_30) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 977, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 686, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_31) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 686, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && ((rcs_status == 2 && thrust_rot > 0.0))) || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {	// if rotation DO or DO translation assist is being used
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 977, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && thrust_rot == 0.0 ) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1)) {	// if no rotation or translation is being performed, regardless of DO or DPO
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 686, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_32) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 686, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && inertial_status != 0) || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 977, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && inertial_status == 0) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 686, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_33) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 13, 1074, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 13, 783, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_34) {
		if ((ELS_check || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) && !no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 1074, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 222, 783, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_35) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 783, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && ((rcs_status == 0 && thrust_rot > 0.0) || thrust_trans_dpo > 0.0)) || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {	// if rotation DPO or translation is being used
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 1074, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && thrust_rot == 0 && thrust_trans_dpo == 0) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1)) {	// if no rotation or translation is being performed in DPO
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 431, 783, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == ELS_36) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 783, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if (ELS_check || (last_ELS_state[id - ELS_1] == 0 && thrust_main > 0) || (last_ELS_state[id - ELS_1] == 1 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 1074, 205, 93);
			last_ELS_state[id - ELS_1] = 1;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
		else if ((last_ELS_state[id - ELS_1] == 1 && thrust_main == 0) || (last_ELS_state[id - ELS_1] == 0 && first_ELS_load[id - ELS_1] == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 640, 783, 205, 93);
			last_ELS_state[id - ELS_1] = 0;
			first_ELS_load[id - ELS_1] = 0;
			return true;
		}
	}
	else if (id == BTSI_DKD) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 854, 589, 205, 93);
			last_BTSI_DKD_state = 0;
			first_BTSI_DKD_load = 0;
			return true;
		}
		if ((last_BTSI_DKD_state == 0 && thrust_main > 0 && engine_status != 0) || (last_BTSI_DKD_state == 1 && first_BTSI_DKD_load == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 854, 686, 205, 93);
			last_BTSI_DKD_state = 1;
			first_BTSI_DKD_load = 0;
			return true;
		}
		else if ((last_BTSI_DKD_state == 1 && thrust_main == 0) || (last_BTSI_DKD_state == 0 && first_BTSI_DKD_load == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 854, 589, 205, 93);
			last_BTSI_DKD_state = 0;
			first_BTSI_DKD_load = 0;
			return true;
		}
	}
	else if (id == INK_MP) {
		if (no_power) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1063, 589, 205, 93);
			last_MP_state = 0;
			first_MP_load = 0;
			return true;
		}
		else if ((last_MP_state == 0 && ink_select_status == 2) || (last_MP_state == 1 && first_MP_load == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1063, 686, 205, 93);
			last_MP_state = 1;
			first_MP_load = 0;
			return true;
		}
		else if ((last_MP_state == 1 && ink_select_status != 2) || (last_MP_state == 0 && first_MP_load == 1)) {
			oapiBltPanelAreaBackground(id, surf);
			oapiBlt(surf, dyn_tex_second, 0, 0, 1063, 589, 205, 93);
			last_MP_state = 0;
			first_MP_load = 0;
			return true;
		}
	}
	return false;

}

bool Soyuz7k_T::KSUBlt(SURFHANDLE surf) { 

	if (no_power || !KSU_update) {
		return true;
	}

	oapiBltPanelAreaBackground(KSUl_cyl, surf);

	//V-1
	if (AKSP_status) {
		oapiBlt(surf, dyn_tex_KSU_lit, 256, 0, 256, 0, 127, 95);
	}

	//V-25
	if (sep_status == 2 || sep_status == 4) {
		oapiBlt(surf, dyn_tex_KSU_lit, 256, 1236, 256, 1236, 127, 95);
	}
	
	//V-26
	if (SUS_status > 1) {
		oapiBlt(surf, dyn_tex_KSU_lit, 256, 1339, 256, 1339, 127, 95);
	}

	//I-1
	if (rcs_status == 0) {
		oapiBlt(surf, dyn_tex_KSU_lit, 896, 0, 896, 0, 127, 95);
	}

	//I-3
	if (rcs_status == 2 || rcs_status == 4) {
		oapiBlt(surf, dyn_tex_KSU_lit, 896, 103, 896, 103, 127, 95);
	}

	//I-23
	if (thermals_off) {
		oapiBlt(surf, dyn_tex_KSU_lit, 896, 1133, 896, 1133, 127, 95);
	}

	//K-7
	if (pulsed_status == 1) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1024, 309, 1024, 309, 127, 95);
	}
	//K-5
	else if (pulsed_status == 0) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1024, 206, 1024, 206, 127, 95);
	}

	//K-3
	if (BDUS_status == BDUS_ON) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1024, 103, 1024, 103, 127, 95);
	}

	//L-1
	if (manual_settings) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1152, 0, 1152, 0, 127, 95);
	}

	//P-15
	if (orb_ori_dir == 0) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1536, 721, 1536, 721, 127, 95);
		oapiBlt(surf, dyn_tex_KSU_unlit, 1536, 824, 1536, 824, 127, 95);
	}
	//P-17
	else {
		oapiBlt(surf, dyn_tex_KSU_unlit, 1536, 721, 1536, 721, 127, 95);
		oapiBlt(surf, dyn_tex_KSU_lit, 1536, 824, 1536, 824, 127, 95);
	}

	//K-9
	if (ori_ap_status != 0) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1024, 412, 1024, 412, 127, 95);
	}

	//M-3
	if (PVU_cmd_off) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1280, 103, 1280, 103, 127, 95);
	}

	//M-7
	if (program_on == 2) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1280, 309, 1280, 309, 127, 95);
	}

	//M-11
	if (program_on == 1) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1280, 515, 1280, 515, 127, 95);
	}

	//M-15
	if (sun_ap_status != 0) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1280, 721, 1280, 721, 127, 95);
	}

	//I-13
	if (vzor_status == 3) {
		oapiBlt(surf, dyn_tex_KSU_lit, 896, 618, 896, 618, 127, 95);
	}
	//ZH-17
	else if (vzor_status == 2) {
		oapiBlt(surf, dyn_tex_KSU_lit, 768, 824, 768, 824, 127, 95);
	}

	//K-1
	if (modes_off != 0) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1024, 0, 1024, 0, 127, 95);
	}

	//N-13
	if (engine_status == 2) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1408, 618, 1408, 618, 127, 95);
	}

	//ZH-1
	if (dock_probe_status == 2 && ssvp_on) {
		oapiBlt(surf, dyn_tex_KSU_lit, 768, 0, 768, 0, 127, 95);
	}
	//S-5
	else if (dock_probe_status == 0 && ssvp_on) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1792, 206, 1792, 206, 127, 95);
	}

	//ZH-3
	if (latches_status == 2 && ssvp_on) {
		oapiBlt(surf, dyn_tex_KSU_lit, 768, 103, 768, 103, 127, 95);
	}
	//S-3
	else if (latches_status == 0 && ssvp_on) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1792, 103, 1792, 103, 127, 95);
	}

	//ZH-19
	if (hooks_status == 0 && ssvp_on) {
		oapiBlt(surf, dyn_tex_KSU_lit, 768, 927, 768, 927, 127, 95);
	}
	//S-7
	else if (hooks_status == 2 && ssvp_on) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1792, 309, 1792, 309, 127, 95);
	}

	//ZH-25
	if (dock_probe_status == 2 && latches_status == 2 && hooks_status == 0 && ssvp_on) {
		oapiBlt(surf, dyn_tex_KSU_lit, 768, 1236, 768, 1236, 127, 95);
	}

	//S-1
	if (ssvp_on == 1) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1792, 0, 1792, 0, 127, 95);
	}

	//ZH-26
	if (dock_probe_status == 2 && latches_status == 2 && hMother && ssvp_on) {
		oapiBlt(surf, dyn_tex_KSU_lit, 768, 1339, 768, 1339, 127, 95);
	}

	//ZH-27
	if (dock_probe_status == 0 && hooks_status == 2 && hMother && ssvp_on) {
		oapiBlt(surf, dyn_tex_KSU_lit, 768, 1442, 768, 1442, 127, 95);
	}

	//P-5
	if (BDUS_integr) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1536, 206, 1536, 206, 127, 95);
	}

	//K-19
	if (accel_z_status) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1024, 927, 1024, 927, 127, 95);
	}

	//ZH-5
	if (man_appr) {
		oapiBlt(surf, dyn_tex_KSU_lit, 768, 206, 768, 206, 127, 95);
	}

	//ZH-7
	if (igla_status > 0) {
		oapiBlt(surf, dyn_tex_KSU_lit, 768, 309, 768, 309, 127, 95);
	}

	//R-13
	if (combined_power) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1664, 618, 1664, 618, 127, 95);
	}

	//R-15
	if (recharge) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1664, 721, 1664, 721, 127, 95);
	}

	//N-1
	if (backup_DO) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1408, 0, 1408, 0, 127, 95);
	}

	//N-3
	/*if (fuel_iso) {
		oapiBlt(surf, dyn_tex_KSU_lit, 1408, 103, 1408, 103, 127, 95);
	}*/

	KSU_update = 0;
	return true;
}

void Soyuz7k_T::BTSIBlt(SURFHANDLE surf, int counter) {

	int Dv_hundreds, Dv_tens, Dv_unit, Dv_frac, Dv_temp;

	if (counter == 4) {
		if (btsi_5_flag != 0) {
			// Split accelerometer delta-v into individual digits
			Dv_temp = round(btsi_5_counter * 10);
			Dv_hundreds = Dv_temp / 1000;
			Dv_tens = Dv_temp / 100;
			Dv_tens = Dv_tens % 10;
			Dv_unit = Dv_temp / 10;
			Dv_unit = Dv_unit % 10;
			Dv_frac = Dv_temp % 10;

			// Accelerometer Delta-V counter blitting
			oapiBltPanelAreaBackground(BTSI_5, surf);
			oapiBlt(surf, dyn_tex_first, 1, 1, (200 + 29 * (Dv_hundreds)), 370, 24, 34);
			oapiBlt(surf, dyn_tex_first, 53, 1, (200 + 29 * (Dv_tens)), 370, 24, 34);
			oapiBlt(surf, dyn_tex_first, 105, 1, (200 + 29 * (Dv_unit)), 370, 24, 34);
			oapiBlt(surf, dyn_tex_first, 157, 1, (200 + 29 * (Dv_frac)), 370, 24, 34);
			if (btsi_5_flag == 2) btsi_5_flag = 0;

		}
	}
	else if (counter == 5) {
		// Split delta-v into individual digits
		Dv_temp = round(deltav * 10);
		Dv_hundreds = Dv_temp / 1000;
		Dv_tens = Dv_temp / 100;
		Dv_tens = Dv_tens % 10;
		Dv_unit = Dv_temp / 10;
		Dv_unit = Dv_unit % 10;
		Dv_frac = Dv_temp % 10;

		// Delta-V counter blitting
		oapiBlt(surf, dyn_tex_first, 1, 1, (200 + 29 * (Dv_hundreds)), 370, 24, 34);
		oapiBlt(surf, dyn_tex_first, 53, 1, (200 + 29 * (Dv_tens)), 370, 24, 34);
		oapiBlt(surf, dyn_tex_first, 105, 1, (200 + 29 * (Dv_unit)), 370, 24, 34);
		oapiBlt(surf, dyn_tex_first, 157, 1, (200 + 29 * (Dv_frac)), 370, 24, 34);
	}
}

bool Soyuz7k_T::INKBlt(SURFHANDLE surf, int flag) {

	int P_tens, P_unit, P_frac1, P_frac2, P_temp;

	if (flag == 0 && INK_update) {
		P_temp = round(ink_period * 100);
		P_tens = P_temp / 1000;
		P_unit = P_temp / 100;
		P_unit = P_unit % 10;
		P_frac1 = P_temp / 10;
		P_frac1 = P_frac1 % 10;
		P_frac2 = P_temp % 10;

		// Period counter blitting
		oapiBltPanelAreaBackground(INK_PERIOD, surf);
		oapiBlt(surf, dyn_tex_first, 1, 1, (200 + 29 * (P_tens)), 370, 24, 34);
		oapiBlt(surf, dyn_tex_first, 27, 1, (200 + 29 * (P_unit)), 370, 24, 34);
		oapiBlt(surf, dyn_tex_first, 79, 1, (200 + 29 * (P_frac1)), 410, 24, 34);
		oapiBlt(surf, dyn_tex_first, 131, 1, (200 + 29 * (P_frac2)), 410, 24, 34);
		INK_update = 0;
		return true;
	}
	else if (flag == 1 && ink_vit_flag != 0) {
		P_temp = round(orbit_count * 10);
		P_tens = P_temp / 1000;
		P_unit = P_temp / 100;
		P_unit = P_unit % 10;
		P_frac1 = P_temp / 10;
		P_frac1 = P_frac1 % 10;
		P_frac2 = P_temp % 10;

		// Orbit counter blitting
		oapiBltPanelAreaBackground(INK_VIT, surf);
		oapiBlt(surf, dyn_tex_first, 1, 1, (200 + 29 * (P_tens)), 370, 24, 34);
		oapiBlt(surf, dyn_tex_first, 49, 1, (200 + 29 * (P_unit)), 370, 24, 34);
		oapiBlt(surf, dyn_tex_first, 97, 1, (200 + 29 * (P_frac1)), 370, 24, 34);
		oapiBlt(surf, dyn_tex_first, 145, 1, (200 + 29 * (P_frac2)), 370, 24, 34);
		if (ink_vit_flag == 2) ink_vit_flag = 0;
		return true;
	}
	else if (flag == 2 && ink_ugol_flag != 0) {
		P_unit = ugol_pos / 100;
		P_frac1 = ugol_pos / 10;
		P_frac1 = P_frac1 % 10;
		P_frac2 = ugol_pos % 10;

		// Ugol pos. counter blitting
		oapiBltPanelAreaBackground(INK_UGOL, surf);
		oapiBlt(surf, dyn_tex_first, 1, 1, (200 + 29 * (P_unit)), 370, 24, 34);
		oapiBlt(surf, dyn_tex_first, 49, 1, (200 + 29 * (P_frac1)), 370, 24, 34);
		oapiBlt(surf, dyn_tex_first, 97, 1, (200 + 29 * (P_frac2)), 370, 24, 34);
		if (ink_ugol_flag == 2) ink_ugol_flag = 0;
		return true;
	}
	
	return false;
}

bool Soyuz7k_T::IKPBlt(SURFHANDLE surf) {

	if (no_power) {
		return false;
	}
	else if (IKP_update) {
		
		oapiBltPanelAreaBackground(IKP_PANEL, surf);

		if (IKP_check) {
			// blit whole screen
			oapiBlt(surf, dyn_tex_IKP, 0, 0, 701, 14, 675, 995);
			// blit ticker according to accelerated timer
			oapiBlt(surf, dyn_tex_IKP, 266, (73 + 10 * PVU_minute), 1401, 28, 47, 10);
			IKP_update = 0;
			return true;
		}
		else {
			// blit time scale
			oapiBlt(surf, dyn_tex_IKP, 196, 33, 897, 47, 85, 924);

			if (program_on != 0) {
				// if program on, blit common "PROGRAMMA" indicator
				oapiBlt(surf, dyn_tex_IKP, 55, 42, 756, 56, 117, 22);
				
				// Program 1: avt manevr
				if (program_on == 1) {
					// blit all manevr signals
					oapiBlt(surf, dyn_tex_IKP, 316, 504, 1386, 652, 126, 345);							// scale-label connectors
					oapiBlt(surf, dyn_tex_IKP, 143, 776, 844, 790, 44, 34);								// SKDU time scale
					oapiBlt(surf, dyn_tex_IKP, 21, 200, 722, 214, 129, 24);								// Konets na 107m
					oapiBlt(surf, dyn_tex_IKP, 25, 162, 726, 176, 70, 24);								// Manevr
					oapiBlt(surf, dyn_tex_IKP, 448, 487, 1149, 501, 74, 86);							// DUS / Orient / Inerts 1
					if (PVU_cmd_seq < 1) oapiBlt(surf, dyn_tex_IKP, 617, 487, 1318, 501, 32, 20);		// DUS indicator
					if (PVU_cmd_seq < 2) oapiBlt(surf, dyn_tex_IKP, 617, 518, 1318, 532, 32, 20);		// Orient indicator
					if (PVU_cmd_seq < 3) oapiBlt(surf, dyn_tex_IKP, 617, 549, 1318, 563, 32, 20);		// Inerts 1 indicator
					oapiBlt(surf, dyn_tex_IKP, 448, 580, 1149, 594, 123, 55);							// Nadduv SKDU / Inerts 2
					if (PVU_cmd_seq < 4) oapiBlt(surf, dyn_tex_IKP, 617, 580, 1318, 594, 32, 20);		// Nadduv SKDU indicator
					if (PVU_cmd_seq < 5) oapiBlt(surf, dyn_tex_IKP, 617, 611, 1318, 625, 32, 20);		// Inerts 2 indicator
					oapiBlt(surf, dyn_tex_IKP, 448, 735, 1149, 749, 67, 20);							// Integr
					if (PVU_cmd_seq < 6) oapiBlt(surf, dyn_tex_IKP, 617, 735, 1318, 749, 32, 20);		// Integr indicator
					oapiBlt(surf, dyn_tex_IKP, 448, 828, 1149, 842, 113, 24);							// Inerts orient (razarretir)
					if (PVU_cmd_seq < 7) oapiBlt(surf, dyn_tex_IKP, 617, 828, 1318, 842, 32, 20);		// Inerts orient (razarretir) indicator
					if (PVU_cmd_seq == 8) oapiBlt(surf, dyn_tex_IKP, 22, 752, 723, 766, 118, 24);		// Rabota SKDU
				}
				
				// Program 2: spusk 2
				else if (program_on == 2) {
					// blit all spusk 2 signals
					oapiBlt(surf, dyn_tex_IKP, 318, 406, 1383, 99, 124, 474);							// scale-label connectors
					oapiBlt(surf, dyn_tex_IKP, 143, 676, 844, 690, 44, 34);								// SKDU time scale
					oapiBlt(surf, dyn_tex_IKP, 25, 507, 726, 521, 121, 24);								// Konets na 97m
					oapiBlt(surf, dyn_tex_IKP, 62, 458, 763, 472, 70, 20);								// Spusk 2
					oapiBlt(surf, dyn_tex_IKP, 448, 394, 1149, 408, 35, 24);							// DUS
					if (PVU_cmd_seq < 1) oapiBlt(surf, dyn_tex_IKP, 617, 394, 1318, 408, 32, 20);		// DUS indicator
					oapiBlt(surf, dyn_tex_IKP, 448, 425, 1149, 439, 74, 55);							// Orient / Inerts 1
					if (PVU_cmd_seq < 2) oapiBlt(surf, dyn_tex_IKP, 617, 425, 1318, 439, 32, 20);		// Orient indicator
					if (PVU_cmd_seq < 3) oapiBlt(surf, dyn_tex_IKP, 617, 456, 1318, 470, 32, 20);		// Inerts 1 indicator
					oapiBlt(surf, dyn_tex_IKP, 448, 549, 1149, 563, 61, 24);							// Inerts
					oapiBlt(surf, dyn_tex_IKP, 527, 549, 1228, 563, 11, 20);							// 2
					if (PVU_cmd_seq < 4) oapiBlt(surf, dyn_tex_IKP, 617, 549, 1318, 563, 32, 20);		// Inerts 2 indicator
					oapiBlt(surf, dyn_tex_IKP, 448, 580, 1149, 594, 123, 24);							// Nadduv SKDU
					if (PVU_cmd_seq < 5) oapiBlt(surf, dyn_tex_IKP, 617, 580, 1318, 594, 32, 20);		// Nadduv SKDU indicator
					oapiBlt(surf, dyn_tex_IKP, 448, 642, 1149, 656, 67, 20);							// Integr
					if (PVU_cmd_seq < 6) oapiBlt(surf, dyn_tex_IKP, 617, 642, 1318, 656, 32, 20);		// Integr indicator
					oapiBlt(surf, dyn_tex_IKP, 448, 673, 1149, 687, 113, 55);							// SUS / Inerts orient
					if (PVU_cmd_seq < 7) oapiBlt(surf, dyn_tex_IKP, 617, 673, 1318, 687, 32, 20);		// SUS indicator
					if (PVU_cmd_seq < 8) oapiBlt(surf, dyn_tex_IKP, 617, 704, 1318, 718, 32, 20);		// Inerts orient (razarretir) indicator
					if (PVU_cmd_seq == 9) oapiBlt(surf, dyn_tex_IKP, 22, 652, 723, 666, 118, 24);		// Rabota SKDU
					oapiBlt(surf, dyn_tex_IKP, 448, 766, 1149, 780, 78, 24);							// DUS SUS
					if (PVU_cmd_seq < 11) oapiBlt(surf, dyn_tex_IKP, 617, 766, 1318, 780, 32, 20);		// DUS SUS indicator
					oapiBlt(surf, dyn_tex_IKP, 448, 859, 1149, 873, 111, 24);							// Razdel
					if (PVU_cmd_seq < 12) oapiBlt(surf, dyn_tex_IKP, 617, 859, 1318, 873, 32, 20);		// Razdel indicator
				}
			}
			// blit ticker according to timer
			oapiBlt(surf, dyn_tex_IKP, 266, (73 + 10 * PVU_minute), 1401, 28, 47, 10);
			IKP_update = 0;
			return true;
		}
	}
	return false;
}

bool Soyuz7k_T::clbkLoadVC(int id) {

	bool ok = false;

	switch (id) {

	case 0:
		SetCameraOffset(_V(0, 0.2, 0.5));
		if (sep_status == 2) SetCameraOffset(_V(0, 0.2, 1.0));
		else if (sep_status > 2) SetCameraOffset(_V(0, 0.2, 0.4));
		SetCameraDefaultDirection(_V(0, -0.707, 0.707));
		oapiVCSetNeighbours(1, 2, -1, 3);
		ok = true;
		break;

	case 1:
		SetCameraOffset(_V(-0.45, 0.2, 0.8));
		if (sep_status == 2) SetCameraOffset(_V(-0.45, 0.2, 1.3));
		else if (sep_status > 2) SetCameraOffset(_V(-0.45, 0.2, 0.7));
		SetCameraDefaultDirection(_V(-0.239313, -0.714201, 0.657758));
		oapiVCSetNeighbours(-1, 0, -1, 4);
		ok = true;
		break;

	case 2:
		SetCameraOffset(_V(0.45, 0.2, 0.8));
		if (sep_status == 2) SetCameraOffset(_V(0.45, 0.2, 1.3));
		else if (sep_status > 2) SetCameraOffset(_V(0.45, 0.2, 0.7));
		SetCameraDefaultDirection(_V(0.239313, -0.714201, 0.657758));
		oapiVCSetNeighbours(0, -1, -1, -1);
		ok = true;
		break;

	case 3:
		SetCameraOffset(_V(0.278637, -0.305673, 0.87));
		if (sep_status == 2) SetCameraOffset(_V(0.278637, -0.305673, 1.37));
		else if (sep_status > 2) SetCameraOffset(_V(0.278637, -0.305673, 0.77));
		SetCameraDefaultDirection(_V(0, -0.993854, 0.110703));
		oapiVCSetNeighbours(-1, -1, 0, -1);
		ok = true;
		break;

	case 4:
		SetCameraOffset(_V(-0.34, -0.3, 1.08));
		if (sep_status == 2) SetCameraOffset(_V(-0.34, -0.3, 1.58));
		else if (sep_status > 2) SetCameraOffset(_V(-0.34, -0.3, 0.98));
		SetCameraDefaultDirection(_V(0, -0.976202, 0.216864));
		oapiVCSetNeighbours(-1, 0, 4, -1);
		ok = true;
		break;
	}

	SetCameraRotationRange(RAD * 120, RAD * 120, RAD * 70, RAD * 70);
	SetCameraShiftRange(_V(0, 0, 0.1), _V(-0.2, 0, 0), _V(0.2, 0, 0));
	oapiCameraSetAperture(22.5 * RAD);

	// register both Vzor screens for update with cameras
	if (gcEnabled()) {
		oapiVCRegisterArea(VZOR_CENTRAL, _R(0, 0, 1023, 1023), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, vzTex);
		oapiVCRegisterArea(VZOR_PERIPH, _R(0, 0, 1023, 1023), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, vzTex2);
	}

	// register Vzor controls
	oapiVCRegisterArea(VZOR_P_TOGGLE, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(VZOR_P_TOGGLE, _V(0.228643, -0.543856 - mesh_shift_y, 0.352767 + mesh_shift_z), 0.012);

	oapiVCRegisterArea(VZOR_C_TOGGLE, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(VZOR_C_TOGGLE, _V(0.380995, -0.557172 - mesh_shift_y, 0.319087 + mesh_shift_z), 0.012);

	// register clock on/off switch
	oapiVCRegisterArea(CLOCK_ON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(CLOCK_ON, _V(-0.214873, -0.579814 - mesh_shift_y, 0.28954 + mesh_shift_z), 0.005);

	// register crono toggle button
	oapiVCRegisterArea(CRONO_TOGGLE, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(CRONO_TOGGLE, _V(-0.193616, -0.576243 - mesh_shift_y, 0.293488 + mesh_shift_z), 0.005);

	// crono state digit
	// load dynamic textures for VC
	surf_dynamic_first = oapiGetTextureHandle(VC_panel_mesh, 2); 
	oapiVCRegisterArea(CRONO_DIGIT, _R(42, 70, 63, 97), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_first);
	
	// MET displays register
	oapiVCRegisterArea(MET_MIN_DISP, _R(29, 547, 96, 584), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_first);
	oapiVCRegisterArea(MET_HOUR_DISP, _R(29, 225, 96, 261), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_first);
	oapiVCRegisterArea(MET_DAY_DISP, _R(29, 612, 96, 649), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_first);
	
	// MET clear knob
	oapiVCRegisterArea(MET_CLEAR, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(MET_CLEAR, _V(-0.218241, -0.564396 - mesh_shift_y, 0.340942 + mesh_shift_z), 0.0051);

	// Second advance button
	oapiVCRegisterArea(SEC_ADVANCE, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(SEC_ADVANCE, _V(-0.078485, -0.573357 - mesh_shift_y, 0.309527 + mesh_shift_z), 0.0045);

	// clock setting knob
	oapiVCRegisterArea(CLOCK_SET, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN|PANEL_MOUSE_RBDOWN|PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_RBPRESSED);
	oapiVCSetAreaClickmode_Spherical(CLOCK_SET, _V(-0.121312, -0.574029 - mesh_shift_y, 0.291207 + mesh_shift_z), 0.007);

	// CM JETTISON
	oapiVCRegisterArea(CM_JETT_BUTTON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCSetAreaClickmode_Spherical(CM_JETT_BUTTON, _V(-0.029415, -0.559737 - mesh_shift_y, 0.385593 + mesh_shift_z), 0.008);

	// EMERGENCY UNDOCK
	oapiVCRegisterArea(EMERG_UNDOCK_BUTTON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCSetAreaClickmode_Spherical(EMERG_UNDOCK_BUTTON, _V(-0.012733, -0.559737 - mesh_shift_y, 0.385593 + mesh_shift_z), 0.008);

	// SKDU ON block
	oapiVCRegisterArea(BLK_SKDU_ON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(BLK_SKDU_ON, _V(-0.029415, -0.567945 - mesh_shift_y, 0.348643 + mesh_shift_z), 0.008);

	// SKDU OFF block
	oapiVCRegisterArea(BLK_SKDU_OFF, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(BLK_SKDU_OFF, _V(-0.012732, -0.567945 - mesh_shift_y, 0.348643 + mesh_shift_z), 0.008);

	// SKDU ON
	oapiVCRegisterArea(CMD_SKDU_ON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_LBUP);
	oapiVCSetAreaClickmode_Spherical(CMD_SKDU_ON, _V(-0.029415, -0.563814 - mesh_shift_y, 0.36724 + mesh_shift_z), 0.008);

	// SKDU OFF
	oapiVCRegisterArea(CMD_SKDU_OFF, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED|PANEL_MOUSE_LBUP);
	oapiVCSetAreaClickmode_Spherical(CMD_SKDU_OFF, _V(-0.012733, -0.563814 - mesh_shift_y, 0.36724 + mesh_shift_z), 0.008);

	// KSU LEFT SELECT
	oapiVCRegisterArea(KSU_LEFT_BUTTON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(KSU_LEFT_BUTTON, _V(-0.029415, -0.572098 - mesh_shift_y, 0.32995 + mesh_shift_z), 0.008);

	// KSU RIGHT SELECT
	oapiVCRegisterArea(KSU_RIGHT_BUTTON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(KSU_RIGHT_BUTTON, _V(-0.012732, -0.572098 - mesh_shift_y, 0.32995 + mesh_shift_z), 0.008);

	// KSU BOTH SELECT
	oapiVCRegisterArea(KSU_BOTH_BUTTON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(KSU_BOTH_BUTTON, _V(-0.029415, -0.576156 - mesh_shift_y, 0.311682 + mesh_shift_z), 0.008);

	// KSU OFF SELECT
	oapiVCRegisterArea(KSU_OFF_BUTTON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(KSU_OFF_BUTTON, _V(-0.012732, -0.576156 - mesh_shift_y, 0.311682 + mesh_shift_z), 0.008);

	// PREPARE RB
	oapiVCRegisterArea(PREP_RB_BUTTON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(PREP_RB_BUTTON, _V(-0.029415, -0.580271 - mesh_shift_y, 0.29316 + mesh_shift_z), 0.008);

	// REMOVE PREPARE RB
	oapiVCRegisterArea(NOPREP_RB_BUTTON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCSetAreaClickmode_Spherical(NOPREP_RB_BUTTON, _V(-0.012732, -0.580271 - mesh_shift_y, 0.29316 + mesh_shift_z), 0.008);

	// INK GLOBE ADJUST - O
	oapiVCRegisterArea(INK_O_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_PRESSED);
	oapiVCSetAreaClickmode_Quadrilateral(INK_O_KNB, _V(-0.27402, -0.537453 - mesh_shift_y, 0.461039 + mesh_shift_z), _V(-0.245011, -0.537453 - mesh_shift_y, 0.461039 + mesh_shift_z), _V(-0.27402, -0.543744 - mesh_shift_y, 0.432721 + mesh_shift_z), _V(-0.245011, -0.543744 - mesh_shift_y, 0.432721 + mesh_shift_z));

	// INK GLOBE ADJUST - E
	oapiVCRegisterArea(INK_E_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_PRESSED);
	oapiVCSetAreaClickmode_Spherical(INK_E_KNB, _V(-0.259515, -0.530935 - mesh_shift_y, 0.444733 + mesh_shift_z), 0.008);

	// INK MODE SELECT KNOB
	oapiVCRegisterArea(INK_SELECT_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_RBDOWN);
	oapiVCSetAreaClickmode_Spherical(INK_SELECT_KNB, _V(-0.263337, -0.515346 - mesh_shift_y, 0.559348 + mesh_shift_z), 0.02);

	// INK DIGIT SELECT KNOB
	oapiVCRegisterArea(INK_PERIOD_DIGIT_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_RBDOWN);
	oapiVCSetAreaClickmode_Spherical(INK_PERIOD_DIGIT_KNB, _V(-0.215552, -0.520568 - mesh_shift_y, 0.515959 + mesh_shift_z), 0.015);

	// INK PERIOD SET KNOB
	oapiVCRegisterArea(INK_PERIOD_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_RBDOWN);
	oapiVCSetAreaClickmode_Spherical(INK_PERIOD_KNB, _V(-0.215552, -0.501044 - mesh_shift_y, 0.511622 + mesh_shift_z), 0.008);

	// INK PERIOD DISPLAY
	oapiVCRegisterArea(INK_PERIOD, _R(460, 128, 616, 164), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_first);

	// INK ORBIT COUNT KNOB
	oapiVCRegisterArea(INK_VIT_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_PRESSED);
	oapiVCSetAreaClickmode_Spherical(INK_VIT_KNB, _V(-0.178357, -0.532508 - mesh_shift_y, 0.471323 + mesh_shift_z), 0.01);

	// INK ORBIT COUNT DISPLAY
	oapiVCRegisterArea(INK_VIT, _R(444, 242, 614, 278), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_first);

	// INK UGOL KNOB
	oapiVCRegisterArea(INK_UGOL_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_PRESSED);
	oapiVCSetAreaClickmode_Spherical(INK_UGOL_KNB, _V(-0.1827, -0.539757 - mesh_shift_y, 0.438694 + mesh_shift_z), 0.01);

	// INK UGOL DISPLAY
	oapiVCRegisterArea(INK_UGOL, _R(444, 318, 566, 354), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_first);

	// IRS Speed Scale
	oapiVCRegisterArea(IRS_SPEED, _R(8, 784, 72, 814), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_first);

	// IRS Range Scales
	oapiVCRegisterArea(IRS_RANGE_10, _R(6, 676, 70, 706), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_first);
	oapiVCRegisterArea(IRS_RANGE_100, _R(73, 676, 137, 706), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_first);
	
	// IRS needles
	oapiVCRegisterArea(IRS_SPEED_NEEDLE, PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE);
	oapiVCRegisterArea(IRS_RANGE_NEEDLE, PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE);

	// INT Knob
	oapiVCRegisterArea(INT_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_RBDOWN);
	oapiVCSetAreaClickmode_Spherical(INT_KNB, _V(-0.412873, -0.574465 - mesh_shift_y, 0.297633 + mesh_shift_z), 0.01);

	// INT needle
	oapiVCRegisterArea(INT_NEEDLE, PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE);

	surf_int = oapiGetTextureHandle(VC_panel_mesh, 15);

	// INT Scale
	oapiVCRegisterArea(INT_SCALE, _R(0, 0, 1023, 1023), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_int);
	

	// Load textures for electroluminescent displays
	surf_dynamic_second = oapiGetTextureHandle(VC_panel_mesh, 3);
	surf_dynamic_third = oapiGetTextureHandle(VC_panel_mesh, 12);

	// IKP PANEL
	oapiVCRegisterArea(IKP_PANEL, _R(20, 14, 695, 1009), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_third);

	// INK MP LIGHT
	oapiVCRegisterArea(INK_MP, _R(1063, 589, 1268, 682), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	
	// ELS
	oapiVCRegisterArea(ELS_1, _R(13, 4, 218, 97), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_2, _R(222, 4, 427, 97), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_3, _R(431, 4, 636, 97), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_4, _R(640, 4, 845, 97), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_5, _R(13, 101, 218, 194), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_6, _R(222, 101, 427, 194), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_7, _R(431, 101, 636, 194), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_8, _R(640, 101, 845, 194), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_9, _R(13, 198, 218, 291), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_10, _R(222, 198, 427, 291), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_11, _R(431, 198, 636, 291), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_12, _R(640, 198, 845, 291), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);

	oapiVCRegisterArea(ELS_13, _R(854, 4, 1059, 97), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_14, _R(1063, 4, 1268, 97), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_15, _R(1272, 4, 1477, 97), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_16, _R(1481, 4, 1686, 97), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_17, _R(854, 101, 1059, 194), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_18, _R(1063, 101, 1268, 194), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_19, _R(1272, 101, 1477, 194), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_20, _R(1481, 101, 1686, 194), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_21, _R(854, 198, 1059, 291), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_22, _R(1063, 198, 1268, 291), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_23, _R(1272, 198, 1477, 291), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_24, _R(1481, 198, 1686, 291), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);

	oapiVCRegisterArea(ELS_25, _R(13, 589, 218, 682), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_26, _R(222, 589, 427, 682), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_27, _R(431, 589, 636, 682), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_28, _R(640, 589, 845, 682), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_29, _R(13, 686, 218, 779), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_30, _R(222, 686, 427, 779), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_31, _R(431, 686, 636, 779), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_32, _R(640, 686, 845, 779), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_33, _R(13, 783, 218, 876), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_34, _R(222, 783, 427, 876), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_35, _R(431, 783, 636, 876), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);
	oapiVCRegisterArea(ELS_36, _R(640, 783, 845, 876), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);

	// Right side buttons
	// ELS
	oapiVCRegisterArea(ELS_BUTTON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(ELS_BUTTON, _V(0.430652, -0.551505 - mesh_shift_y, 0.415682 + mesh_shift_z), 0.008);

	// IKP
	oapiVCRegisterArea(IKP_BUTTON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(IKP_BUTTON, _V(0.447334, -0.551505 - mesh_shift_y, 0.415682 + mesh_shift_z), 0.008);

	// MAIN BATTERY
	oapiVCRegisterArea(MAIN_BATT_BUTTON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(MAIN_BATT_BUTTON, _V(0.430652, -0.555173 - mesh_shift_y, 0.39917 + mesh_shift_z), 0.008);

	// BACKUP BATTERY
	oapiVCRegisterArea(BCKP_BATT_BUTTON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(BCKP_BATT_BUTTON, _V(0.447334, -0.555173 - mesh_shift_y, 0.39917 + mesh_shift_z), 0.008);

	// SOUND
	oapiVCRegisterArea(SOUND_BUTTON, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCSetAreaClickmode_Spherical(SOUND_BUTTON, _V(0.399168, -0.551505 - mesh_shift_y, 0.415682 + mesh_shift_z), 0.008);

	//KSUl Buttons
	oapiVCRegisterArea(KSUl_1, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_2, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_3, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_4, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_5, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_6, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_7, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_8, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_9, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_10, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_11, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_12, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_13, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_14, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_15, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_16, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_17, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_18, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_19, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_20, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_21, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_22, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_23, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUl_24, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCSetAreaClickmode_Spherical(KSUl_1, _V(-0.612572, -0.31754 - mesh_shift_y, 0.548376 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_2, _V(-0.637384, -0.305061 - mesh_shift_y, 0.544522 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_3, _V(-0.612721, -0.322573 - mesh_shift_y, 0.533014 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_4, _V(-0.637384, -0.309832 - mesh_shift_y, 0.529079 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_5, _V(-0.612721, -0.327344 - mesh_shift_y, 0.517571 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_6, _V(-0.637384, -0.314602 - mesh_shift_y, 0.513635 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_7, _V(-0.612721, -0.332114 - mesh_shift_y, 0.502128 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_8, _V(-0.637384, -0.319372 - mesh_shift_y, 0.498192 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_9, _V(-0.612721, -0.336884 - mesh_shift_y, 0.486685 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_10, _V(-0.637384, -0.324143 - mesh_shift_y, 0.482749 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_11, _V(-0.612721, -0.341654 - mesh_shift_y, 0.471241 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_12, _V(-0.637384, -0.328913 - mesh_shift_y, 0.467306 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_13, _V(-0.612721, -0.346425 - mesh_shift_y, 0.455798 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_14, _V(-0.637384, -0.333683 - mesh_shift_y, 0.451862 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_15, _V(-0.612721, -0.351195 - mesh_shift_y, 0.440355 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_16, _V(-0.637384, -0.338453 - mesh_shift_y, 0.436419 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_17, _V(-0.612721, -0.355965 - mesh_shift_y, 0.424912 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_18, _V(-0.637384, -0.343224 - mesh_shift_y, 0.420976 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_19, _V(-0.612721, -0.360736 - mesh_shift_y, 0.409468 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_20, _V(-0.637384, -0.347994 - mesh_shift_y, 0.405533 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_21, _V(-0.612721, -0.365506 - mesh_shift_y, 0.394025 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_22, _V(-0.637384, -0.352764 - mesh_shift_y, 0.39009 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_23, _V(-0.612721, -0.370276 - mesh_shift_y, 0.378582 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_24, _V(-0.637384, -0.357535 - mesh_shift_y, 0.374646 + mesh_shift_z), 0.008);

	oapiVCRegisterArea(KSUl_A, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_B, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_V, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_G, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_D, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_E, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_ZH, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_I, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_K, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_L, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_M, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_N, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_P, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_R, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_S, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUl_T, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(KSUl_A, _V(-0.506034, -0.357033 - mesh_shift_y, 0.561711 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_B, _V(-0.506023, -0.36254 - mesh_shift_y, 0.546273 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_V, _V(-0.506023, -0.367373 - mesh_shift_y, 0.530628 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_G, _V(-0.506023, -0.372205 - mesh_shift_y, 0.514982 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_D, _V(-0.506023, -0.377038 - mesh_shift_y, 0.499337 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_E, _V(-0.506023, -0.381871 - mesh_shift_y, 0.483691 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_ZH, _V(-0.506023, -0.386704 - mesh_shift_y, 0.468046 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_I, _V(-0.506023, -0.391537 - mesh_shift_y, 0.4524 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_K, _V(-0.506023, -0.396369 - mesh_shift_y, 0.436754 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_L, _V(-0.506023, -0.401202 - mesh_shift_y, 0.421109 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_M, _V(-0.506023, -0.406035 - mesh_shift_y, 0.405463 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_N, _V(-0.506023, -0.410868 - mesh_shift_y, 0.389817 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_P, _V(-0.506023, -0.415701 - mesh_shift_y, 0.374172 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_R, _V(-0.506023, -0.420534 - mesh_shift_y, 0.358526 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_S, _V(-0.506023, -0.425366 - mesh_shift_y, 0.342881 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUl_T, _V(-0.506023, -0.430199 - mesh_shift_y, 0.327235 + mesh_shift_z), 0.008);


	// KSUp buttons
	oapiVCRegisterArea(KSUp_1, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_2, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_3, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_4, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_5, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_6, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_7, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_8, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_9, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_10, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_11, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_12, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_13, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_14, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_15, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_16, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_17, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_18, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_19, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_20, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_21, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_22, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_23, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCRegisterArea(KSUp_24, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBPRESSED | PANEL_MOUSE_LBUP);
	oapiVCSetAreaClickmode_Spherical(KSUp_1, _V(0.628957, -0.31758 - mesh_shift_y, 0.548376 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_2, _V(0.653175, -0.305465 - mesh_shift_y, 0.544522 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_3, _V(0.629106, -0.322614 - mesh_shift_y, 0.533014 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_4, _V(0.653175, -0.310236 - mesh_shift_y, 0.529079 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_5, _V(0.629106, -0.327384 - mesh_shift_y, 0.517571 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_6, _V(0.653175, -0.315006 - mesh_shift_y, 0.513635 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_7, _V(0.629106, -0.332154 - mesh_shift_y, 0.502128 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_8, _V(0.653175, -0.319776 - mesh_shift_y, 0.498192 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_9, _V(0.629106, -0.336924 - mesh_shift_y, 0.486685 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_10, _V(0.653175, -0.324547 - mesh_shift_y, 0.482749 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_11, _V(0.629106, -0.341695 - mesh_shift_y, 0.471241 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_12, _V(0.653175, -0.329317 - mesh_shift_y, 0.467306 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_13, _V(0.629106, -0.346465 - mesh_shift_y, 0.455798 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_14, _V(0.653175, -0.334087 - mesh_shift_y, 0.451862 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_15, _V(0.629106, -0.351235 - mesh_shift_y, 0.440355 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_16, _V(0.653175, -0.338857 - mesh_shift_y, 0.436419 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_17, _V(0.629106, -0.356006 - mesh_shift_y, 0.424912 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_18, _V(0.653175, -0.343628 - mesh_shift_y, 0.420976 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_19, _V(0.629106, -0.360776 - mesh_shift_y, 0.409468 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_20, _V(0.653175, -0.348398 - mesh_shift_y, 0.405533 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_21, _V(0.629106, -0.365546 - mesh_shift_y, 0.394025 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_22, _V(0.653175, -0.353168 - mesh_shift_y, 0.39009 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_23, _V(0.629106, -0.370316 - mesh_shift_y, 0.378582 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_24, _V(0.653175, -0.357939 - mesh_shift_y, 0.374646 + mesh_shift_z), 0.008);

	oapiVCRegisterArea(KSUp_A, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_B, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_V, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_G, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_D, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_E, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_ZH, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_I, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_K, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_L, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_M, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_N, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_P, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_R, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_S, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(KSUp_T, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(KSUp_A, _V(0.526306, -0.357033 - mesh_shift_y, 0.561711 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_B, _V(0.526289, -0.36254 - mesh_shift_y, 0.546273 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_V, _V(0.526289, -0.367373 - mesh_shift_y, 0.530628 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_G, _V(0.526289, -0.372205 - mesh_shift_y, 0.514982 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_D, _V(0.526289, -0.377038 - mesh_shift_y, 0.499337 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_E, _V(0.526289, -0.381871 - mesh_shift_y, 0.483691 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_ZH, _V(0.526289, -0.386704 - mesh_shift_y, 0.468046 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_I, _V(0.526289, -0.391537 - mesh_shift_y, 0.4524 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_K, _V(0.526289, -0.396369 - mesh_shift_y, 0.436754 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_L, _V(0.526289, -0.401202 - mesh_shift_y, 0.421109 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_M, _V(0.526289, -0.406035 - mesh_shift_y, 0.405463 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_N, _V(0.526289, -0.410868 - mesh_shift_y, 0.389817 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_P, _V(0.526289, -0.415701 - mesh_shift_y, 0.374172 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_R, _V(0.526289, -0.420534 - mesh_shift_y, 0.358526 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_S, _V(0.526289, -0.425366 - mesh_shift_y, 0.342881 + mesh_shift_z), 0.008);
	oapiVCSetAreaClickmode_Spherical(KSUp_T, _V(0.526289, -0.430199 - mesh_shift_y, 0.327235 + mesh_shift_z), 0.008);

	surf_dynamic_KSUl = oapiGetTextureHandle(VC_KSU_mesh, 3);

	oapiVCRegisterArea(KSUl_cyl, _R(0, 0, 2047, 1640), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_KSUl);
	
	//BTSI displays
	//oapiVCRegisterArea(BTSI_1, _R(196, 128, 378, 164), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_first);
	//oapiVCRegisterArea(BTSI_2, _R(196, 166, 378, 202), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_first);

	//oapiVCRegisterArea(BTSI_4, _R(196, 242, 378, 278), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_first);
	oapiVCRegisterArea(BTSI_5, _R(196, 280, 378, 316), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_RBDOWN, PANEL_MAP_BGONREQUEST, surf_dynamic_first);
	oapiVCRegisterArea(BTSI_6, _R(196, 318, 378, 354), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BACKGROUND, surf_dynamic_first);
	
	oapiVCRegisterArea(BTSI_DKD, _R(854, 589, 1059, 682), PANEL_REDRAW_ALWAYS, PANEL_MOUSE_IGNORE, PANEL_MAP_BGONREQUEST, surf_dynamic_second);

	//BTSI knobs
	//oapiVCRegisterArea(BTSI_1_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	//oapiVCRegisterArea(BTSI_2_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	//oapiVCRegisterArea(BTSI_3_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	//oapiVCRegisterArea(BTSI_4_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(BTSI_5_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_PRESSED);
	oapiVCRegisterArea(BTSI_6_KNB, PANEL_REDRAW_NEVER, PANEL_MOUSE_PRESSED);

	oapiVCSetAreaClickmode_Spherical(BTSI_5_KNB, _V(0.375614, -0.532315 - mesh_shift_y, 0.473173 + mesh_shift_z), 0.01);
	oapiVCSetAreaClickmode_Spherical(BTSI_6_KNB, _V(0.431066, -0.532315 - mesh_shift_y, 0.473173 + mesh_shift_z), 0.01);

	//oapiVCSetAreaClickmode_Quadrilateral(BTSI_5, _V(0.358406, -0.535275 - mesh_shift_y, 0.49578 + mesh_shift_z), _V(0.39051, -0.535275 - mesh_shift_y, 0.49578 + mesh_shift_z), _V(0.358406, -0.536659 - mesh_shift_y, 0.489549 + mesh_shift_z), _V(0.39051, -0.536659 - mesh_shift_y, 0.489549 + mesh_shift_z));


	for (int i = 0; i < 36; i++) {
		first_ELS_load[i] = 1;
	}
	first_BTSI_DKD_load = 1;
	first_MP_load = 1;
	IKP_update = 1;
	KSU_update = 1;
	INK_update = 1;
	if (MET_update == 0) MET_update = 3;
	btsi_5_flag = 2;
	ink_ugol_flag = 2;
	ink_vit_flag = 2;

	crono_digit_changed = 1;
	if (IRS_speed_scale == 40 || IRS_speed_scale == 2) IRS_speed_scale--;
	if (IRS_range_scale == 5000 || IRS_range_scale == 50000) IRS_range_scale--;

	return ok;
}

bool Soyuz7k_T::clbkVCRedrawEvent(int id, int event, SURFHANDLE surf) {
	
	switch (id) {

	case VZOR_CENTRAL:
		if (vzor_c_on == 2) {		// render a blank before setting to zero and off
			oapiBltPanelAreaBackground(VZOR_CENTRAL, surf);
			oapiClearSurface(hSurfVzor);
			oapiClearSurface(hSurfVzor2);
			oapiBlt(surf, hSurfVzor, 0, 0, 0, 0, 1023, 1023);
			vzor_c_on = 0;
			return true;
		}
		else if (sep_status > 2 || !vzor_c_on) return false;
		oapiBltPanelAreaBackground(VZOR_CENTRAL, surf);
		VzorUpdate(surf, 0);
		return true;

	case VZOR_PERIPH:
		if (vzor_p_on == 2) {		// render a blank before setting to zero and off
			oapiBltPanelAreaBackground(VZOR_PERIPH, surf);
			oapiClearSurface(hSurfVzor3);
			oapiBlt(surf, hSurfVzor3, 0, 0, 0, 0, 1023, 1023);
			vzor_p_on = 0;
			return true;
		}
		else if (sep_status > 2 || !vzor_p_on) return false;	// do nothing
		oapiBltPanelAreaBackground(VZOR_PERIPH, surf);
		VzorUpdate(surf, 1);
		return true;

	case KSUl_cyl:
		return KSUBlt(surf);

	case BTSI_5:
	case BTSI_6:
		BTSIBlt(surf, id - BTSI_1);
		return true;

	case INK_PERIOD:
		return INKBlt(surf, 0);

	case INK_VIT:
		return INKBlt(surf, 1);

	case INK_UGOL:
		return INKBlt(surf, 2);

	case IKP_PANEL:
		return IKPBlt(surf);

	case CRONO_DIGIT:
		if (crono_digit_changed == 1) {
			oapiBltPanelAreaBackground(CRONO_DIGIT, surf);
			if (crono_enable == 0) {
				oapiBlt(surf, dyn_tex_first, 0, 0, 42, 106, 21, 27);
			}
			else if (crono_enable == 2 || crono_enable == 1) {
				oapiBlt(surf, dyn_tex_first, 0, 0, 75, 70, 21, 27);
			}
			else {
				oapiBlt(surf, dyn_tex_first, 0, 0, 108, 70, 21, 27);
			}
			crono_digit_changed = 0;
			return true;
		}

	case MET_MIN_DISP:
		return METBlt(0, surf);
	case MET_HOUR_DISP:
		return METBlt(1, surf);
	case MET_DAY_DISP:
		return METBlt(2, surf);

	case IRS_SPEED:
		if (IRS_speed_scale == 1) {
			oapiBltPanelAreaBackground(IRS_SPEED, surf);
			oapiBlt(surf, dyn_tex_first, 0, 0, 75, 784, 64, 30);
			IRS_speed_scale++;
			return true;
		}
		else if (IRS_speed_scale == 39) {
			oapiBltPanelAreaBackground(IRS_SPEED, surf);
			oapiBlt(surf, dyn_tex_first, 0, 0, 8, 784, 64, 30);
			IRS_speed_scale++;
			return true;
		}
		return false;

	case IRS_RANGE_10:
		if (IRS_range_scale == 4999) {
			oapiBltPanelAreaBackground(IRS_RANGE_10, surf);
			oapiBlt(surf, dyn_tex_first, 13, 6, 19, 719, 36, 18);
			IRS_range_scale++;
			return true;
		}
		else if (IRS_range_scale == 49999 || IRS_range_scale == 498 || IRS_range_scale == 499) {
			oapiBltPanelAreaBackground(IRS_RANGE_10, surf);
			oapiBlt(surf, dyn_tex_first, 0, 0, 6, 676, 64, 30);
			if (IRS_range_scale == 498 || IRS_range_scale == 499) IRS_range_scale++;
			return true;
		}
		return false;

	case IRS_RANGE_100:
		if (IRS_range_scale == 49999) {
			oapiBltPanelAreaBackground(IRS_RANGE_100, surf);
			oapiBlt(surf, dyn_tex_first, 8, 6, 81, 719, 47, 18);
			IRS_range_scale++;
			return true;
		}
		else if (IRS_range_scale == 4999 || IRS_range_scale == 498 || IRS_range_scale == 499){
			oapiBltPanelAreaBackground(IRS_RANGE_100, surf);
			oapiBlt(surf, dyn_tex_first, 0, 0, 73, 676, 64, 30);
			if (IRS_range_scale == 498 || IRS_range_scale == 499) IRS_range_scale++;
			return true;
		}
		return false;

	case IRS_SPEED_NEEDLE:
		if ((igla_status > 0 || man_appr) && (IRS_speed_scale == 2 || IRS_speed_scale == 40)) {
			IRS_speed_proc = tgt_range_rate / (double)IRS_speed_scale;
			SetAnimation(anim_IRS_speed, IRS_speed_proc);
			return true;
		}
		else if (igla_status == 0 && !man_appr && IRS_speed_proc != 0) {
			IRS_speed_proc = 0;
			SetAnimation(anim_IRS_speed, IRS_speed_proc);
			return true;
		}
		//else return false;

	case IRS_RANGE_NEEDLE:
		if ((igla_status > 0 || man_appr) && (IRS_range_scale == 500 || IRS_range_scale == 5000 || IRS_range_scale == 50000)) {
			IRS_range_proc = tgt_range / (double)IRS_range_scale;
			SetAnimation(anim_IRS_range, IRS_range_proc);
			return true;
		}
		else if (igla_status == 0 && !man_appr && IRS_range_proc != 0) {
			IRS_range_proc = 0;
			SetAnimation(anim_IRS_range, IRS_range_proc);
			return true;
		}
		//else return false;

	case INT_SCALE:
		oapiBltPanelAreaBackground(INT_SCALE, surf);
		if (INT_status != 3) {
			oapiBlt(surf, dyn_tex_first, 145, 465, 194, 576, 84, 84);	//0
			oapiBlt(surf, dyn_tex_first, 297, 355, 194, 671, 84, 84);	//1
			oapiBlt(surf, dyn_tex_first, 465, 318, 318, 671, 84, 84);	//2
			oapiBlt(surf, dyn_tex_first, 632, 354, 194, 763, 84, 84);	//3
			oapiBlt(surf, dyn_tex_first, 776, 467, 318, 763, 84, 84);	//4 //+1tgt
		}
		else {
			oapiBlt(surf, dyn_tex_first, 145, 465, 194, 576, 84, 84);	//0
			oapiBlt(surf, dyn_tex_first, 297, 355, 318, 671, 84, 84);	//2
			oapiBlt(surf, dyn_tex_first, 465, 318, 318, 763, 84, 84);	//4
			oapiBlt(surf, dyn_tex_first, 632, 354, 194, 859, 84, 84);	//6
			oapiBlt(surf, dyn_tex_first, 776, 467, 318, 859, 84, 84);	//8 //+1tgt
		}
		return true;

	case INT_NEEDLE:
		double INT_needle_proc;
		if (INT_status == 1) {
			
			if (BB_check) {
				if (SA_BB_status) INT_needle_proc = SA_BB_voltage / 40.0;
				else INT_needle_proc = BB_voltage / 40.0;
			}
			else if (RB_check) {
				INT_needle_proc = RB_voltage / 40.0;
			}
			else {
				INT_needle_proc = 0.0;
			}
			SetAnimation(anim_INT_needle, INT_needle_proc);
			return true;
		}
		else if (INT_status == 2) {

			INT_needle_proc = charge_current / 40.0;
			SetAnimation(anim_INT_needle, INT_needle_proc);
			return true;
		}
		else if (INT_status == 3) {
			
			if (BB_check) {
				if (SA_BB_status) INT_needle_proc = SA_BB_current / 80.0;
				else INT_needle_proc = BB_current / 80.0;
			}
			else if (RB_check) {
				INT_needle_proc = RB_current / 80.0;
			}
			else {
				INT_needle_proc = 0.0;
			}
			SetAnimation(anim_INT_needle, INT_needle_proc);
			return true;
		}
		else {
			INT_needle_proc = 0;
			SetAnimation(anim_INT_needle, INT_needle_proc);
			return true;
		}

	case INK_MP:
	case BTSI_DKD:
	case ELS_1:
	case ELS_2:
	case ELS_3:
	case ELS_4:
	case ELS_5:
	case ELS_6:
	case ELS_7:
	case ELS_8:
	case ELS_9:
	case ELS_10:
	case ELS_11:
	case ELS_12:
	case ELS_13:
	case ELS_14:
	case ELS_15:
	case ELS_16:
	case ELS_17:
	case ELS_18:
	case ELS_19:
	case ELS_20:
	case ELS_21:
	case ELS_22:
	case ELS_23:
	case ELS_24:
	case ELS_25:
	case ELS_26:
	case ELS_27:
	case ELS_28:
	case ELS_29:
	case ELS_30:
	case ELS_31:
	case ELS_32:
	case ELS_33:
	case ELS_34:
	case ELS_35:
	case ELS_36: 
		return ELSBlt(id, surf);
	}

	return false;
}

bool Soyuz7k_T::clbkVCMouseEvent(int id, int event, VECTOR3& p) {
	
	switch (id) {

	case VZOR_P_TOGGLE:
		if (event == PANEL_MOUSE_LBDOWN) {
			if (gcEnabled()) {
				if (vzor_p_on) {
					vzor_p_on = 2;	// intermediate state to render a blank before setting to zero
					gcCustomCameraOnOff(hCam_vzor_periph, false);
				}
				else {
					vzor_p_on = 1;
					gcCustomCameraOnOff(hCam_vzor_periph, true);
				}
			}
			return true;
		}
		return false;

	case VZOR_C_TOGGLE:
		if (event == PANEL_MOUSE_LBDOWN) {
			if (gcEnabled()) {
				if (vzor_c_on) {
					vzor_c_on = 2;	// intermediate state to render a blank before setting to zero
					gcCustomCameraOnOff(hCam_vzor_fwd, false);
					gcCustomCameraOnOff(hCam_vzor_nad, false);
				}
				else {
					vzor_c_on = 1;
					if (vzor_status == 2 || vzor_status == 0) gcCustomCameraOnOff(hCam_vzor_fwd, true);
					else gcCustomCameraOnOff(hCam_vzor_nad, true);
				}
			}
			return true;
		}
		return false;

	case BTSI_5_KNB:
		if (event & PANEL_MOUSE_LBPRESSED) {
			btsi_5_flag = 1;
			return true;
		}
		else if (event & PANEL_MOUSE_RBPRESSED) {
			btsi_5_flag = -1;
			return true;
		}
	/* //switch knob position
	case BTSI_5:
		if (event == PANEL_MOUSE_LBDOWN) {
			if (BTSI_knb_5_digit == 3) {
				BTSI_knb_5_digit = 0;
				SetAnimation(anim_BTSI_knb_5, 0);
			}
		}
		else if (event == PANEL_MOUSE_RBDOWN) {
			if (BTSI_knb_5_digit == 0) {
				BTSI_knb_5_digit = 3;
				SetAnimation(anim_BTSI_knb_5, 1);
			}
		}
		return true;*/

	case INK_SELECT_KNB:
		if (event == PANEL_MOUSE_LBDOWN) {
			if (ink_select_status == 0) {
				SetAnimation(anim_ink_select_knb, 0.5);
				ink_select_status = 1;
				return true;
			}
			else if (ink_select_status == 1) {
				SetAnimation(anim_ink_select_knb, 1);
				ink_select_status = 2;
				//advance orbital track by ugol_pos degrees
				globe_o_proc -= ugol_pos / 360.0;
				//get decimal minutes equivalent to how long ugol_pos degrees take to travel in orbital track
				double x = ink_period * ugol_pos / 360.0;
				//percentage of rotation around the poles corresponding to previous calculated time x
				globe_e_proc += x / 1415.87;
				//save ugol on activation to track real-time changes in MP mode
				last_ugol_pos = ugol_pos;
				return true;
			}
		}
		else if (event == PANEL_MOUSE_RBDOWN) {
			if (ink_select_status == 2) {
				SetAnimation(anim_ink_select_knb, 0.5);
				ink_select_status = 1;
				//move back orbital track by ugol_pos degrees
				globe_o_proc += ugol_pos / 360.0;
				//get decimal minutes equivalent to how long ugol_pos degrees take to travel in orbital track
				double x = ink_period * ugol_pos / 360.0;
				//percentage of rotation around the poles corresponding to previous calculated time x
				globe_e_proc -= x / 1415.87;
				return true;
			}
			else if (ink_select_status == 1) {
				SetAnimation(anim_ink_select_knb, 0);
				ink_select_status = 0;
				return true;
			}
		}
		return true;

	case INK_PERIOD_DIGIT_KNB:
		if (event == PANEL_MOUSE_LBDOWN) {
			if (ink_period_status == 2) {
				SetAnimation(anim_ink_period_knb, 0.6);
				ink_period_status = 1;
				return true;
			}
			else if (ink_period_status == 1) {
				SetAnimation(anim_ink_period_knb, 0);
				ink_period_status = 0;
				return true;
			}
		}
		else if (event == PANEL_MOUSE_RBDOWN) {
			if (ink_period_status == 0) {
				SetAnimation(anim_ink_period_knb, 0.6);
				ink_period_status = 1;
				return true;
			}
			else if (ink_period_status == 1) {
				SetAnimation(anim_ink_period_knb, 1);
				ink_period_status = 2;
				return true;
			}
		}
		return true;

	case INK_PERIOD_KNB:
		if (event == PANEL_MOUSE_LBDOWN) {
			if (ink_period_status == 2 && ink_period > 86.85 && !no_power) {
				ink_period -= 0.01;
				globe_o_speed = 1.001 / (ink_period * 60.0);
				INK_update = 1;
				return true;
			}
			else if (ink_period_status == 1 && ink_period > 86.94 && !no_power) {
				ink_period -= 0.1;
				globe_o_speed = 1.001 / (ink_period * 60.0);
				INK_update = 1;
				return true;
			}
			else if (ink_period_status == 0 && ink_period > 87.84 && !no_power) {
				ink_period -= 1;
				globe_o_speed = 1.001 / (ink_period * 60.0);
				INK_update = 1;
				return true;
			}
		}
		else if (event == PANEL_MOUSE_RBDOWN) {
			if (ink_period_status == 0 && ink_period < 95.86 && !no_power) {
				ink_period += 1;
				globe_o_speed = 1.001 / (ink_period * 60.0);
				INK_update = 1;
				return true;
			}
			else if (ink_period_status == 1 && ink_period < 96.76 && !no_power) {
				ink_period += 0.1;
				globe_o_speed = 1.001 / (ink_period * 60.0);
				INK_update = 1;
				return true;
			}
			if (ink_period_status == 2 && ink_period <= 96.84 && !no_power) {
				ink_period += 0.01;
				globe_o_speed = 1.001 / (ink_period * 60.0);
				INK_update = 1;
				return true;
			}
		}
		return true;

	case INK_VIT_KNB:
		if (event & PANEL_MOUSE_LBPRESSED) {
			ink_vit_flag = -1;
			return true;
		}
		else if (event & PANEL_MOUSE_RBPRESSED) {
			ink_vit_flag = 1;
			return true;
		}
		//return true;

	case INK_UGOL_KNB:
		if (event & PANEL_MOUSE_LBPRESSED) {
			ink_ugol_flag = -1;
			return true;
		}
		else if (event & PANEL_MOUSE_RBPRESSED) {
			ink_ugol_flag = 1;
			return true;
		}
		//return true;

	case INK_O_KNB:
		if (event & PANEL_MOUSE_LBPRESSED) {
			ink_o_flag = 1;
			return true;
		}
		else if (event & PANEL_MOUSE_RBPRESSED) {
			ink_o_flag = -1;
			return true;
		}

	case INK_E_KNB:
		if (event & PANEL_MOUSE_LBPRESSED) {
			ink_e_flag = 1;
			return true;
		}
		else if (event & PANEL_MOUSE_RBPRESSED) {
			ink_e_flag = -1;
			return true;
		}

	case INT_KNB:
		if (event == PANEL_MOUSE_RBDOWN) {
			if (INT_status == 0) {
				SetAnimation(anim_INT_knob, 0);
				INT_status = 1;
				return true;
			}
			else if (INT_status == 1) {
				SetAnimation(anim_INT_knob, 0.143);
				INT_status = 2;
				return true;
			}
			else if (INT_status == 2) {
				SetAnimation(anim_INT_knob, 0.286);
				INT_status = 3;
				return true;
			}
		}
		else if (event == PANEL_MOUSE_LBDOWN) {
			if (INT_status == 3) {
				SetAnimation(anim_INT_knob, 0.143);
				INT_status = 2;
				return true;
			}
			else if (INT_status == 2) {
				SetAnimation(anim_INT_knob, 0);
				INT_status = 1;
				return true;
			}
			else if (INT_status == 1) {
				SetAnimation(anim_INT_knob, 1);
				INT_status = 0;
				return true;
			}
		}
		return true;

	case CM_JETT_BUTTON:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (CM_jett == 0 && !no_power) {
				CM_jett = 1;
				// only undock if hard-dock doesn't exist
				if (hMother && hooks_status != 2) {
					Undock(0);
					DelDock(hDock);
					hDock = NULL;
					combined_power = 0;
					recharge = 0;
				}
				// no undock command if not docked
				else if (!hMother) {
					DelDock(hDock);
					hDock = NULL;
				}
				// if hard-dock exists, docked state remains and dock port is deleted on undocking
				DelAnimation(anim_dock_probe);
				DelMesh(mesh[2], true);
				VECTOR3 ofs = _V(0, 0, mesh_shift_z);
				mesh[2] = AddMesh("Soyuz_7k\\T_BO_noCM", &ofs);
				SetEmptyMass(GetMass() - 150);
			}
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_CM_jett, 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_CM_jett, 0.0);
			return true;
		}

	case EMERG_UNDOCK_BUTTON:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (!no_power) {
				if (hMother && latches_status == 0) Undock(0);
				hooks_status = 5;
				hooks_proc = 0;
				combined_power = 0;
				recharge = 0;
				DelDock(hDock);
				hDock = NULL;
			}
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_emerg_undock, 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_emerg_undock, 0.0);
			return true;
		}

	case BLK_SKDU_ON:
		if (block_skdu_on == 0) {
			block_skdu_on = 1;
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_block_skdu_on, 1.0);
		}
		else {
			block_skdu_on = 0;
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			SetAnimation(anim_block_skdu_on, 0.0);
		}
		return true;

	case BLK_SKDU_OFF:
		if (block_skdu_off == 0) {
			block_skdu_off = 1;
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_block_skdu_off, 1.0);
		}
		else {
			block_skdu_off = 0;
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			SetAnimation(anim_block_skdu_off, 0.0);
		}
		return true;

	case CMD_SKDU_ON:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (!block_skdu_on && !no_power) command_skdu_on = 1;
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_skdu_on, 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP){
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_skdu_on, 0.0);
			return true;
		}

	case CMD_SKDU_OFF:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (!block_skdu_off && !no_power) command_skdu_on = 3;
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_skdu_off, 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_skdu_off, 0.0);
			return true;
		}

	case PREP_RB_BUTTON: 
		if (prepRB_status) return false;
		else {
			SetAnimation(anim_prepRB_button, 1.0);
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			prepRB_status = 1;
			// backup battery on
			RB_status = 1;
			// it's one or the other - maybe in the future, both remain on
			BB_status = 0;
			BB_voltage = 0;
			BB_current = 0;
			return true;
		}

	case NOPREP_RB_BUTTON:
		if (event == PANEL_MOUSE_LBPRESSED) {
			prepRB_status = 0;
			// swap batteries
			RB_status = 0;
			RB_voltage = 0;
			RB_current = 0;
			BB_status = 1;
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
				m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			}
			SetAnimation(anim_noprepRB_button, 1.0);
			SetAnimation(anim_prepRB_button, 0.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_noprepRB_button, 0.0);
			return true;
		}
		return false;

	case KSU_LEFT_BUTTON:
		if (KSU_left_enable == 1 && KSU_right_enable == 0) return false;
		else {
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_KSU_left_button, 1.0);
			SetAnimation(anim_KSU_right_button, 0.0);
			SetAnimation(anim_KSU_both_button, 0.0);
			SetAnimation(anim_KSU_off_button, 0.0);
			KSU_left_enable = 1;
			KSU_right_enable = 0;
			return true;
		}

	case KSU_RIGHT_BUTTON:
		if (KSU_left_enable == 0 && KSU_right_enable == 1) return false;
		else {
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_KSU_left_button, 0.0);
			SetAnimation(anim_KSU_right_button, 1.0);
			SetAnimation(anim_KSU_both_button, 0.0);
			SetAnimation(anim_KSU_off_button, 0.0);
			KSU_left_enable = 0;
			KSU_right_enable = 1;
			return true;
		}

	case KSU_BOTH_BUTTON:
		if (KSU_left_enable == 1 && KSU_right_enable == 1) return false;
		else {
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_KSU_left_button, 0.0);
			SetAnimation(anim_KSU_right_button, 0.0);
			SetAnimation(anim_KSU_both_button, 1.0);
			SetAnimation(anim_KSU_off_button, 0.0);
			KSU_left_enable = 1;
			KSU_right_enable = 1;
			return true;
		}

	case KSU_OFF_BUTTON:
		if (KSU_left_enable == 0 && KSU_right_enable == 0) return false;
		else {
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_KSU_left_button, 0.0);
			SetAnimation(anim_KSU_right_button, 0.0);
			SetAnimation(anim_KSU_both_button, 0.0);
			SetAnimation(anim_KSU_off_button, 1.0);
			KSU_left_enable = 0;
			KSU_right_enable = 0;
			return true;
		}

	case CLOCK_ON:
		if (clock_enable == 0) {
			clock_enable = 1;
			if (crono_enable == 1) clock_power = 0.0004;
			else clock_power = 0.0002;
			SetAnimation(anim_clk_enable, 0.0);
			m_pXRSound->PlayWav(ClockSwitch, false, 0.6);
			return true;
		}
		else if (clock_enable == 2) {
			clock_enable = 0;
			if (crono_enable == 1) clock_power = 0.0002;
			else clock_power = 0;
			SetAnimation(anim_clk_enable, 1.0);
			m_pXRSound->PlayWav(ClockSwitch, false, 0.6);
			return true;
		}

	case CRONO_TOGGLE:
		if (crono_enable == 0) {
			crono_enable = 1;
			if (clock_enable) clock_power = 0.0004;
			else clock_power = 0.0002;
			crono_digit_changed = 1;
			m_pXRSound->PlayWav(ChronoButton, false, 0.7);
			return true;
		}
		else if (crono_enable == 2) {
			crono_enable = 3;
			if (clock_enable) clock_power = 0.0002;
			else clock_power = 0;
			crono_digit_changed = 1;
			m_pXRSound->PlayWav(ChronoButton, false, 0.7);
			return true;
		}
		else if (crono_enable == 3) {
			crono_enable = 0;
			crono_digit_changed = 1;
			SetAnimation(anim_crono_sec, 0.0);
			SetAnimation(anim_crono_min, 0.0);
			SetAnimation(anim_crono_hour, 0.0);
			crono_sec_proc = 0;
			crono_min_proc = 0;
			crono_hour_proc = 0;
			m_pXRSound->PlayWav(ChronoButton, false, 0.7);
			return true;
		}

	case MET_CLEAR:
		MET_day = MET_hour = MET_minute = 0;
		start_MJD = last_MJD;
		return true;

	case SEC_ADVANCE:
		if (clock_enable == 0 && !no_power) {
			if (clock_set_status == 1) {
				clock_set_status = 0;
				SetAnimation(anim_clock_set, 0.0);
			}
			clk_sec_proc += 0.00833333333333333334;
			clk_min_proc += 0.00013888888888888888;
			clk_hour_proc += 0.000004629629629629628;
			if (clk_sec_proc >= 1) clk_sec_proc -= 1.00;
			if (clk_min_proc >= 1) clk_min_proc -= 1.00;
			if (clk_hour_proc >= 1) clk_hour_proc -= 1.00;
			SetAnimation(anim_clk_sec, clk_sec_proc);
			SetAnimation(anim_clk_min, clk_min_proc);
			SetAnimation(anim_clk_hour, clk_hour_proc);
			m_pXRSound->StopWav(ChronoButton);
			m_pXRSound->PlayWav(ChronoButton, false, 0.7);
			return true;
		}

	case CLOCK_SET:
		if (event == PANEL_MOUSE_RBDOWN && clock_set_status == 0) {
			SetAnimation(anim_clock_set, 1.0);
			clock_set_status = 1;
			return true;
		}
		else if (event == PANEL_MOUSE_LBDOWN && clock_set_status == 0 && clock_enable == 0 && (clk_sec_proc <= 0.00833333333333333334 || clk_sec_proc >= 0.99166666666666666666)) {
			start_MJD -= 0.000694444403;
			return true;
		}
		else if (event == PANEL_MOUSE_LBPRESSED && clock_set_status == 1 && clock_enable == 0) {
			clk_sec_proc -= 0.00833333333333333334;
			clk_min_proc -= 0.00013888888888888888;
			clk_hour_proc -= 0.000004629629629629628;
			if (clk_sec_proc <= 0) clk_sec_proc += 1.00;
			if (clk_min_proc <= 0) clk_min_proc += 1.00;
			if (clk_hour_proc <= 0) clk_hour_proc += 1.00;
			SetAnimation(anim_clk_sec, clk_sec_proc);
			SetAnimation(anim_clk_min, clk_min_proc);
			SetAnimation(anim_clk_hour, clk_hour_proc);
			return true;
		}
		else if (event == PANEL_MOUSE_RBPRESSED && clock_set_status == 1 && clock_enable == 0) {
			clk_sec_proc += 0.00833333333333333334;
			clk_min_proc += 0.00013888888888888888;
			clk_hour_proc += 0.000004629629629629628;
			if (clk_sec_proc >= 1) clk_sec_proc -= 1.00;
			if (clk_min_proc >= 1) clk_min_proc -= 1.00;
			if (clk_hour_proc >= 1) clk_hour_proc -= 1.00;
			SetAnimation(anim_clk_sec, clk_sec_proc);
			SetAnimation(anim_clk_min, clk_min_proc);
			SetAnimation(anim_clk_hour, clk_hour_proc);
			return true;
		}

	case ELS_BUTTON:
		if (ELS_check == 0) {
			ELS_check = 1;
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_ELS_button, 1.0);
		}
		else {
			ELS_check = 0;
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			SetAnimation(anim_ELS_button, 0.0);
		}
		return true;

	case IKP_BUTTON:
		if (IKP_check == 0) {
			IKP_check = 1;
			ikp_power = 0.01134;						// kW, 420 mA at 27 V
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_IKP_button, 1.0);
			IKP_update = 1;
		}
		else {
			IKP_check = 0;
			if (!program_on) ikp_power = 0.0108;		// kW, 400 mA at 27 V
			else ikp_power = 0.0162;					// kW, 600 mA at 27 V
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			SetAnimation(anim_IKP_button, 0.0);
			IKP_update = 1;
		}
		return true;

	case MAIN_BATT_BUTTON:
		if (BB_check == 0) {
			BB_check = 1;
			RB_check = 0;
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_BB_button, 1.0);
			SetAnimation(anim_RB_button, 0.0);
		}
		else {
			BB_check = 0;
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			SetAnimation(anim_BB_button, 0.0);
		}
		return true;

	case BCKP_BATT_BUTTON:
		if (RB_check == 0) {
			RB_check = 1;
			BB_check = 0;
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_BB_button, 0.0);
			SetAnimation(anim_RB_button, 1.0);
		}
		else {
			RB_check = 0;
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			SetAnimation(anim_RB_button, 0.0);
		}
		return true;

	case SOUND_BUTTON:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (avaria) {
				avaria = 0;
				m_pXRSound->StopWav(AlarmSound);
			}
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_sound_button, 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_sound_button, 0.0);
			return true;
		}

	case KSUl_A:
	case KSUl_B:
	case KSUl_V:
	case KSUl_G:
	case KSUl_D:
	case KSUl_E:
	case KSUl_ZH:
	case KSUl_I:
	case KSUl_K:
	case KSUl_L:
	case KSUl_M:
	case KSUl_N:
	case KSUl_P:
	case KSUl_R:
	case KSUl_S:
	case KSUl_T:
		if (KSUl_curr_col != id - KSUl_A && !KSUl_cyl_rotating && !no_power) {
			SetAnimation(anim_KSUl_sysbutt[KSUl_curr_col], 0.0);
			int coeff;
			if (id - KSUl_A > KSUl_curr_col) coeff = id - KSUl_A - KSUl_curr_col;
			else coeff = (KSUl_T - KSUl_A - KSUl_curr_col) + (id - KSUl_A) + 1;
			KSUl_rot_tgt = (double)coeff * 0.0625;
			KSUl_rot_acc = 0;
			KSUl_curr_col = id - KSUl_A;
			KSUl_cyl_rotating = 1;
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_KSUl_sysbutt[id - KSUl_A], 1.0);
			return true;
		}
		else if (no_power) {
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_KSUl_sysbutt[KSUl_curr_col], 0.0);
			SetAnimation(anim_KSUl_sysbutt[id - KSUl_A], 1.0);
			KSUl_curr_col = id - KSUl_A;
			return true;
		}
		return false;
	
	case KSUp_A:
	case KSUp_B:
	case KSUp_V:
	case KSUp_G:
	case KSUp_D:
	case KSUp_E:
	case KSUp_ZH:
	case KSUp_I:
	case KSUp_K:
	case KSUp_L:
	case KSUp_M:
	case KSUp_N:
	case KSUp_P:
	case KSUp_R:
	case KSUp_S:
	case KSUp_T:													// KSUp sys button logic:
		if (KSUp_curr_col != id - KSUp_A && !KSUp_cyl_rotating && !no_power) {		// if pressed sys button is not the currently active button
			SetAnimation(anim_KSUp_sysbutt[KSUp_curr_col], 0.0);		// pop out previous button
			int coeff;
			if (id - KSUp_A > KSUp_curr_col) coeff = id - KSUp_A - KSUp_curr_col;
			else coeff = (KSUp_T - KSUp_A - KSUp_curr_col) + (id - KSUp_A) + 1;
			KSUp_rot_tgt = (double)coeff * 0.0625;
			KSUp_rot_acc = 0;
			KSUp_curr_col = id - KSUp_A;								// switch current selected system flag
			KSUp_cyl_rotating = 1;										// trigger cylinder rotation
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_KSUp_sysbutt[id - KSUp_A], 1.0);			// pop pressed button in
			return true;
		}
		else if (no_power) {
			m_pXRSound->PlayWav(ButtonIn, false, 0.6);
			SetAnimation(anim_KSUp_sysbutt[KSUp_curr_col], 0.0);
			SetAnimation(anim_KSUp_sysbutt[id - KSUp_A], 1.0);
			KSUp_curr_col = id - KSUp_A;
			return true;
		}
		return false;

		//KSUl
	case KSUl_1:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[0], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_V - KSUl_A && KSU_left_enable == 1 && !no_power) {
				if (!AKSP_status) AKSP_status = 1;
			}
			else if (KSUl_curr_col == KSUl_ZH - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if ((dock_probe_status == 0 || dock_probe_status == 3) && ssvp_on) {
					dock_probe_status = 1;
				}
			}
			else if (KSUl_curr_col == KSUl_I - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				rcs_status = 3;
			}
			else if (KSUl_curr_col == KSUl_K - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {	//cancel all K modes
				BDUS_status = BDUS_OFF;
				pulsed_status = 2;
				if (ori_ap_status != 0 || inertial_status != 0 || modes_off != 1) {
					ori_ap_status = 0;		//switch orbital orientation mode off, regardless of current state
					ori_ap_aux = 0;
					inertial_status = 0;	//switch inertial orientation mode off, regardless of current state
					accel_z_status = 0;		//switch off longitudinal accelerometer
					accel_z = 0;			//reset integrating accelerometer
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
				}
			}
			else if (KSUl_curr_col == KSUl_L - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				manual_settings = 1;
			}
			else if (KSUl_curr_col == KSUl_M - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {	//cancel all M modes
				sun_ap_status = 0;		//switch solar orientation mode off, regardless of current state
				ori_ap_status = 0;		//switch orbital orientation mode off, regardless of current state
				ori_ap_aux = 0;
				inertial_status = 0;	//switch inertial orientation mode off, regardless of current state
				accel_z_status = 0;		//switch off longitudinal accelerometer
				accel_z = 0;			//reset integrating accelerometer
				BDUS_status = BDUS_OFF;
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
				SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
				program_on = 0;
				PVU_cmd_seq = 0;
				IKP_update = 1;
			}
			else if (KSUl_curr_col == KSUl_N - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				backup_DO = 1;
				for (int i = 0; i < 8; i++) {
					SetThrusterResource(th_DO[i], hpr_do_res);
				}
				m_pXRSound->PlayWav(ValveBlow, false, 1.0);
			}
			else if (KSUl_curr_col == KSUl_S - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				ssvp_on = 1;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[0], 0.0);
			return true;
		}

	case KSUl_2:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[1], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_S - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				ssvp_on = 0;
			}
			else if (KSUl_curr_col == KSUl_I - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (rcs_status == 0) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					SetThrusterGroupLevel(THGROUP_ATT_UP, 0);
					SetThrusterGroupLevel(THGROUP_ATT_DOWN, 0);
					SetThrusterGroupLevel(THGROUP_ATT_LEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_FORWARD, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BACK, 0);
					SetThrusterGroupLevel(THGROUP_RETRO, 0);
					DelThrusterGroup(THGROUP_ATT_YAWLEFT, false);
					DelThrusterGroup(THGROUP_ATT_YAWRIGHT, false);
					DelThrusterGroup(THGROUP_ATT_PITCHUP, false);
					DelThrusterGroup(THGROUP_ATT_PITCHDOWN, false);
					DelThrusterGroup(THGROUP_ATT_BANKLEFT, false);
					DelThrusterGroup(THGROUP_ATT_BANKRIGHT, false);
					DelThrusterGroup(THGROUP_ATT_UP, false);
					DelThrusterGroup(THGROUP_ATT_DOWN, false);
					DelThrusterGroup(THGROUP_ATT_LEFT, false);
					DelThrusterGroup(THGROUP_ATT_RIGHT, false);
					DelThrusterGroup(THGROUP_ATT_FORWARD, false);
					DelThrusterGroup(THGROUP_ATT_BACK, false);
					DelThrusterGroup(THGROUP_RETRO, false);
					rcs_status = 6;
				}
			}
			else if (KSUl_curr_col == KSUl_L - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				manual_settings = 0;
				btsi_5_read = 0;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[1], 0.0);
			return true;
		}

	case KSUl_3:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[2], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_ZH - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if ((latches_status == 0 || latches_status == 3) && ssvp_on) {
					latches_status = 1;
				}
			}
			else if (KSUl_curr_col == KSUl_I - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				rcs_status = 1;
			}
			else if (KSUl_curr_col == KSUl_K - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				BDUS_status = BDUS_ON;
			}
			else if (KSUl_curr_col == KSUl_M - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				PVU_cmd_off = 1;
			}
			else if (KSUl_curr_col == KSUl_N - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				
			}
			else if (KSUl_curr_col == KSUl_S - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if ((latches_status == 2 || latches_status == 1) && ssvp_on) {
					latches_status = 3;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[2], 0.0);
			return true;
		}

	case KSUl_4:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[3], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_M - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				PVU_cmd_off = 0;
			}
			else if (KSUl_curr_col == KSUl_I - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (rcs_status == 2) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					DelThrusterGroup(THGROUP_ATT_YAWLEFT, false);
					DelThrusterGroup(THGROUP_ATT_YAWRIGHT, false);
					DelThrusterGroup(THGROUP_ATT_PITCHUP, false);
					DelThrusterGroup(THGROUP_ATT_PITCHDOWN, false);
					DelThrusterGroup(THGROUP_ATT_BANKLEFT, false);
					DelThrusterGroup(THGROUP_ATT_BANKRIGHT, false);
					rcs_status = 6;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[3], 0.0);
			return true;
		}

	case KSUl_5:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[4], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_K - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				pulsed_status = 0;
			}
			else if (KSUl_curr_col == KSUl_ZH - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (igla_status > 0 && tgt_range <= 200) {
					man_appr = 1;
					IglaOff();
				}
			}
			else if (KSUl_curr_col == KSUl_P - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (!BDUS_integr) {
					BDUS_integr = 1;
				}
			}
			else if (KSUl_curr_col == KSUl_S - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if ((dock_probe_status == 2 || dock_probe_status == 1) && ssvp_on) {
					dock_probe_status = 3;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[4], 0.0);
			return true;
		}

	case KSUl_6:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[5], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_ZH - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (man_appr) {
					man_appr = 0;
					IglaOff();
				}
			}
			else if (KSUl_curr_col == KSUl_P - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (BDUS_integr) {
					BDUS_integr = 0;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[5], 0.0);
			return true;
		}

	case KSUl_7:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[6], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_ZH - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (sun_ap_status == 0 && ori_ap_status == 0 && igla_status == 0 && hAttached == NULL) {
					igla_status = 1;		//switch igla mode on
					rcs_status = 1;		  //force DO mode
				}
			}
			else if (KSUl_curr_col == KSUl_K - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				pulsed_status = 1;
			}
			else if (KSUl_curr_col == KSUl_M - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				program_on = 2;
				PVU_timer = 0;
				PVU_minute = 0;
				IKP_update = 1;
			}
			else if (KSUl_curr_col == KSUl_S - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if ((((hooks_status == 0 || hooks_status == 3) && !hMother) || (hMother && dock_probe_status == 4)) && ssvp_on) {
					hooks_status = 1;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[6], 0.0);
			return true;
		}

	case KSUl_8:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[7], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_ZH - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (igla_status > 0) {
					IglaOff();
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[7], 0.0);
			return true;
		}

	case KSUl_9:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[8], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_K - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (ori_ap_status == 0 && sun_ap_status == 0 && igla_status == 0 && BDUS_status == BDUS_ON) {
					ori_ap_status = 1;		//switch orbital orientation mode on
					rcs_status = 1;			//force DO mode
				}
			}
			else if (KSUl_curr_col == KSUl_S - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (GetDockStatus(hDock) && hooks_status == 2 && latches_status == 0 && !combined_power && ssvp_on) {
					hooks_status = 3;
					undock_commanded = 1;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[8], 0.0);
			return true;
		}

	case KSUl_10:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[9], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[9], 0.0);
			return true;
		}

	case KSUl_11:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[10], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_L - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (manual_settings) {
					btsi_5_read = 1;
					btsi_5_buffer = btsi_5_counter;
				}
			}
			else if (KSUl_curr_col == KSUl_M - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				program_on = 1;
				PVU_timer = 0;
				PVU_minute = 0;
				IKP_update = 1;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[10], 0.0);
			return true;
		}

	case KSUl_13:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[12], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_I - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				vzor_status = 1;
				if (gcEnabled()) {
					if (vzor_c_on) {
						vzor_c_on = 2;	// intermediate state to render a blank before setting to zero
						gcCustomCameraOnOff(hCam_vzor_fwd, false);
						gcCustomCameraOnOff(hCam_vzor_nad, false);
					}
				}
			}
			else if (KSUl_curr_col == KSUl_N - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (engine_status == 0 || engine_status == 3) {
					engine_status = 1;
				}
			}
			else if (KSUl_curr_col == KSUl_R - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (!combined_power && hMother && hooks_status == 2) {
					combined_power = 1;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[12], 0.0);
			return true;
		}

	case KSUl_14:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[13], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_N - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (engine_status == 1 || engine_status == 2) {
					engine_status = 3;
				}
			}
			else if (KSUl_curr_col == KSUl_R - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (combined_power) {
					combined_power = 0;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[13], 0.0);
			return true;
		}

	case KSUl_15:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[14], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_P - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				orb_ori_dir = 0;
			}
			else if (KSUl_curr_col == KSUl_M - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (ori_ap_status == 0 && sun_ap_status == 0 && igla_status == 0 && BDUS_status == BDUS_ON) {
					sun_ap_status = 1;		//switch solar orientation mode on
					rcs_status = 1;			//force DO mode
				}
			}
			else if (KSUl_curr_col == KSUl_I - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && BDUS_status == BDUS_ON && !no_power) {
				if (ori_ap_status != 0 || sun_ap_status != 0) {
					ori_ap_status = 0;		//switch orbital orientation mode off, regardless of current state
					sun_ap_status = 0;		//switch solar orientation mode off, regardless of current state
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					ActivateNavmode(1);
				}
				inertial_status = 1;
				Uncage();
			}
			else if (KSUl_curr_col == KSUl_R - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (!recharge && combined_power) {
					recharge = 1;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[14], 0.0);
			return true;
		}

	case KSUl_16:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[15], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_R - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (recharge) {
					recharge = 0;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[15], 0.0);
			return true;
		}

	case KSUl_17:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[16], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_P - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				orb_ori_dir = 1;
			}
			else if (KSUl_curr_col == KSUl_ZH - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				vzor_status = 0;
				if (gcEnabled()) {
					if (vzor_c_on) {
						vzor_c_on = 2;	// intermediate state to render a blank before setting to zero
						gcCustomCameraOnOff(hCam_vzor_fwd, false);
						gcCustomCameraOnOff(hCam_vzor_nad, false);
					}
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[16], 0.0);
			return true;
		}

	case KSUl_19:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[18], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_ZH - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if ((hooks_status == 2 || hooks_status == 1) && ssvp_on) hooks_status = 3;
			}
			else if (KSUl_curr_col == KSUl_K - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if (!accel_z_status) {
					accel_z_status = 1;
					accel_z = 0.0;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[18], 0.0);
			return true;
		}

	case KSUl_23:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[22], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_I - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				if(sep_status < 3) thermals_off = 1;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[22], 0.0);
			return true;
		}

	case KSUl_24:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUl_commbutt[23], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUl_curr_col == KSUl_I - KSUl_A && KSU_left_enable == 1 && KSU_right_enable == 0 && !no_power) {
				thermals_off = 0;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUl_commbutt[23], 0.0);
			return true;
		}

		// KSUp
	case KSUp_1:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[0], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_V - KSUp_A && KSU_left_enable == 0 && KSU_right_enable == 1 && !no_power) {
				if (!AKSP_status) AKSP_status = 1;
			}
			else if (KSUp_curr_col == KSUp_ZH - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if ((dock_probe_status == 0 || dock_probe_status == 3) && ssvp_on) dock_probe_status = 1;
			}
			else if (KSUp_curr_col == KSUp_I - KSUp_A && KSU_right_enable == 1 && !no_power) {
				rcs_status = 3;
			}
			else if (KSUp_curr_col == KSUp_K - KSUp_A && KSU_right_enable == 1 && !no_power) {	//cancel all K modes
				BDUS_status = BDUS_OFF;
				pulsed_status = 2;
				if (ori_ap_status != 0 || inertial_status != 0 || modes_off != 1 && !no_power) {
					ori_ap_status = 0;		//switch orbital orientation mode off, regardless of current state
					ori_ap_aux = 0;
					inertial_status = 0;	//switch inertial orientation mode off, regardless of current state
					accel_z_status = 0;		//switch off longitudinal accelerometer
					accel_z = 0;			//reset integrating accelerometer
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
				}
			}
			else if (KSUp_curr_col == KSUp_L - KSUp_A && KSU_right_enable == 1 && !no_power) {
				manual_settings = 1;
			}
			else if (KSUp_curr_col == KSUp_M - KSUp_A && KSU_right_enable == 1 && !no_power) {	//cancel all M modes
				sun_ap_status = 0;		//switch solar orientation mode off, regardless of current state
				ori_ap_status = 0;		//switch orbital orientation mode off, regardless of current state
				ori_ap_aux = 0;
				inertial_status = 0;	//switch inertial orientation mode off, regardless of current state
				accel_z_status = 0;		//switch off longitudinal accelerometer
				accel_z = 0;			//reset integrating accelerometer
				BDUS_status = BDUS_OFF;
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
				SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
				program_on = 0;
				PVU_cmd_seq = 0;
				IKP_update = 1;
			}
			else if (KSUp_curr_col == KSUp_N - KSUp_A && KSU_right_enable == 1 && !no_power) {
				backup_DO = 1;
				for (int i = 0; i < 8; i++) {
					SetThrusterResource(th_DO[i], hpr_do_res);
				}
				m_pXRSound->PlayWav(ValveBlow, false, 1.0);
			}
			else if (KSUp_curr_col == KSUp_S - KSUp_A && KSU_right_enable == 1 && !no_power) {
				ssvp_on = 1;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[0], 0.0);
			return true;
		}

	case KSUp_2:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[1], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_S - KSUp_A && KSU_right_enable == 1 && !no_power) {
				ssvp_on = 0;
			}
			else if (KSUp_curr_col == KSUp_I - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (rcs_status == 0) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					SetThrusterGroupLevel(THGROUP_ATT_UP, 0);
					SetThrusterGroupLevel(THGROUP_ATT_DOWN, 0);
					SetThrusterGroupLevel(THGROUP_ATT_LEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_FORWARD, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BACK, 0);
					SetThrusterGroupLevel(THGROUP_RETRO, 0);
					DelThrusterGroup(THGROUP_ATT_YAWLEFT, false);
					DelThrusterGroup(THGROUP_ATT_YAWRIGHT, false);
					DelThrusterGroup(THGROUP_ATT_PITCHUP, false);
					DelThrusterGroup(THGROUP_ATT_PITCHDOWN, false);
					DelThrusterGroup(THGROUP_ATT_BANKLEFT, false);
					DelThrusterGroup(THGROUP_ATT_BANKRIGHT, false);
					DelThrusterGroup(THGROUP_ATT_UP, false);
					DelThrusterGroup(THGROUP_ATT_DOWN, false);
					DelThrusterGroup(THGROUP_ATT_LEFT, false);
					DelThrusterGroup(THGROUP_ATT_RIGHT, false);
					DelThrusterGroup(THGROUP_ATT_FORWARD, false);
					DelThrusterGroup(THGROUP_ATT_BACK, false);
					DelThrusterGroup(THGROUP_RETRO, false);
					rcs_status = 6;
				}
			}
			else if (KSUp_curr_col == KSUp_L - KSUp_A && KSU_right_enable == 1 && !no_power) {
				manual_settings = 0;
				btsi_5_read = 0;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[1], 0.0);
			return true;
		}

	case KSUp_3:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[2], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_ZH - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if ((latches_status == 0 || latches_status == 3) && ssvp_on) latches_status = 1;
			}
			else if (KSUp_curr_col == KSUp_I - KSUp_A && KSU_right_enable == 1 && !no_power) {
				rcs_status = 1;
			}
			else if (KSUp_curr_col == KSUp_K - KSUp_A && KSU_right_enable == 1 && !no_power) {
				BDUS_status = BDUS_ON;
			}
			else if (KSUp_curr_col == KSUp_M - KSUp_A && KSU_right_enable == 1 && !no_power) {
				PVU_cmd_off = 1;
			}
			else if (KSUp_curr_col == KSUp_N - KSUp_A && KSU_right_enable == 1 && !no_power) {
				
			}
			else if (KSUp_curr_col == KSUp_S - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if ((latches_status == 2 || latches_status == 1) && ssvp_on) latches_status = 3;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[2], 0.0);
			return true;
		}

	case KSUp_4:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[3], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_M - KSUp_A && KSU_right_enable == 1 && !no_power) {
				PVU_cmd_off = 0;
			}
			else if (KSUp_curr_col == KSUp_I - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (rcs_status == 2) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					DelThrusterGroup(THGROUP_ATT_YAWLEFT, false);
					DelThrusterGroup(THGROUP_ATT_YAWRIGHT, false);
					DelThrusterGroup(THGROUP_ATT_PITCHUP, false);
					DelThrusterGroup(THGROUP_ATT_PITCHDOWN, false);
					DelThrusterGroup(THGROUP_ATT_BANKLEFT, false);
					DelThrusterGroup(THGROUP_ATT_BANKRIGHT, false);
					rcs_status = 6;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[3], 0.0);
			return true;
		}

	case KSUp_5:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[4], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_K - KSUp_A && KSU_right_enable == 1 && !no_power) {
				pulsed_status = 0;
			}
			else if (KSUp_curr_col == KSUp_ZH - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (igla_status > 0 && tgt_range <= 200) {
					man_appr = 1;
					IglaOff();
				}
			}
			else if (KSUp_curr_col == KSUp_P - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (!BDUS_integr) BDUS_integr = 1;
			}
			else if (KSUp_curr_col == KSUp_S - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if ((dock_probe_status == 2 || dock_probe_status == 1) && ssvp_on) dock_probe_status = 3;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[4], 0.0);
			return true;
		}

	case KSUp_6:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[5], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_ZH - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (man_appr) {
					man_appr = 0;
					IglaOff();
				}
			}
			else if (KSUp_curr_col == KSUp_P - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (BDUS_integr) BDUS_integr = 0;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[5], 0.0);
			return true;
		}

	case KSUp_7:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[6], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_ZH - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (sun_ap_status == 0 && ori_ap_status == 0 && igla_status == 0 && hAttached == NULL) {
					igla_status = 1;		//switch igla mode on
					rcs_status = 1;		  //force DO mode
				}
			}
			else if (KSUp_curr_col == KSUp_K - KSUp_A && KSU_right_enable == 1 && !no_power) {
				pulsed_status = 1;
			}
			else if (KSUp_curr_col == KSUp_M - KSUp_A && KSU_right_enable == 1 && !no_power) {
				program_on = 2;
				PVU_timer = 0;
				PVU_minute = 0;
				IKP_update = 1;
			}
			else if (KSUp_curr_col == KSUp_S - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if ((((hooks_status == 0 || hooks_status == 3) && !hMother) || (hMother && dock_probe_status == 4)) && ssvp_on) hooks_status = 1;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[6], 0.0);
			return true;
		}

	case KSUp_8:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[7], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_ZH - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (igla_status > 0) {
					IglaOff();
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[7], 0.0);
			return true;
		}

	case KSUp_9:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[8], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_K - KSUp_A && KSU_right_enable == 1) {
				if (ori_ap_status == 0 && sun_ap_status == 0 && igla_status == 0 && BDUS_status == BDUS_ON && !no_power) {
					ori_ap_status = 1;		//switch orbital orientation mode on
					rcs_status = 1;			//force DO mode
				}
			}
			else if (KSUp_curr_col == KSUp_S - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (GetDockStatus(hDock) && hooks_status == 2 && latches_status == 0 && !combined_power && ssvp_on) {
					hooks_status = 3;
					undock_commanded = 1;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[8], 0.0);
			return true;
		}

	case KSUp_10:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[9], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[9], 0.0);
			return true;
		}

	case KSUp_11:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[10], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_L - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (manual_settings) {
					btsi_5_read = 1;
					btsi_5_buffer = btsi_5_counter;
				}
			}
			else if (KSUp_curr_col == KSUp_M - KSUp_A && KSU_right_enable == 1 && !no_power) {
				program_on = 1;
				PVU_timer = 0;
				PVU_minute = 0;
				IKP_update = 1;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[10], 0.0);
			return true;
		}

	case KSUp_13:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[12], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_I - KSUp_A && KSU_right_enable == 1 && !no_power) {
				vzor_status = 1;
				if (gcEnabled()) {
					if (vzor_c_on) {
						vzor_c_on = 2;	// intermediate state to render a blank before setting to zero
						gcCustomCameraOnOff(hCam_vzor_fwd, false);
						gcCustomCameraOnOff(hCam_vzor_nad, false);
					}
				}
			}
			else if (KSUp_curr_col == KSUp_N - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (engine_status == 0 || engine_status == 3) engine_status = 1;
			}
			else if (KSUp_curr_col == KSUp_R - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (!combined_power && hMother && hooks_status == 2) combined_power = 1;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[12], 0.0);
			return true;
		}

	case KSUp_14:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[13], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_N - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (engine_status == 1 || engine_status == 2) engine_status = 3;
			}
			else if (KSUp_curr_col == KSUp_R - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (combined_power) combined_power = 0;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[13], 0.0);
			return true;
		}

	case KSUp_15:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[14], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_P - KSUp_A && KSU_right_enable == 1 && !no_power) {
				orb_ori_dir = 0;
			}
			else if (KSUp_curr_col == KSUp_M - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (ori_ap_status == 0 && sun_ap_status == 0 && igla_status == 0 && BDUS_status == BDUS_ON) {
					sun_ap_status = 1;		//switch solar orientation mode on
					rcs_status = 1;			//force DO mode
				}
			}
			else if (KSUp_curr_col == KSUp_I - KSUp_A && KSU_right_enable == 1 && BDUS_status == BDUS_ON && !no_power) {
				if (ori_ap_status != 0 || sun_ap_status != 0) {
					ori_ap_status = 0;		//switch orbital orientation mode off, regardless of current state
					sun_ap_status = 0;		//switch solar orientation mode off, regardless of current state
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					ActivateNavmode(1);
				}
				inertial_status = 1;
				Uncage();
			}
			else if (KSUp_curr_col == KSUp_R - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (!recharge && combined_power) recharge = 1;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[14], 0.0);
			return true;
		}

	case KSUp_16:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[15], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_R - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (recharge) recharge = 0;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[15], 0.0);
			return true;
		}

	case KSUp_17:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[16], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_P - KSUp_A && KSU_right_enable == 1 && !no_power) {
				orb_ori_dir = 1;
			}
			else if (KSUp_curr_col == KSUp_ZH - KSUp_A && KSU_right_enable == 1 && !no_power) {
				vzor_status = 0;
				if (gcEnabled()) {
					if (vzor_c_on) {
						vzor_c_on = 2;	// intermediate state to render a blank before setting to zero
						gcCustomCameraOnOff(hCam_vzor_fwd, false);
						gcCustomCameraOnOff(hCam_vzor_nad, false);
					}
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[16], 0.0);
			return true;
		}

	case KSUp_19:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[18], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_ZH - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if ((hooks_status == 2 || hooks_status == 1) && ssvp_on) hooks_status = 3;
			}
			else if (KSUp_curr_col == KSUp_K - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (!accel_z_status) {
					accel_z_status = 1;
					accel_z = 0.0;
				}
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[18], 0.0);
			return true;
		}

	case KSUp_23:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[22], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_I - KSUp_A && KSU_right_enable == 1 && !no_power) {
				if (sep_status < 3) thermals_off = 1;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[22], 0.0);
			return true;
		}

	case KSUp_24:
		if (event == PANEL_MOUSE_LBPRESSED) {
			if (mouse_downl == 0) {
				m_pXRSound->PlayWav(ButtonIn, false, 0.6);
				mouse_downl = 1;
			}
			SetAnimation(anim_KSUp_commbutt[23], 1.0);
			return true;
		}
		else if (event == PANEL_MOUSE_LBUP) {
			if (KSUp_curr_col == KSUp_I - KSUp_A && KSU_right_enable == 1 && !no_power) {
				thermals_off = 0;
			}
			m_pXRSound->PlayWav(ButtonOut, false, 0.6);
			mouse_downl = 0;
			SetAnimation(anim_KSUp_commbutt[23], 0.0);
			return true;
		}

	}

	return false;
}

void Soyuz7k_T::DefineAnimations(int mode) {
	
	// animation component definitions
	fold_Lantenna = new MGROUP_ROTATE(0, &GRP_LANTENNA, 1, LANTENNA_REF, LANTENNA_AXIS, ANTENNA_RANGE);
	fold_Rantenna = new MGROUP_ROTATE(0, &GRP_RANTENNA, 1, RANTENNA_REF, RANTENNA_AXIS, ANTENNA_RANGE);
	fold_iglamain_port = new MGROUP_ROTATE(2, &GRP_IGLAMAIN_PORT, 1, IGLAMAIN_PORT_REF, IGLAMAIN_PORT_AXIS, IGLAMAIN_PORT_RANGE);
	fold_iglamain_stbd = new MGROUP_ROTATE(2, &GRP_IGLAMAIN_STBD, 1, IGLAMAIN_STBD_REF, IGLAMAIN_STBD_AXIS, IGLAMAIN_STBD_RANGE);
	fold_iglamain_down = new MGROUP_ROTATE(2, grp_iglamain_fold, 25, IGLAMAIN_DOWN_REF, IGLAMAIN_DOWN_AXIS, IGLAMAIN_DOWN_RANGE);
	fold_aft_igla = new MGROUP_ROTATE(0, grp_aft_igla, 5, FOLD_AFT_IGLA_REF, FOLD_AFT_IGLA_AXIS, FOLD_AFT_IGLA_RANGE);
	fold_fwd_igla = new MGROUP_ROTATE(2, grp_fwd_igla, 14, FOLD_FWD_IGLA_REF, FOLD_FWD_IGLA_AXIS, FOLD_FWD_IGLA_RANGE);
	fold_mid_igla = new MGROUP_ROTATE(2, grp_mid_igla, 6, FOLD_MID_IGLA_REF, FOLD_MID_IGLA_AXIS, FOLD_MID_IGLA_RANGE);
	fold_igla_shield = new MGROUP_ROTATE(2, &GRP_IGLA_SHIELD, 1, FOLD_IGLA_SHIELD_REF, FOLD_IGLA_SHIELD_AXIS, FOLD_IGLA_SHIELD_RANGE);
	fold_BO_lights = new MGROUP_ROTATE(2, grp_BO_lights, 4, FOLD_BO_LIGHTS_REF, FOLD_BO_LIGHTS_AXIS, FOLD_BO_LIGHTS_RANGE);
	fold_BO_lights2 = new MGROUP_ROTATE(2, grp_BO_lights2, 4, FOLD_BO_LIGHTS2_REF, FOLD_BO_LIGHTS2_AXIS, -FOLD_BO_LIGHTS_RANGE);
	fold_ion1 = new MGROUP_ROTATE(0, grp_ion1, 3, FOLD_ION1_REF, FOLD_ION_AXIS, FOLD_ION_RANGE);
	fold_ion2 = new MGROUP_ROTATE(0, grp_ion2, 3, FOLD_ION2_REF, FOLD_ION_AXIS, FOLD_ION_RANGE);
	fold_ion3 = new MGROUP_ROTATE(0, grp_ion3, 3, FOLD_ION3_REF, FOLD_ION_AXIS, FOLD_ION_RANGE);
	fold_ion4 = new MGROUP_ROTATE(0, grp_ion4, 3, FOLD_ION4_REF, FOLD_ION_AXIS, FOLD_ION_RANGE);
	fold_rear_ant1 = new MGROUP_ROTATE(0, grp_rear_ant1, 2, FOLD_REAR_ANT1_REF, FOLD_REAR_ANT1_AXIS, ANTENNA_RANGE);
	fold_rear_ant2 = new MGROUP_ROTATE(0, grp_rear_ant2, 2, FOLD_REAR_ANT2_REF, FOLD_REAR_ANT2_AXIS, -ANTENNA_RANGE);
	igla_main_gimbal = new MGROUP_ROTATE(2, grp_iglamain_gimbal, 5, IGLA_MAIN_GIMBAL_REF, IGLA_MAIN_GIMBAL_AXIS, IGLA_MAIN_GIMBAL_RANGE);
	long_rot = new MGROUP_ROTATE(19, &grp_long, 1, LONG_REF, LONG_AXIS, TWOPI_RANGE);
	lat_rot = new MGROUP_ROTATE(19, &grp_lat, 1, LAT_REF, LAT_AXIS, TWOPI_RANGE);
	clk_sec_rot = new MGROUP_ROTATE(19, &grp_clk_sec, 1, CLK_SEC_REF, PAN_NORM_AXIS, TWOPI_RANGE);
	clk_min_rot = new MGROUP_ROTATE(19, &grp_clk_min, 1, CLK_MIN_REF, PAN_NORM_AXIS, TWOPI_RANGE);
	clk_hour_rot = new MGROUP_ROTATE(19, &grp_clk_hour, 1, CLK_HOUR_REF, PAN_NORM_AXIS, TWOPI_RANGE);
	clk_enable_rot = new MGROUP_ROTATE(19, &grp_clk_enable, 1, CLK_ENABLE_REF, IGLAMAIN_DOWN_AXIS, -FOLD_FWD_IGLA_RANGE);
	crono_sec_rot = new MGROUP_ROTATE(19, &grp_crono_sec, 1, CRONO_SEC_REF, PAN_NORM_AXIS, TWOPI_RANGE);
	crono_min_rot = new MGROUP_ROTATE(19, &grp_crono_min, 1, CRONO_MIN_REF, PAN_NORM_AXIS, TWOPI_RANGE);
	crono_hour_rot = new MGROUP_ROTATE(19, &grp_crono_hour, 1, CRONO_HOUR_REF, PAN_NORM_AXIS, TWOPI_RANGE);
	IRS_speed_rot = new MGROUP_ROTATE(19, &grp_IRS_speed, 1, IRS_SPEED_REF, PAN_NORM_AXIS, IRS_SPEED_RANGE);
	IRS_range_rot = new MGROUP_ROTATE(19, &grp_IRS_range, 1, IRS_RANGE_REF, PAN_NORM_AXIS, IRS_RANGE_RANGE);
	KSU_cyl_left_rot = new MGROUP_ROTATE(20, &grp_KSU_cyl_left, 1, CYL_LEFT_REF, CYL_ROT_AXIS, TWOPI_RANGE);
	KSU_cyl_right_rot = new MGROUP_ROTATE(20, &grp_KSU_cyl_right, 1, CYL_RIGHT_REF, CYL_ROT_AXIS, TWOPI_RANGE);
	latch1 = new MGROUP_ROTATE(2, &grp_latch1, 1, _V(0, 0.060032, 4.42097), _V(1, 0, 0), HALFPI_RANGE);
	latch2 = new MGROUP_ROTATE(2, &grp_latch2, 1, _V(0.060345, -0.000567, 4.42097), _V(0, -1, 0), HALFPI_RANGE);
	latch3 = new MGROUP_ROTATE(2, &grp_latch3, 1, _V(-0.059866, -0.000669, 4.42097), _V(0, 1, 0), HALFPI_RANGE);
	latch4 = new MGROUP_ROTATE(2, &grp_latch4, 1, _V(0.000119, -0.060179, 4.42097), _V(-1, 0, 0), HALFPI_RANGE);
	globe_o = new MGROUP_ROTATE(19, &grp_globe, 1, GLOBE_REF, _V(1, 0, 0), TWOPI_RANGE);
	globe_e = new MGROUP_ROTATE(19, &grp_globe, 1, GLOBE_REF, _V(0.61896, -0.76673, 0.170334), TWOPI_RANGE);
	e_knb = new MGROUP_ROTATE(19, &grp_globe_e_knb, 1, E_KNB_REF, -PAN_NORM_AXIS, TWOPI_RANGE);
	o_knb = new MGROUP_ROTATE(19, &grp_globe_o_knb, 1, O_KNB_REF, -PAN_NORM_AXIS, TWOPI_RANGE);
	ink_select_knb = new MGROUP_ROTATE(19, grp_ink_select, 2, INK_SELECT_KNB_REF, -PAN_NORM_AXIS, INK_SELECT_RANGE);
	ink_period_knb = new MGROUP_ROTATE(19, grp_period_select, 2, INK_PERIOD_KNB_REF, -PAN_NORM_AXIS, INK_PERIOD_SELECT_RANGE);
	int_knb = new MGROUP_ROTATE(19, grp_int_knob, 2, INT_KNB_REF, -PAN_NORM_AXIS, INT_KNOB_RANGE);
	int_needle = new MGROUP_ROTATE(19, &grp_int_needle, 1, INT_NEEDLE_REF, PAN_NORM_AXIS, INT_NEEDLE_RANGE);


	dock_probe = new MGROUP_TRANSLATE(2, grp_dock_probe, 3, DOCK_PROBE_SHIFT);
	clock_set_knob = new MGROUP_TRANSLATE(19, &grp_clock_set, 1, CLOCK_SET_SHIFT);
	trans_CM_jett = new MGROUP_TRANSLATE(19, &grp_CM_jett, 1, BUTTONS_SHIFT);
	trans_emerg_undock = new MGROUP_TRANSLATE(19, &grp_emerg_undock, 1, BUTTONS_SHIFT);
	trans_block_skdu_on = new MGROUP_TRANSLATE(19, &grp_block_skdu_on, 1, BUTTONS_SHIFT);
	trans_block_skdu_off = new MGROUP_TRANSLATE(19, &grp_block_skdu_off, 1, BUTTONS_SHIFT);
	trans_skdu_on = new MGROUP_TRANSLATE(19, &grp_skdu_on, 1, BUTTONS_SHIFT);
	trans_skdu_off = new MGROUP_TRANSLATE(19, &grp_skdu_off, 1, BUTTONS_SHIFT);
	trans_prepRB_button = new MGROUP_TRANSLATE(19, &grp_prepRB_button, 1, BUTTONS_SHIFT);
	trans_noprepRB_button = new MGROUP_TRANSLATE(19, &grp_noprepRB_button, 1, BUTTONS_SHIFT);
	trans_ELS_button = new MGROUP_TRANSLATE(19, &grp_ELS_button, 1, BUTTONS_SHIFT);
	trans_IKP_button = new MGROUP_TRANSLATE(19, &grp_IKP_button, 1, BUTTONS_SHIFT);
	trans_sound_button = new MGROUP_TRANSLATE(19, &grp_sound_button, 1, BUTTONS_SHIFT);
	trans_BB_button = new MGROUP_TRANSLATE(19, &grp_BB_button, 1, BUTTONS_SHIFT);
	trans_RB_button = new MGROUP_TRANSLATE(19, &grp_RB_button, 1, BUTTONS_SHIFT);
	trans_KSU_left_button = new MGROUP_TRANSLATE(19, &grp_KSU_left, 1, BUTTONS_SHIFT);
	trans_KSU_right_button = new MGROUP_TRANSLATE(19, &grp_KSU_right, 1, BUTTONS_SHIFT);
	trans_KSU_both_button = new MGROUP_TRANSLATE(19, &grp_KSU_both, 1, BUTTONS_SHIFT);
	trans_KSU_off_button = new MGROUP_TRANSLATE(19, &grp_KSU_off, 1, BUTTONS_SHIFT);

	KSUl_A_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_A, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_B_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_B, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_V_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_V, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_G_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_G, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_D_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_D, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_E_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_E, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_ZH_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_ZH, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_I_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_I, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_K_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_K, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_L_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_L, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_M_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_M, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_N_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_N, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_P_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_P, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_R_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_R, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_S_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_S, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUl_T_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_T, 1, KSUl_SYS_BUTTONS_SHIFT);

	KSUp_A_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_A, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_B_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_B, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_V_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_V, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_G_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_G, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_D_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_D, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_E_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_E, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_ZH_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_ZH, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_I_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_I, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_K_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_K, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_L_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_L, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_M_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_M, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_N_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_N, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_P_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_P, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_R_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_R, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_S_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_S, 1, KSUl_SYS_BUTTONS_SHIFT);
	KSUp_T_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_T, 1, KSUl_SYS_BUTTONS_SHIFT);

	KSUl_1_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_1, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_2_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_2, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_3_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_3, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_4_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_4, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_5_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_5, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_6_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_6, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_7_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_7, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_8_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_8, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_9_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_9, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_10_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_10, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_11_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_11, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_12_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_12, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_13_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_13, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_14_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_14, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_15_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_15, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_16_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_16, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_17_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_17, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_18_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_18, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_19_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_19, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_20_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_20, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_21_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_21, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_22_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_22, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_23_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_23, 1, KSUl_MISC_BUTTONS_SHIFT);
	KSUl_24_tr = new MGROUP_TRANSLATE(20, &grp_KSUl_24, 1, KSUl_MISC_BUTTONS_SHIFT);

	KSUp_1_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_1, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_2_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_2, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_3_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_3, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_4_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_4, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_5_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_5, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_6_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_6, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_7_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_7, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_8_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_8, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_9_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_9, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_10_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_10, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_11_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_11, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_12_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_12, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_13_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_13, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_14_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_14, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_15_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_15, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_16_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_16, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_17_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_17, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_18_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_18, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_19_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_19, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_20_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_20, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_21_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_21, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_22_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_22, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_23_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_23, 1, KSUp_MISC_BUTTONS_SHIFT);
	KSUp_24_tr = new MGROUP_TRANSLATE(20, &grp_KSUp_24, 1, KSUp_MISC_BUTTONS_SHIFT);
	
	BTSI_knb_5_tr = new MGROUP_TRANSLATE(19, grp_BTSI_knb_5, 2, BTSI_KNB_SHIFT);

	dock_probe_spring = new MGROUP_SCALE(2, &GRP_DOCK_PROBE_SPRING, 1, DOCK_PROBE_SPRING_REF, DOCK_PROBE_SPRING_SCALE);
	main_chute_scale = new MGROUP_SCALE(17, grp_main_chute, 2, MAIN_CHUTE_REF, MAIN_CHUTE_SCALE);
	drogue_chute_scale = new MGROUP_SCALE(17, grp_drogue_chute, 3, DROGUE_CHUTE_REF, DROGUE_CHUTE_SCALE);

	ANIMATIONCOMPONENT_HANDLE parent;

	if (mode == 0) {
		anim_antennae = CreateAnimation(1.0);
		AddAnimationComponent(anim_antennae, 0, 1, fold_Lantenna);
		AddAnimationComponent(anim_antennae, 0, 1, fold_Rantenna);
		AddAnimationComponent(anim_antennae, 0, 1, fold_iglamain_down);
		AddAnimationComponent(anim_antennae, 0, 1, fold_aft_igla);
		AddAnimationComponent(anim_antennae, 0, 1, fold_fwd_igla);
		AddAnimationComponent(anim_antennae, 0, 1, fold_mid_igla);
		AddAnimationComponent(anim_antennae, 0, 1, fold_igla_shield);
		AddAnimationComponent(anim_antennae, 0, 1, fold_BO_lights);
		AddAnimationComponent(anim_antennae, 0, 1, fold_BO_lights2);
		AddAnimationComponent(anim_antennae, 0, 1, fold_rear_ant1);
		AddAnimationComponent(anim_antennae, 0, 1, fold_rear_ant2);
		
		anim_igla_gimbal = CreateAnimation(0.0);
		AddAnimationComponent(anim_igla_gimbal, 0, 1, igla_main_gimbal);

		anim_ion = CreateAnimation(1.0);
		AddAnimationComponent(anim_ion, 0, 1, fold_ion1);
		AddAnimationComponent(anim_ion, 0, 1, fold_ion2);
		AddAnimationComponent(anim_ion, 0, 1, fold_ion3);
		AddAnimationComponent(anim_ion, 0, 1, fold_ion4);

		anim_igla_sides = CreateAnimation(1.0);
		AddAnimationComponent(anim_igla_sides, 0, 1, fold_iglamain_port);
		AddAnimationComponent(anim_igla_sides, 0, 1, fold_iglamain_stbd);

		anim_dock_probe = CreateAnimation(1.0);
		parent = AddAnimationComponent(anim_dock_probe, 0, 1, dock_probe);
		AddAnimationComponent(anim_dock_probe, 0, 1, dock_probe_spring);

		anim_latches = CreateAnimation(0.0);
		AddAnimationComponent(anim_latches, 0, 1, latch1, parent);
		AddAnimationComponent(anim_latches, 0, 1, latch2, parent);
		AddAnimationComponent(anim_latches, 0, 1, latch3, parent);
		AddAnimationComponent(anim_latches, 0, 1, latch4, parent);

		anim_BTSI_knb_5 = CreateAnimation(0.0);
		AddAnimationComponent(anim_BTSI_knb_5, 0, 1, BTSI_knb_5_tr);

		anim_ink_select_knb = CreateAnimation(0.0);
		AddAnimationComponent(anim_ink_select_knb, 0, 1, ink_select_knb);

		anim_ink_period_knb = CreateAnimation(0.0);
		AddAnimationComponent(anim_ink_period_knb, 0, 1, ink_period_knb);

		anim_globe_o = CreateAnimation(0.0);
		AddAnimationComponent(anim_globe_o, 0, 1, globe_o);

		anim_globe_e = CreateAnimation(0.0);
		AddAnimationComponent(anim_globe_e, 0, 1, globe_e);

		anim_o_knb = CreateAnimation(0.0);
		AddAnimationComponent(anim_o_knb, 0, 1, o_knb);

		anim_e_knb = CreateAnimation(0.0);
		AddAnimationComponent(anim_e_knb, 0, 1, e_knb);

		anim_long = CreateAnimation(0.0);
		AddAnimationComponent(anim_long, 0, 1, long_rot);

		anim_lat = CreateAnimation(0.0);
		AddAnimationComponent(anim_lat, 0, 1, lat_rot);

		anim_clk_sec = CreateAnimation(0.0);
		AddAnimationComponent(anim_clk_sec, 0, 1, clk_sec_rot);

		anim_clk_min = CreateAnimation(0.0);
		AddAnimationComponent(anim_clk_min, 0, 1, clk_min_rot);

		anim_clk_hour = CreateAnimation(0.0);
		AddAnimationComponent(anim_clk_hour, 0, 1, clk_hour_rot);

		anim_clk_enable = CreateAnimation(0.0);
		AddAnimationComponent(anim_clk_enable, 0, 1, clk_enable_rot);

		anim_crono_sec = CreateAnimation(0.0);
		AddAnimationComponent(anim_crono_sec, 0, 1, crono_sec_rot);

		anim_crono_min = CreateAnimation(0.0);
		AddAnimationComponent(anim_crono_min, 0, 1, crono_min_rot);

		anim_crono_hour = CreateAnimation(0.0);
		AddAnimationComponent(anim_crono_hour, 0, 1, crono_hour_rot);

		anim_clock_set = CreateAnimation(0.0);
		AddAnimationComponent(anim_clock_set, 0, 1, clock_set_knob);

		anim_CM_jett = CreateAnimation(0.0);
		AddAnimationComponent(anim_CM_jett, 0, 1, trans_CM_jett);

		anim_emerg_undock = CreateAnimation(0.0);
		AddAnimationComponent(anim_emerg_undock, 0, 1, trans_emerg_undock);

		anim_block_skdu_on = CreateAnimation(0.0);
		AddAnimationComponent(anim_block_skdu_on, 0, 1, trans_block_skdu_on);

		anim_block_skdu_off = CreateAnimation(0.0);
		AddAnimationComponent(anim_block_skdu_off, 0, 1, trans_block_skdu_off);

		anim_prepRB_button = CreateAnimation(0.0);
		AddAnimationComponent(anim_prepRB_button, 0, 1, trans_prepRB_button);

		anim_noprepRB_button = CreateAnimation(0.0);
		AddAnimationComponent(anim_noprepRB_button, 0, 1, trans_noprepRB_button);

		anim_ELS_button = CreateAnimation(0.0);
		AddAnimationComponent(anim_ELS_button, 0, 1, trans_ELS_button);

		anim_IKP_button = CreateAnimation(0.0);
		AddAnimationComponent(anim_IKP_button, 0, 1, trans_IKP_button);

		anim_sound_button = CreateAnimation(0.0);
		AddAnimationComponent(anim_sound_button, 0, 1, trans_sound_button);

		anim_BB_button = CreateAnimation(0.0);
		AddAnimationComponent(anim_BB_button, 0, 1, trans_BB_button);

		anim_RB_button = CreateAnimation(0.0);
		AddAnimationComponent(anim_RB_button, 0, 1, trans_RB_button);

		anim_skdu_on = CreateAnimation(0.0);
		AddAnimationComponent(anim_skdu_on, 0, 1, trans_skdu_on);

		anim_skdu_off = CreateAnimation(0.0);
		AddAnimationComponent(anim_skdu_off, 0, 1, trans_skdu_off);

		anim_IRS_speed = CreateAnimation(0.0);
		AddAnimationComponent(anim_IRS_speed, 0, 1, IRS_speed_rot);

		anim_IRS_range = CreateAnimation(0.0);
		AddAnimationComponent(anim_IRS_range, 0, 1, IRS_range_rot);

		anim_INT_knob = CreateAnimation(0.0);
		AddAnimationComponent(anim_INT_knob, 0, 1, int_knb);

		anim_INT_needle = CreateAnimation(0.0);
		AddAnimationComponent(anim_INT_needle, 0, 1, int_needle);

		anim_KSU_left_button = CreateAnimation(0.0);
		AddAnimationComponent(anim_KSU_left_button, 0, 1, trans_KSU_left_button);

		anim_KSU_right_button = CreateAnimation(0.0);
		AddAnimationComponent(anim_KSU_right_button, 0, 1, trans_KSU_right_button);

		anim_KSU_both_button = CreateAnimation(0.0);
		AddAnimationComponent(anim_KSU_both_button, 0, 1, trans_KSU_both_button);

		anim_KSU_off_button = CreateAnimation(0.0);
		AddAnimationComponent(anim_KSU_off_button, 0, 1, trans_KSU_off_button);

		anim_KSUl_cyl = CreateAnimation(0.0);
		AddAnimationComponent(anim_KSUl_cyl, 0, 1, KSU_cyl_left_rot);

		anim_KSUp_cyl = CreateAnimation(0.0);
		AddAnimationComponent(anim_KSUp_cyl, 0, 1, KSU_cyl_right_rot);

		for (int i = 0; i < 16; i++) {
			anim_KSUl_sysbutt[i] = CreateAnimation(0.0);
			anim_KSUp_sysbutt[i] = CreateAnimation(0.0);
		}
		AddAnimationComponent(anim_KSUl_sysbutt[0], 0, 1, KSUl_A_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[1], 0, 1, KSUl_B_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[2], 0, 1, KSUl_V_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[3], 0, 1, KSUl_G_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[4], 0, 1, KSUl_D_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[5], 0, 1, KSUl_E_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[6], 0, 1, KSUl_ZH_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[7], 0, 1, KSUl_I_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[8], 0, 1, KSUl_K_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[9], 0, 1, KSUl_L_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[10], 0, 1, KSUl_M_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[11], 0, 1, KSUl_N_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[12], 0, 1, KSUl_P_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[13], 0, 1, KSUl_R_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[14], 0, 1, KSUl_S_tr);
		AddAnimationComponent(anim_KSUl_sysbutt[15], 0, 1, KSUl_T_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[0], 0, 1, KSUp_A_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[1], 0, 1, KSUp_B_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[2], 0, 1, KSUp_V_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[3], 0, 1, KSUp_G_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[4], 0, 1, KSUp_D_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[5], 0, 1, KSUp_E_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[6], 0, 1, KSUp_ZH_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[7], 0, 1, KSUp_I_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[8], 0, 1, KSUp_K_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[9], 0, 1, KSUp_L_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[10], 0, 1, KSUp_M_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[11], 0, 1, KSUp_N_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[12], 0, 1, KSUp_P_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[13], 0, 1, KSUp_R_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[14], 0, 1, KSUp_S_tr);
		AddAnimationComponent(anim_KSUp_sysbutt[15], 0, 1, KSUp_T_tr);

		for (int i = 0; i < 24; i++) {
			anim_KSUl_commbutt[i] = CreateAnimation(0.0);
			anim_KSUp_commbutt[i] = CreateAnimation(0.0);
		}
		AddAnimationComponent(anim_KSUl_commbutt[0], 0, 1, KSUl_1_tr);
		AddAnimationComponent(anim_KSUl_commbutt[1], 0, 1, KSUl_2_tr);
		AddAnimationComponent(anim_KSUl_commbutt[2], 0, 1, KSUl_3_tr);
		AddAnimationComponent(anim_KSUl_commbutt[3], 0, 1, KSUl_4_tr);
		AddAnimationComponent(anim_KSUl_commbutt[4], 0, 1, KSUl_5_tr);
		AddAnimationComponent(anim_KSUl_commbutt[5], 0, 1, KSUl_6_tr);
		AddAnimationComponent(anim_KSUl_commbutt[6], 0, 1, KSUl_7_tr);
		AddAnimationComponent(anim_KSUl_commbutt[7], 0, 1, KSUl_8_tr);
		AddAnimationComponent(anim_KSUl_commbutt[8], 0, 1, KSUl_9_tr);
		AddAnimationComponent(anim_KSUl_commbutt[9], 0, 1, KSUl_10_tr);
		AddAnimationComponent(anim_KSUl_commbutt[10], 0, 1, KSUl_11_tr);
		AddAnimationComponent(anim_KSUl_commbutt[11], 0, 1, KSUl_12_tr);
		AddAnimationComponent(anim_KSUl_commbutt[12], 0, 1, KSUl_13_tr);
		AddAnimationComponent(anim_KSUl_commbutt[13], 0, 1, KSUl_14_tr);
		AddAnimationComponent(anim_KSUl_commbutt[14], 0, 1, KSUl_15_tr);
		AddAnimationComponent(anim_KSUl_commbutt[15], 0, 1, KSUl_16_tr);
		AddAnimationComponent(anim_KSUl_commbutt[16], 0, 1, KSUl_17_tr);
		AddAnimationComponent(anim_KSUl_commbutt[17], 0, 1, KSUl_18_tr);
		AddAnimationComponent(anim_KSUl_commbutt[18], 0, 1, KSUl_19_tr);
		AddAnimationComponent(anim_KSUl_commbutt[19], 0, 1, KSUl_20_tr);
		AddAnimationComponent(anim_KSUl_commbutt[20], 0, 1, KSUl_21_tr);
		AddAnimationComponent(anim_KSUl_commbutt[21], 0, 1, KSUl_22_tr);
		AddAnimationComponent(anim_KSUl_commbutt[22], 0, 1, KSUl_23_tr);
		AddAnimationComponent(anim_KSUl_commbutt[23], 0, 1, KSUl_24_tr);
		AddAnimationComponent(anim_KSUp_commbutt[0], 0, 1, KSUp_1_tr);
		AddAnimationComponent(anim_KSUp_commbutt[1], 0, 1, KSUp_2_tr);
		AddAnimationComponent(anim_KSUp_commbutt[2], 0, 1, KSUp_3_tr);
		AddAnimationComponent(anim_KSUp_commbutt[3], 0, 1, KSUp_4_tr);
		AddAnimationComponent(anim_KSUp_commbutt[4], 0, 1, KSUp_5_tr);
		AddAnimationComponent(anim_KSUp_commbutt[5], 0, 1, KSUp_6_tr);
		AddAnimationComponent(anim_KSUp_commbutt[6], 0, 1, KSUp_7_tr);
		AddAnimationComponent(anim_KSUp_commbutt[7], 0, 1, KSUp_8_tr);
		AddAnimationComponent(anim_KSUp_commbutt[8], 0, 1, KSUp_9_tr);
		AddAnimationComponent(anim_KSUp_commbutt[9], 0, 1, KSUp_10_tr);
		AddAnimationComponent(anim_KSUp_commbutt[10], 0, 1, KSUp_11_tr);
		AddAnimationComponent(anim_KSUp_commbutt[11], 0, 1, KSUp_12_tr);
		AddAnimationComponent(anim_KSUp_commbutt[12], 0, 1, KSUp_13_tr);
		AddAnimationComponent(anim_KSUp_commbutt[13], 0, 1, KSUp_14_tr);
		AddAnimationComponent(anim_KSUp_commbutt[14], 0, 1, KSUp_15_tr);
		AddAnimationComponent(anim_KSUp_commbutt[15], 0, 1, KSUp_16_tr);
		AddAnimationComponent(anim_KSUp_commbutt[16], 0, 1, KSUp_17_tr);
		AddAnimationComponent(anim_KSUp_commbutt[17], 0, 1, KSUp_18_tr);
		AddAnimationComponent(anim_KSUp_commbutt[18], 0, 1, KSUp_19_tr);
		AddAnimationComponent(anim_KSUp_commbutt[19], 0, 1, KSUp_20_tr);
		AddAnimationComponent(anim_KSUp_commbutt[20], 0, 1, KSUp_21_tr);
		AddAnimationComponent(anim_KSUp_commbutt[21], 0, 1, KSUp_22_tr);
		AddAnimationComponent(anim_KSUp_commbutt[22], 0, 1, KSUp_23_tr);
		AddAnimationComponent(anim_KSUp_commbutt[23], 0, 1, KSUp_24_tr);

	}
	else if (mode == 1) {

		main_chute_deploy = CreateAnimation(0.0);
		AddAnimationComponent(main_chute_deploy, 0, 1, main_chute_scale);

		drogue_chute_deploy = CreateAnimation(0.0);
		AddAnimationComponent(drogue_chute_deploy, 0, 1, drogue_chute_scale);
	}
}

void Soyuz7k_T::GetAngularVel() {

	if (BDUS_status == BDUS_ON) {
		VESSEL3::GetAngularVel(avel);

		avel.x = avel.x * DEG;
		avel.y = avel.y * DEG;
		avel.z = avel.z * DEG;
	}
	else {
		avel.x = 0;
		avel.y = 0;
		avel.z = 0;
	}
}

void Soyuz7k_T::GetAngularVelSUS() {
	
	VECTOR3 avelTemp;
	VESSEL3::GetAngularVel(avelTemp);

	double rotAmount = -21.0 * RAD;
	avel.x = avelTemp.x;
	avel.y = avelTemp.y * cos(rotAmount) - avelTemp.z * sin(rotAmount);
	avel.z = avelTemp.y * sin(rotAmount) + avelTemp.z * cos(rotAmount);

	avel.x = avel.x * DEG;
	avel.y = avel.y * DEG;
	avel.z = avel.z * DEG;

	/*
	if (avel.x > 18.0) avel.x = 18.0;
	else if (avel.x < -18.0) avel.x = -18.0;
	if (avel.y > 18.0) avel.y = 18.0;
	else if (avel.y < -18.0) avel.y = -18.0;
	if (avel.z > 18.0) avel.z = 18.0;
	else if (avel.z < -18.0) avel.z = -18.0;
	*/
}

void Soyuz7k_T::VzorUpdate(SURFHANDLE surf, int screen) {

	if (hSurfVzor && hSurfVzor2 && hSurfVzor3 && oapiCockpitMode() == COCKPIT_VIRTUAL && gcEnabled()) {
		if (screen == 0) {
			//if (vzor_status == 3 || vzor_status == 1) oapiBlt(surf, hSurfVzor, 0, 0, 0, 0, 1023, 1023);
			if (vzor_status == 3) oapiBlt(surf, hSurfVzor, 0, 0, 0, 0, 1023, 1023);
			//else if (vzor_status == 2 || vzor_status == 0) oapiBlt(surf, hSurfVzor2, 0, 0, 0, 0, 1023, 1023);
			else oapiBlt(surf, hSurfVzor2, 0, 0, 0, 0, 1023, 1023);
		}
		else oapiBlt(surf, hSurfVzor3, 0, 0, 0, 0, 1023, 1023);
	}
}

// ==============================================================
// Overloaded callback functions
// ==============================================================

// --------------------------------------------------------------
// Set the capabilities of the vessel class
// --------------------------------------------------------------
void Soyuz7k_T::clbkSetClassCaps (FILEHANDLE cfg)
{

	// physical vessel parameters
	SetSize (S7K_SIZE);
	SetEmptyMass (S7K_EMPTYMASS);
	SetPMI (S7K_PMI);
	SetCrossSections (S7K_CS);
	SetRotDrag (S7K_RD);
	SetTouchdownPoints (tdvtx, ntdvtx);

	// docking port definitions
	hDock = CreateDock(S7K_DOCK_POS, S7K_DOCK_DIR, S7K_DOCK_ROT);

	// attachment point for launch vehicle
	attach1 = CreateAttachment(true, _V(0, 0, -3.5), _V(0, 0, -1), _V(1, 0, 0), "R7");

	// define light sources
	beac1_pos = _V(-0.28, -1.26, 1.92);
	beac2_pos = _V(-0.28, -1.36, 1.92);
	beac3_pos = _V(-0.20055, 1.15153, -1.09022);
	beac4_pos = _V(0.204819, -1.14812, -1.09022);
	beac5_pos = _V(0, 0.75, 0.4);
	beac6_pos = _V(-0.282019, 1.29958, 1.92);
	
	beac1.shape = BEACONSHAPE_STAR;
	beac1.pos = &beac1_pos;
	beac1.col = &bscol;
	beac1.size = 0.07;
	beac1.falloff = 0.6;
	beac1.period = 0.7;
	beac1.duration = 0.35;
	beac1.tofs = 0;
	beac1.active = false;
	AddBeacon(&beac1);

	beac2.shape = BEACONSHAPE_STAR;
	beac2.pos = &beac2_pos;
	beac2.col = &bscol;
	beac2.size = 0.07;
	beac2.falloff = 0.6;
	beac2.period = 0;
	beac2.duration = 0.35;
	beac2.tofs = 0;
	beac2.active = false;
	AddBeacon(&beac2);

	beac3.shape = BEACONSHAPE_STAR;
	beac3.pos = &beac3_pos;
	beac3.col = &bscol;
	beac3.size = 0.2;
	beac3.falloff = 0.6;
	beac3.period = 0.7;
	beac3.duration = 0.35;
	beac3.tofs = 0;
	beac3.active = false;
	AddBeacon(&beac3);

	beac4.shape = BEACONSHAPE_STAR;
	beac4.pos = &beac4_pos;
	beac4.col = &bscol;
	beac4.size = 0.2;
	beac4.falloff = 0.6;
	beac4.period = 0.7;
	beac4.duration = 0.35;
	beac4.tofs = 0;
	beac4.active = false;
	AddBeacon(&beac4);

	beac6.shape = BEACONSHAPE_STAR;
	beac6.pos = &beac6_pos;
	beac6.col = &bscol;
	beac6.size = 0.07;
	beac6.falloff = 0.6;
	beac6.period = 0;
	beac6.duration = 0.35;
	beac6.tofs = 0;
	beac6.active = false;
	AddBeacon(&beac6);

	COLOUR4 col_d = { 1,1,1,0 };
	COLOUR4 col_s = { 1,1,1,0 };
	COLOUR4 col_a = { 1,1,1,0 };
	spotlight1 = (SpotLight*)AddSpotLight(beac2_pos, _V(0, 0, 1), 35, 0.1, 0, 0.1, 45*RAD, 100*RAD, col_d, col_s, col_a);
	spotlight1->Activate(false);
	spotlight2 = (SpotLight*)AddSpotLight(beac6_pos, _V(0, 0, 1), 35, 0.1, 0, 0.1, 45 * RAD, 100 * RAD, col_d, col_s, col_a);
	spotlight2->Activate(false);

	VClight_status = 1;
	col_d = { 1,0.91,0.81,0 };
	col_s = { 1,0.98,0.91,0 };
	col_a = { 1,0.98,0.91,0 };
	spotlightVC = AddSpotLight(beac5_pos, _V(0, -1, 0), 1.5, 0.3, 0, 0.3, 0.1 * RAD, 120 * RAD, col_d, col_s, col_a);
	spotlightVC->SetPosition(_V(0, 0.75, 0.4));
	spotlightVC->Activate(VClight_status);
	spotlightVC->SetVisibility(LightEmitter::VIS_COCKPIT);
	

	// airfoil definitions
	airfoil_v = CreateAirfoil2(LIFT_VERTICAL, _V(0, -0.12, 0), vliftNull, 2.17, 6.82, 0.1); 
	airfoil_h = CreateAirfoil2(LIFT_HORIZONTAL, _V(0, 0, 0), hliftNull, 2.17, 6.82, 0.1); 

	surf_dynamic_first = NULL;
	surf_dynamic_second = NULL;
	dyn_tex_first = NULL;
	dyn_tex_second = NULL;

	block_skdu_on = block_skdu_off = command_skdu_on = command_skdu_off = 0;

	last_MJD_clock = 0;
	last_MJD_crono = 0;
	clock_enable = 2;
	crono_enable = 0;
	crono_digit_changed = 1;

	crono_sec_proc = 0;
	crono_min_proc = 0;
	crono_hour_proc = 0;
	clk_sec_proc = 0;
	clk_min_proc = 0;
	clk_hour_proc = 0;

	sep_status = 0;
	skin_status = 0;
	timer = 0;

	// counters initial values
	ink_period = 90.00;
	dv_accel = 0.0;

	antennae_status = 2;
	antennae_proc = 1;

	igla_sides_status = 2;
	igla_sides_proc = 1;
	
	main_chute_status = 0;
	main_chute_proc = 0;

	pulsed_status = 0;
	engine_status = 3;

	igla_status = 0;
	sun_ap_status = 0;
	ori_ap_status = 0;
	ori_ap_aux = 0;
	ori_ap_roll = 2;

	igla_ap_aux_pitch = 0;
	igla_ap_aux_yaw = 0;
	igla_ap_aux_roll = 0;
	igla_ap_aux_vert = 0;
	igla_ap_aux_lat = 0;
	igla_ap_pitch = 0;
	igla_ap_yaw = 0;
	igla_ap_roll = 0;
	igla_ap_vert = 0;
	igla_ap_lat = 0;

	trans_lateral_flag = 0;
	trans_vertical_flag = 0;

	target_oriented = 0;

	relvel_sample = 0;
	acc_SimDT = 0;

	vzor_status = 0;
	vzor_c_on = 1;
	vzor_p_on = 1;

	burn_seq = 0;
	igla_idle = 1;

	INT_status = 0;
	BTSI_knb_5_digit = 0;

	base_power = BASE_POWER_DRAIN;

	for (int i = 0; i < 6; i++) {
		rotation_command[i] = 0;
	}

	// propellant resources
	hpr = CreatePropellantResource(S7K_FUELMASS);
	hpr_dpo = CreatePropellantResource(S7K_FUELDPOMASS);
	hpr_do = CreatePropellantResource(S7K_FUELDOMASS);
	hpr_do_res = CreatePropellantResource(S7K_FUELDORESMASS);

	// retro engines
	th_retro[0] = CreateThruster(_V(-1.133,0,-0.9), _V(0.226,0,-0.974), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	th_retro[1] = CreateThruster(_V(1.133, 0, -0.9), _V(-0.226, 0, -0.974), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	//CreateThrusterGroup (th_retro, 2, THGROUP_RETRO);
	AddExhaustStream(th_retro[0], &DPO_exhaust);
	AddExhaustStream(th_retro[1], &DPO_exhaust);

	// RCS engines
	th_trans_fwd[0] = CreateThruster(_V(-1.163, 0, -3.5), _V(0, 0, 1), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	th_trans_fwd[1] = CreateThruster(_V(1.163, 0, -3.5), _V(0, 0, 1), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	
	
	// DKD thrusters
	//th_trans_fwd[0] = CreateThruster(_V(0, 0.38, -3.5), _V(0, 1, 1), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	//th_trans_fwd[1] = CreateThruster(_V(0, -0.38, -3.5), _V(0, -1, 1), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	//th_trans_fwd[2] = CreateThruster(_V(-0.385, 0, -3.5), _V(-1, 0, 1), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	//th_trans_fwd[3] = CreateThruster(_V(0.385, 0, -3.5), _V(1, 0, 1), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);


	for (int i = 0; i < 2; i++) {
		AddExhaustStream(th_trans_fwd[i], &DPO_exhaust);
	}
	
	th_DPO[0] = CreateThruster(_V(0, 1.2, -1.15), _V(0, -1, 0), S7K_MAXDPOTH_2, hpr_dpo, S7K_SIOISP);
	th_DPO[1] = CreateThruster(_V(0, 1.15, -2.94), _V(0, -1, 0), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	th_DPO[2] = CreateThruster(_V(0, -1.15, -2.94), _V(0, 1, 0), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	th_DPO[3] = CreateThruster(_V(0, -1.2, -1.15), _V(0, 1, 0), S7K_MAXDPOTH_2, hpr_dpo, S7K_SIOISP);
	th_DPO[4] = CreateThruster(_V(0, -1.2, -1.15), _V(0, 1, 0), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	th_DPO[5] = CreateThruster(_V(0, 1.2, -1.15), _V(0, -1, 0), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	th_DPO[6] = CreateThruster(_V(-1.046, 0.578, -1.11), _V(1, -1, 0), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	th_DPO[7] = CreateThruster(_V(-1.046, -0.578, -1.11), _V(1, 1, 0), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	th_DPO[8] = CreateThruster(_V(1.046, 0.578, -1.11), _V(-1, -1, 0), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);
	th_DPO[9] = CreateThruster(_V(1.046, -0.578, -1.11), _V(-1, 1, 0), S7K_MAXDPOTH, hpr_dpo, S7K_SIOISP);

	for (int i = 0; i < 10; i++) {
		AddExhaustStream(th_DPO[i], &DPO_exhaust);
	}

	//pitch
	th_DO[0] = CreateThruster(_V(0, 1.134, -3.18), _V(0, -1, 0), S7K_MAXDOTH, hpr_do, S7K_SIOISP);
	th_DO[1] = CreateThruster(_V(0, -1.134, -3.18), _V(0, 1, 0), S7K_MAXDOTH, hpr_do, S7K_SIOISP);
	//begin rolls
	th_DO[2] = CreateThruster(_V(-0.9, 0.81, -3.18), _V(1, 1, 0), S7K_MAXDOTH, hpr_do, S7K_SIOISP);
	th_DO[3] = CreateThruster(_V(0.9, -0.81, -3.18), _V(-1, -1, 0), S7K_MAXDOTH, hpr_do, S7K_SIOISP);
	th_DO[4] = CreateThruster(_V(0.9, 0.81, -3.18), _V(-1, 1, 0), S7K_MAXDOTH, hpr_do, S7K_SIOISP);
	th_DO[5] = CreateThruster(_V(-0.9, -0.81, -3.18), _V(1, -1, 0), S7K_MAXDOTH, hpr_do, S7K_SIOISP);
	//end rolls
	//yaw
	th_DO[6] = CreateThruster(_V(-1.134, 0, -3.18), _V(1, 0, 0), S7K_MAXDOTH, hpr_do, S7K_SIOISP);
	th_DO[7] = CreateThruster(_V(1.134, 0, -3.18), _V(-1, 0, 0), S7K_MAXDOTH, hpr_do, S7K_SIOISP);

	for (int i = 0; i < 8; i++) {
		AddExhaustStream(th_DO[i], &DO_exhaust);
	}

	
	//translation correction thrusters
	th_correct[0] = CreateThruster(_V(1.163, 0, -3.5), _V(0, 0, 1), S7K_MAXDPOTH * side_null_fact, hpr_dpo, S7K_SIOISP);
	th_correct[1] = CreateThruster(_V(-1.133, 0, -0.9), _V(0.226, 0, -0.974), S7K_MAXDPOTH * side_null_fact, hpr_dpo, S7K_SIOISP);
	th_correct[2] = CreateThruster(_V(-1.163, 0, -3.5), _V(0, 0, 1), S7K_MAXDPOTH * side_null_fact, hpr_dpo, S7K_SIOISP);
	th_correct[3] = CreateThruster(_V(1.133, 0, -0.9), _V(-0.226, 0, -0.974), S7K_MAXDPOTH * side_null_fact, hpr_dpo, S7K_SIOISP);
	th_correct[4] = CreateThruster(_V(0, 1.15, -2.94), _V(0, -1, 0), S7K_MAXDPOTH * vert_null_fact, hpr_dpo, S7K_SIOISP);
	th_correct[5] = CreateThruster(_V(0, -1.15, -2.94), _V(0, 1, 0), S7K_MAXDPOTH * vert_null_fact, hpr_dpo, S7K_SIOISP);

	for (int i = 0; i < 6; i++) {
		AddExhaustStream(th_correct[i], &DPO_exhaust);
	}

	//define static translation groups
	/*th_group[0] = th_DPO[3];
	th_group[1] = th_correct[4];
	CreateThrusterGroup(th_group, 2, THGROUP_ATT_UP);

	th_group[0] = th_DPO[0];
	th_group[1] = th_correct[5];
	CreateThrusterGroup(th_group, 2, THGROUP_ATT_DOWN);

	th_group[0] = th_DPO[8];
	th_group[1] = th_DPO[9];
	th_group[2] = th_correct[0];
	th_group[3] = th_correct[1];
	//th_group[4] = th_DO[6];
	CreateThrusterGroup(th_group, 4, THGROUP_ATT_LEFT);

	th_group[0] = th_DPO[6];
	th_group[1] = th_DPO[7];
	th_group[2] = th_correct[2];
	th_group[3] = th_correct[3];
	//th_group[4] = th_DO[7];
	CreateThrusterGroup(th_group, 4, THGROUP_ATT_RIGHT);*/

	//CreateThrusterGroup(th_trans_fwd, 2, THGROUP_ATT_FORWARD);

	//CreateThrusterGroup(th_retro, 2, THGROUP_ATT_BACK);

	th_main[0] = CreateThruster(_V(0, 0, -3.5), _V(0, 0, 1), S7K_MAXMAINTH, hpr, S7K_ISP);
	th_main[1] = CreateThruster(_V(0, 0, -3.5), _V(0, 0, 1), S7K_MAXBACKUPTH, hpr, S7K_ISP_BACKUP);

	AddExhaustStream(th_main[0], &main_exhaust);
	AddExhaustStream(th_main[1], &main_exhaust);

	// camera parameters
	SetCameraOffset (_V(0.277,-1.48,0.5));
	SetCameraRotationRange(0, 0, 0, 0);
	
	// associate a mesh for the visual

	hMesh[0] = oapiLoadMeshGlobal("Soyuz_7k\\T_PAO");
	hMesh[1] = oapiLoadMeshGlobal("Soyuz_7k\\OK_SA_body");
	hMesh[2] = oapiLoadMeshGlobal("Soyuz_7k\\T_BO");
	hMesh[3] = oapiLoadMeshGlobal("Soyuz_7k\\OK_cabling");
	hMesh[4] = oapiLoadMeshGlobal("Soyuz_7k\\OK_thermal1");
	hMesh[5] = oapiLoadMeshGlobal("Soyuz_7k\\OK_thermal2");
	hMesh[6] = oapiLoadMeshGlobal("Soyuz_7k\\OK_thermal3");
	hMesh[7] = oapiLoadMeshGlobal("Soyuz_7k\\OK_thermal4");
	hMesh[8] = oapiLoadMeshGlobal("Soyuz_7k\\OK_thermal5");
	hMesh[9] = oapiLoadMeshGlobal("Soyuz_7k\\OK_thermal6");
	hMesh[10] = oapiLoadMeshGlobal("Soyuz_7k\\OK_thermal7");
	hMesh[11] = oapiLoadMeshGlobal("Soyuz_7k\\OK_thermal8");
	hMesh[12] = oapiLoadMeshGlobal("Soyuz_7k\\OK_vzor");
	hMesh[13] = oapiLoadMeshGlobal("Soyuz_7k\\OK_heatshield");
	hMesh[14] = oapiLoadMeshGlobal("Soyuz_7k\\OK_parachute-cover-main");
	hMesh[15] = oapiLoadMeshGlobal("Soyuz_7k\\OK_parachute-cover-backup");
	hMesh[16] = oapiLoadMeshGlobal("Soyuz_7k\\OK_small-cover");
	hMesh[17] = oapiLoadMeshGlobal("Soyuz_7k\\OK_SA_body_burnt");

	mesh[0] = AddMesh(hMesh[0]);
	mesh[1] = AddMesh(hMesh[1]);
	mesh[2] = AddMesh(hMesh[2]);
	mesh[3] = AddMesh(hMesh[3]);
	mesh[4] = AddMesh(hMesh[4]);
	mesh[5] = AddMesh(hMesh[5]);
	mesh[6] = AddMesh(hMesh[6]);
	mesh[7] = AddMesh(hMesh[7]);
	mesh[8] = AddMesh(hMesh[8]);
	mesh[9] = AddMesh(hMesh[9]);
	mesh[10] = AddMesh(hMesh[10]);
	mesh[11] = AddMesh(hMesh[11]);
	mesh[12] = AddMesh(hMesh[12]);
	mesh[13] = AddMesh(hMesh[13]);
	mesh[14] = AddMesh(hMesh[14]);
	mesh[15] = AddMesh(hMesh[15]);
	mesh[16] = AddMesh(hMesh[16]);
	mesh[17] = AddMesh(hMesh[17]);

	VC_mesh = oapiLoadMeshGlobal("Soyuz_7k\\OK_SA_Interior");
	VC_mesh_idx[0] = AddMesh(VC_mesh);
	VC_panel_mesh = oapiLoadMeshGlobal("Soyuz_7k\\Sirius7k_MainPanel");
	VC_mesh_idx[1] = AddMesh(VC_panel_mesh);
	VC_KSU_mesh = oapiLoadMeshGlobal("Soyuz_7k\\Sirius7k_KSU");
	VC_mesh_idx[2] = AddMesh(VC_KSU_mesh);
	VC_Vzor_mesh = oapiLoadMeshGlobal("Soyuz_7k\\OK_Vzor_VC");
	VC_mesh_idx[3] = AddMesh(VC_Vzor_mesh);



	for (int i = 0; i < 17; i++) {
		SetMeshVisibilityMode(mesh[i], MESHVIS_EXTERNAL);
	}
	SetMeshVisibilityMode(mesh[2], MESHVIS_ALWAYS);
	SetMeshVisibilityMode(mesh[17], MESHVIS_NEVER);
	
	for (int i = 0; i < 4; i++) {
		SetMeshVisibilityMode(VC_mesh_idx[i], MESHVIS_VC);
	}

	mesh_shift_y = 0;
	mesh_shift_z = MESH_SHIFT;
	ShiftCG(_V(0, 0, -mesh_shift_z));
}

bool Soyuz7k_T::clbkDrawHUD(int mode, const HUDPAINTSPEC* hps, oapi::Sketchpad* skp) {

	int s = hps->H;
	double d = (s * 0.00130208);

	VESSEL3::clbkDrawHUD(mode, hps, skp);

	int sw = ((hps->W));
	int lw = (int)(16 * sw / 1024);
	int lwoffset = sw - (18 * lw);
	int hlw = (int)(lw / 2);

	int roxl = 0;
	int royl = 0;

	double ds = s;
	double dsw = sw;
	double sc_ratio = ds / dsw;


	if (sc_ratio < 0.7284)
	{
		roxl = (lw * 3);
		royl = (int)(-88 * d);
	}

	int l1 = (int)(170 * d);
	int l2 = (int)(188 * d);
	int l3 = (int)(206 * d);
	int w1 = (int)(312 * d);
	int w2 = (int)(330 * d);
	int w3 = (int)(348 * d);
	int w4 = (int)(366 * d);
	int w5 = (int)(384 * d);
	int w6 = (int)(402 * d);
	int w7 = (int)(420 * d);
	int w8 = (int)(438 * d);
	int w9 = (int)(456 * d);
	int w10 = (int)(474 * d);
	int w11 = (int)(492 * d);
	int w12 = (int)(510 * d);
	int w13 = (int)(528 * d);
	int w14 = (int)(546 * d);
	int w15 = (int)(564 * d);
	int w16 = (int)(582 * d);
	int w17 = (int)(600 * d);
	int w18 = (int)(618 * d);
	int w19 = (int)(636 * d);
	int w20 = (int)(654 * d);
	int w21 = (int)(672 * d);
	int w22 = (int)(690 * d);
	int w23 = (int)(708 * d);
	int w24 = (int)(726 * d);

	if (oapiCockpitMode() != COCKPIT_VIRTUAL)
	{
		if (sep_status == 0) {
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				double main_prop = oapiGetPropellantMass(hpr);
				if (ori_ap_status != 0) {
					if (ori_ap_status == 1) sprintf(abuf, "Orbital Orientation Mode: Roll Search");
					else if (ori_ap_status == 2) sprintf(abuf, "Orbital Orientation Mode: Pitch Align");
					else if (ori_ap_status == 3) sprintf(abuf, "Orbital Orientation Mode: Yaw Align");
					else if (ori_ap_status == 4) sprintf(abuf, "Orbital Orientation Mode: Roll Adjust");
					else if (ori_ap_status == 5) sprintf(abuf, "Orbital Orientation Mode: Pitch Adjust");
					else sprintf(abuf, "Orbital Orientation Mode: Attitude Hold");
					skp->Text((roxl + 400), (l1 + royl), abuf, strlen(abuf));
				}
				else if (sun_ap_status != 0) {
					if (sun_ap_status == 1) sprintf(abuf, "Solar Orientation Mode: Pitch Search");
					else if (sun_ap_status == 2) sprintf(abuf, "Solar Orientation Mode: Roll Search");
					else if (sun_ap_status == 3) sprintf(abuf, "Solar Orientation Mode: Pitch Align");
					else sprintf(abuf, "Solar Orientation Mode: Attitude Hold");
					skp->Text((roxl + 400), (l1 + royl), abuf, strlen(abuf));
				}
				else if (inertial_status != 0) {
					if (inertial_status == 1) sprintf(abuf, "Inertial Orientation Mode: Roll Correct");
					else sprintf(abuf, "Inertial Orientation Mode: Pitch/Yaw Correct");
					skp->Text((roxl + 400), (l1 + royl), abuf, strlen(abuf));
				}
				else if (igla_status != 0) {
					sprintf(abuf, "Igla %d", igla_status);
					skp->Text((roxl + 400), (l1 + royl), abuf, strlen(abuf));
				}
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (vzor_status == 2) {
					sprintf(abuf, "Vzor forward [C]");
					skp->Text((roxl + 400), (l2 + royl), abuf, strlen(abuf));
				}
				else if (vzor_status == 3){
					sprintf(abuf, "Vzor nadir [C]");
					skp->Text((roxl + 400), (l2 + royl), abuf, strlen(abuf));
				}
				else {
					sprintf(abuf, "Vzor reorienting...");
					skp->Text((roxl + 400), (l2 + royl), abuf, strlen(abuf));
				}
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (BB_status) {
					sprintf(abuf, "%0.0lf%% - Main Battery", 100 * BB_power / MAX_BB_POWER);
				}
				else if (SA_BB_status) {
					sprintf(abuf, "%0.0lf%% - SA Battery", 100 * SA_BB_power / MAX_SA_POWER);
				}
				else if (RB_status) {
					sprintf(abuf, "%0.0lf%% - Backup Battery", 100 * RB_power / MAX_RB_POWER);
				}
				else {
					sprintf(abuf, "NO POWER");
				}
				skp->Text((roxl + 400), (l3 + royl), abuf, strlen(abuf));
			}
			if (tgt_range < IGLA_RANGE && (igla_status != 0 || man_appr)) {
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					if(tgt_range < 1000) sprintf(abuf, "Target range: %0.1lf m", tgt_range);
					else sprintf(abuf, "Target range: %0.2lf km", tgt_range/1000);
					skp->Text((roxl + 1200), (w1 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "Target range rate: %+0.2lf m/s", tgt_range_rate);
					skp->Text((roxl + 1200), (w2 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "Rel ori pitch: %+0.1lf deg", tgt_err_pitch);
					skp->Text((roxl + 1200), (w3 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "Rel ori yaw: %+0.1lf deg", tgt_err_yaw);
					skp->Text((roxl + 1200), (w4 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "Rel ori roll: %+0.1lf deg", tgt_err_roll);
					skp->Text((roxl + 1200), (w5 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "Rel vel x: %+0.2lf m/s", tgt_relvel.x);
					skp->Text((roxl + 1200), (w6 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "Rel vel y: %+0.2lf m/s", tgt_relvel.y);
					skp->Text((roxl + 1200), (w7 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "Rel vel z: %+0.2lf m/s", tgt_relvel.z);
					skp->Text((roxl + 1200), (w8 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "LOS drift to +y: %+0.2lf deg", roll_drift_vector);
					skp->Text((roxl + 1200), (w9 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "LOS burns: %d", igla_LOS_burn);
					skp->Text((roxl + 1200), (w10 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "Burn_seq: %d", burn_seq);
					skp->Text((roxl + 1200), (w11 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "Speed Target: %+0.2lf", speed_target);
					skp->Text((roxl + 1200), (w12 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "Intz: %+0.2lf", intz);
					skp->Text((roxl + 1200), (w13 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "Last LOS roll: %+0.2lf", last_tgt_err_roll);
					skp->Text((roxl + 1200), (w14 + royl), abuf, strlen(abuf));
				}
				
			}

			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "SKDU Delta-V: %0.1lf m/s", deltav);
				skp->Text((roxl), (w1 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "DPO propellant: %0.1lf kg", dpo_prop);
				skp->Text((roxl), (w2 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if(backup_DO) sprintf(abuf, "DO propellant: %0.1lf kg", do_res_prop);
				else sprintf(abuf, "DO propellant: %0.1lf kg", do_prop);
				skp->Text((roxl), (w3 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (engine_status == 0) sprintf(abuf, "Engine mode [E]: SKD");
				else sprintf(abuf, "Engine mode [E]: DKD");
				skp->Text((roxl), (w4 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (rcs_status == 0) sprintf(abuf, "RCS mode [M]: DPO");
				else if (rcs_status == 2) sprintf(abuf, "RCS mode [M]: DO");
				else sprintf(abuf, "RCS mode [M]: OFF");
				skp->Text((roxl), (w5 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (CM_jett == 1) sprintf(abuf, "Docking probe [G]: jettisoned");
				else if (dock_probe_status == 0) sprintf(abuf, "Docking probe [G]: retracted");
				else if (dock_probe_status == 1) sprintf(abuf, "Docking probe [G]: extending...");
				else if (dock_probe_status == 3 || dock_probe_status == 4) sprintf(abuf, "Docking probe [G]: retracting...");
				else sprintf(abuf, "Docking probe [G]: extended");
				skp->Text((roxl), (w6 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (latches_status == 0) sprintf(abuf, "Latches: closed");
				else if (latches_status == 1) sprintf(abuf, "Latches: extending...");
				else if (latches_status == 3) sprintf(abuf, "Latches: closing...");
				else sprintf(abuf, "Latches: extended");
				skp->Text((roxl), (w7 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (hooks_status == 0) sprintf(abuf, "Hooks: open");
				else if (hooks_status == 1) sprintf(abuf, "Hooks: closing...");
				else if (hooks_status == 3) sprintf(abuf, "Hooks: opening...");
				else sprintf(abuf, "Hooks: closed");
				skp->Text((roxl), (w8 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (beac1.active == false) sprintf(abuf, "Docking lights [L]: OFF");
				else sprintf(abuf, "Docking lights [L]: ON");
				skp->Text((roxl), (w9 + royl), abuf, strlen(abuf));
			}

			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "-- Angular Velocity --");
				skp->Text((roxl), (w10 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Wx: %+0.3lf deg/s", avel.x);
				skp->Text((roxl), (w11 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Wy: %+0.3lf deg/s", avel.y);
				skp->Text((roxl), (w12 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Wz: %+0.3lf deg/s", avel.z);
				skp->Text((roxl), (w13 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (pulsed_status == 0) sprintf(abuf, "Control mode [F]: RO");
				else if (pulsed_status == 1) sprintf(abuf, "Control mode [F]: Pulsed RO");
				else sprintf(abuf, "Control mode [F]: OFF");
				skp->Text((roxl), (w14 + royl), abuf, strlen(abuf));
			}


			phi = GetSlipAngle();
			theta = PI / 2 - GetPitch();
			VECTOR3 dir;
			dir.z = cos(phi) * sin(theta);
			dir.x = sin(phi) * sin(theta);
			dir.y = cos(theta);

			ion_err = acos(dotp(dir, _V(0, 0, 1)));

			phi = GetBank();
			theta = PI / 2 - GetPitch();
			dir.y = cos(phi) * sin(theta);
			dir.x = sin(phi) * sin(theta);
			dir.z = cos(theta);

			IR_err = acos(dotp(dir, _V(0, 1, 0)));

			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Orientation sensors:");
				skp->Text((roxl), (w16 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Ion err: %+.0f deg", ion_err * DEG);
				skp->Text((roxl), (w17 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (IR_err * DEG < 150) sprintf(abuf, "IR err: %.0f deg", IR_err * DEG);
				else sprintf(abuf, "IR err: -- deg");
				skp->Text((roxl), (w18 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (sun_err * DEG < 6) {
					sprintf(abuf, "45K aligned");
					skp->Text((roxl), (w19 + royl), abuf, strlen(abuf));
				}
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Accel z: %+.3f m/s  Target: %+.3f m/s", accel_z, dv_accel);
				skp->Text((roxl), (w20 + royl), abuf, strlen(abuf));
			}
			if (inertial_status != 0) {
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "-- BDUS Integr --");
					skp->Text((roxl), (w21 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "x: %+0.3lf deg", intx);
					skp->Text((roxl), (w22 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "y: %+0.3lf deg", inty);
					skp->Text((roxl), (w23 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "z: %+0.3lf deg", intz);
					skp->Text((roxl), (w24 + royl), abuf, strlen(abuf));
				}
			}

		}
		else if (sep_status == 2) {
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (ori_ap_status != 0) {
					if (ori_ap_status == 1) sprintf(abuf, "Orbital Orientation Mode: Roll Search");
					else if (ori_ap_status == 2) sprintf(abuf, "Orbital Orientation Mode: Pitch Align");
					else if (ori_ap_status == 3) sprintf(abuf, "Orbital Orientation Mode: Yaw Align");
					else if (ori_ap_status == 4) sprintf(abuf, "Orbital Orientation Mode: Roll Adjust");
					else if (ori_ap_status == 5) sprintf(abuf, "Orbital Orientation Mode: Pitch Adjust");
					else sprintf(abuf, "Orbital Orientation Mode: Attitude Hold");
					skp->Text((roxl + 400), (l1 + royl), abuf, strlen(abuf));
				}
				else if (sun_ap_status != 0) {
					sprintf(abuf, "Solar Orientation Mode %d", sun_ap_status);
					skp->Text((roxl + 400), (l1 + royl), abuf, strlen(abuf));
				}
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (vzor_status == 2) {
					sprintf(abuf, "Vzor forward [C]");
					skp->Text((roxl + 400), (l2 + royl), abuf, strlen(abuf));
				}
				else if (vzor_status == 3) {
					sprintf(abuf, "Vzor nadir [C]");
					skp->Text((roxl + 400), (l2 + royl), abuf, strlen(abuf));
				}
				else {
					sprintf(abuf, "Vzor reorienting...");
					skp->Text((roxl + 400), (l2 + royl), abuf, strlen(abuf));
				}
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (BB_status) {
					sprintf(abuf, "%0.0lf%% - Main Battery", 100 * BB_power / MAX_BB_POWER);
				}
				else if (SA_BB_status) {
					sprintf(abuf, "%0.0lf%% - SA Battery", 100 * SA_BB_power / MAX_SA_POWER);
				}
				else {
					sprintf(abuf, "%0.0lf%% - Backup Battery", 100 * RB_power / MAX_RB_POWER);
				}
				skp->Text((roxl + 400), (l3 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "SKDU Delta-V: %0.1lf m/s", deltav);
				skp->Text((roxl), (w1 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "DPO propellant: %0.1lf kg", dpo_prop);
				skp->Text((roxl), (w2 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (backup_DO) sprintf(abuf, "DO propellant: %0.1lf kg", do_res_prop);
				else sprintf(abuf, "DO propellant: %0.1lf kg", do_prop);
				skp->Text((roxl), (w3 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (engine_status == 0) sprintf(abuf, "Engine mode [E]: SKD");
				else sprintf(abuf, "Engine mode [E]: DKD");
				skp->Text((roxl), (w4 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (rcs_status == 0) sprintf(abuf, "RCS mode [M]: DPO");
				else if (rcs_status == 2) sprintf(abuf, "RCS mode [M]: DO");
				else sprintf(abuf, "RCS mode [M]: OFF");
				skp->Text((roxl), (w5 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "-- Angular Velocity --");
				skp->Text((roxl), (w8 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Wx: %+0.3lf deg/s", avel.x);
				skp->Text((roxl), (w9 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Wy: %+0.3lf deg/s", avel.y);
				skp->Text((roxl), (w10 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Wz: %+0.3lf deg/s", avel.z);
				skp->Text((roxl), (w11 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (pulsed_status == 0) sprintf(abuf, "Control mode [F]: RO");
				else if (pulsed_status == 1) sprintf(abuf, "Control mode [F]: Pulsed RO");
				else sprintf(abuf, "Control mode [F]: OFF");
				skp->Text((roxl), (w12 + royl), abuf, strlen(abuf));
			}


			phi = GetSlipAngle();
			theta = PI / 2 - GetPitch();
			VECTOR3 dir;
			dir.z = cos(phi) * sin(theta);
			dir.x = sin(phi) * sin(theta);
			dir.y = cos(theta);

			ion_err = acos(dotp(dir, _V(0, 0, 1)));

			phi = GetBank();
			theta = PI / 2 - GetPitch();
			dir.y = cos(phi) * sin(theta);
			dir.x = sin(phi) * sin(theta);
			dir.z = cos(theta);

			IR_err = acos(dotp(dir, _V(0, 1, 0)));

			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Orientation sensors:");
				skp->Text((roxl), (w15 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Ion err: %+.0f deg", ion_err * DEG);
				skp->Text((roxl), (w16 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (IR_err * DEG < 100) sprintf(abuf, "IR err: %.0f deg", IR_err * DEG);
				else sprintf(abuf, "IR err: -- deg");
				skp->Text((roxl), (w17 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (sun_err * DEG < 6) {
					sprintf(abuf, "45K aligned");
					skp->Text((roxl), (w18 + royl), abuf, strlen(abuf));
				}
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Accel z: %+.3f m/s  Target: %+.3f m/s", accel_z, dv_accel);
				skp->Text((roxl), (w20 + royl), abuf, strlen(abuf));
			}
			if (inertial_status != 0) {
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "-- BDUS Integr --");
					skp->Text((roxl), (w21 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "x: %+0.3lf deg", intx);
					skp->Text((roxl), (w22 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "y: %+0.3lf deg", inty);
					skp->Text((roxl), (w23 + royl), abuf, strlen(abuf));
				}
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					sprintf(abuf, "z: %+0.3lf deg", intz);
					skp->Text((roxl), (w24 + royl), abuf, strlen(abuf));
				}
			}
		}
		else if (sep_status >= 4){
			
			sa_prop = oapiGetPropellantMass(hpr_sa);

			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "%0.0lf%% - SA Battery", 100 * SA_BB_power / MAX_SA_POWER);
				skp->Text((roxl + 400), (l3 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "DO propellant: %0.1lf kg", sa_prop);
				skp->Text((roxl), (w3 + royl), abuf, strlen(abuf));
			}

			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "-- Angular Velocity --");
				skp->Text((roxl), (w8 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Wx: %+0.3lf deg/s", avel.x);
				skp->Text((roxl), (w9 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Wy: %+0.3lf deg/s", avel.y);
				skp->Text((roxl), (w10 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Wz: %+0.3lf deg/s", avel.z);
				skp->Text((roxl), (w11 + royl), abuf, strlen(abuf));
			}

			VECTOR3 force; 
			GetDragVector(force);

			double forcetot = sqrtl(pow(force.x, 2) + pow(force.y, 2) + pow(force.z, 2));
			double Gforce = (forcetot / GetMass()) / 9.81;
			
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "G: %0.1lf", Gforce);
				skp->Text((roxl), (w13 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Int. Accel: %0.1lf m/s", accel_SUS);
				skp->Text((roxl), (w14 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "SUS Status: %d", SUS_status);
				skp->Text((roxl), (w15 + royl), abuf, strlen(abuf));
			}
		}
	}
	return true;
}

int Soyuz7k_T::clbkConsumeDirectKey(char* kstate) {
	
	if (GetAttitudeMode() == RCS_ROT && sep_status < 3) {
		if (KEYDOWN(kstate, OAPI_KEY_NUMPAD2)) RESETKEY(kstate, OAPI_KEY_NUMPAD2);
		if (KEYDOWN(kstate, OAPI_KEY_NUMPAD8)) RESETKEY(kstate, OAPI_KEY_NUMPAD8);
		if (KEYDOWN(kstate, OAPI_KEY_NUMPAD1)) RESETKEY(kstate, OAPI_KEY_NUMPAD1);
		if (KEYDOWN(kstate, OAPI_KEY_NUMPAD3)) RESETKEY(kstate, OAPI_KEY_NUMPAD3);
		if (KEYDOWN(kstate, OAPI_KEY_NUMPAD4)) RESETKEY(kstate, OAPI_KEY_NUMPAD4);
		if (KEYDOWN(kstate, OAPI_KEY_NUMPAD6)) RESETKEY(kstate, OAPI_KEY_NUMPAD6);	
	}
	if (no_power) {
		if (GetAttitudeMode() == RCS_LIN) {
			if (KEYDOWN(kstate, OAPI_KEY_NUMPAD2)) RESETKEY(kstate, OAPI_KEY_NUMPAD2);
			if (KEYDOWN(kstate, OAPI_KEY_NUMPAD8)) RESETKEY(kstate, OAPI_KEY_NUMPAD8);
			if (KEYDOWN(kstate, OAPI_KEY_NUMPAD1)) RESETKEY(kstate, OAPI_KEY_NUMPAD1);
			if (KEYDOWN(kstate, OAPI_KEY_NUMPAD3)) RESETKEY(kstate, OAPI_KEY_NUMPAD3);
			if (KEYDOWN(kstate, OAPI_KEY_NUMPAD9)) RESETKEY(kstate, OAPI_KEY_NUMPAD9);
			if (KEYDOWN(kstate, OAPI_KEY_NUMPAD6)) RESETKEY(kstate, OAPI_KEY_NUMPAD6);
		}
		if (KEYDOWN(kstate, OAPI_KEY_ADD)) RESETKEY(kstate, OAPI_KEY_ADD);
		if (KEYDOWN(kstate, OAPI_KEY_SUBTRACT)) RESETKEY(kstate, OAPI_KEY_SUBTRACT);
	}
	
	return 0;
}

int Soyuz7k_T::clbkConsumeBufferedKey(DWORD key, bool down, char* kstate) {
	
	//process key release
	if (!down) {
		switch (key) {
		
		//fly-by-wire
		case OAPI_KEY_NUMPAD2: 
			if (GetAttitudeMode() == RCS_ROT && sep_status < 3) {
				if (pulsed_status == 0) rotation_command[0] = 4;
				else rotation_command[0] = 0;
				RO_detent[0] = 0;
				return 1;
			}
			

		case OAPI_KEY_NUMPAD8:
			if (GetAttitudeMode() == RCS_ROT && sep_status < 3) {
				if (pulsed_status == 0) rotation_command[1] = 4;
				else rotation_command[1] = 0;
				RO_detent[1] = 0;
				return 1;
			}
			

		case OAPI_KEY_NUMPAD1:
			if (GetAttitudeMode() == RCS_ROT && sep_status < 3) {
				if (pulsed_status == 0) rotation_command[2] = 4;
				else rotation_command[2] = 0;
				RO_detent[2] = 0;
				return 1;
			}
			

		case OAPI_KEY_NUMPAD3:
			if (GetAttitudeMode() == RCS_ROT && sep_status < 3) {
				if (pulsed_status == 0) rotation_command[3] = 4;
				else rotation_command[3] = 0;
				RO_detent[3] = 0;
				return 1;
			}
			

		case OAPI_KEY_NUMPAD4:
			if (GetAttitudeMode() == RCS_ROT && sep_status < 3) {
				if (pulsed_status == 0) rotation_command[4] = 4;
				else rotation_command[4] = 0;
				RO_detent[4] = 0;
				return 1;
			}
			

		case OAPI_KEY_NUMPAD6:
			if (GetAttitudeMode() == RCS_ROT && sep_status < 3) {
				if (pulsed_status == 0) rotation_command[5] = 4;
				else rotation_command[5] = 0;
				RO_detent[5] = 0;
				return 1;
			}
			
		}

		return 0;
	}
	
	//process key press
	else {

		if (KEYMOD_CONTROL(kstate)) {
			switch (key) {

				//fly-by-wire
			case OAPI_KEY_NUMPAD2:
				if (GetAttitudeMode() == RCS_ROT && rotation_command[0] == 0 && ori_ap_status == 0 && sun_ap_status == 0 && sep_status < 3) {
					rotation_command[0] = 1;
					RO_detent[0] = 1;
					return 1;
				}
				else return 0;


			case OAPI_KEY_NUMPAD8:
				if (GetAttitudeMode() == RCS_ROT && rotation_command[1] == 0 && ori_ap_status == 0 && sun_ap_status == 0 && sep_status < 3) {
					rotation_command[1] = 1;
					RO_detent[1] = 1;
					return 1;
				}
				else return 0;


			case OAPI_KEY_NUMPAD1:
				if (GetAttitudeMode() == RCS_ROT && rotation_command[2] == 0 && ori_ap_status == 0 && sun_ap_status == 0 && sep_status < 3) {
					rotation_command[2] = 1;
					RO_detent[2] = 1;
					return 1;
				}
				else return 0;

			case OAPI_KEY_NUMPAD3:
				if (GetAttitudeMode() == RCS_ROT && rotation_command[3] == 0 && ori_ap_status == 0 && sun_ap_status == 0 && sep_status < 3) {
					rotation_command[3] = 1;
					RO_detent[2] = 1;
					return 1;
				}
				else return 0;

			case OAPI_KEY_NUMPAD4:
				if (GetAttitudeMode() == RCS_ROT && rotation_command[4] == 0 && ori_ap_status == 0 && sun_ap_status == 0 && sep_status < 3) {
					rotation_command[4] = 1;
					RO_detent[4] = 1;
					return 1;
				}
				else return 0;

			case OAPI_KEY_NUMPAD6:
				if (GetAttitudeMode() == RCS_ROT && rotation_command[5] == 0 && ori_ap_status == 0 && sun_ap_status == 0 && sep_status < 3) {
					rotation_command[5] = 1;
					RO_detent[5] = 1;
					return 1;
				}
				else return 0;
			}
			return 0;
		}

		else {
			switch (key) {

				//fly-by-wire
			case OAPI_KEY_NUMPAD2:
				if (GetAttitudeMode() == RCS_ROT && rotation_command[0] == 0 && ori_ap_status == 0 && sun_ap_status == 0 && sep_status < 3) {
					rotation_command[0] = 1;
					RO_detent[0] = 2;
					return 1;
				}
				else return 0;


			case OAPI_KEY_NUMPAD8:
				if (GetAttitudeMode() == RCS_ROT && rotation_command[1] == 0 && ori_ap_status == 0 && sun_ap_status == 0 && sep_status < 3) {
					rotation_command[1] = 1;
					RO_detent[1] = 2;
					return 1;
				}
				else return 0;


			case OAPI_KEY_NUMPAD1:
				if (GetAttitudeMode() == RCS_ROT && rotation_command[2] == 0 && ori_ap_status == 0 && sun_ap_status == 0 && sep_status < 3) {
					rotation_command[2] = 1;
					RO_detent[2] = 2;
					return 1;
				}
				else return 0;

			case OAPI_KEY_NUMPAD3:
				if (GetAttitudeMode() == RCS_ROT && rotation_command[3] == 0 && ori_ap_status == 0 && sun_ap_status == 0 && sep_status < 3) {
					rotation_command[3] = 1;
					RO_detent[3] = 2;
					return 1;
				}
				else return 0;

			case OAPI_KEY_NUMPAD4:
				if (GetAttitudeMode() == RCS_ROT && rotation_command[4] == 0 && ori_ap_status == 0 && sun_ap_status == 0 && sep_status < 3) {
					rotation_command[4] = 1;
					RO_detent[4] = 2;
					return 1;
				}
				else return 0;

			case OAPI_KEY_NUMPAD6:
				if (GetAttitudeMode() == RCS_ROT && rotation_command[5] == 0 && ori_ap_status == 0 && sun_ap_status == 0 && sep_status < 3) {
					rotation_command[5] = 1;
					RO_detent[5] = 2;
					return 1;
				}
				else return 0;


				//deploy/retract antennae
			/*case OAPI_KEY_K:
				if (antennae_status == 0) antennae_status = 1;
				else if (antennae_status == 1) antennae_status = 3;
				else if (igla_sides_status == 2) igla_sides_status = 3;
				else if (antennae_status == 3) antennae_status = 1;
				else if (igla_sides_status == 3) igla_sides_status = 1;
				else if (igla_sides_status == 1) igla_sides_status = 3;
				return 1;*/

				//deploy/retract docking probe
			case OAPI_KEY_G:
				if (ssvp_on) {
					if (dock_probe_status == 0) dock_probe_status = 1;
					else if (dock_probe_status == 1) dock_probe_status = 3;
					else if (dock_probe_status == 2) dock_probe_status = 3;
					else if (dock_probe_status == 3) dock_probe_status = 1;
				}
				return 1;

				//switch RCS mode
			case OAPI_KEY_M:
				if (xflag == 0 && yflag == 0 && zflag == 0 && ori_ap_status == 0 && sun_ap_status == 0) {	//do not allow mode switching while mid-pulse firing nor with orientation mode on
					if (rcs_status == 0) rcs_status = 1;	
					else if (rcs_status == 2) {
						SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
						SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
						SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
						SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
						SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
						SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
						SetThrusterGroupLevel(THGROUP_ATT_UP, 0);
						SetThrusterGroupLevel(THGROUP_ATT_DOWN, 0);
						SetThrusterGroupLevel(THGROUP_ATT_LEFT, 0);
						SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 0);
						SetThrusterGroupLevel(THGROUP_ATT_FORWARD, 0);
						SetThrusterGroupLevel(THGROUP_ATT_BACK, 0);
						SetThrusterGroupLevel(THGROUP_RETRO, 0);
						DelThrusterGroup(THGROUP_ATT_YAWLEFT, false);
						DelThrusterGroup(THGROUP_ATT_YAWRIGHT, false);
						DelThrusterGroup(THGROUP_ATT_PITCHUP, false);
						DelThrusterGroup(THGROUP_ATT_PITCHDOWN, false);
						DelThrusterGroup(THGROUP_ATT_BANKLEFT, false);
						DelThrusterGroup(THGROUP_ATT_BANKRIGHT, false);
						DelThrusterGroup(THGROUP_ATT_UP, false);
						DelThrusterGroup(THGROUP_ATT_DOWN, false);
						DelThrusterGroup(THGROUP_ATT_LEFT, false);
						DelThrusterGroup(THGROUP_ATT_RIGHT, false);
						DelThrusterGroup(THGROUP_ATT_FORWARD, false);
						DelThrusterGroup(THGROUP_ATT_BACK, false);
						DelThrusterGroup(THGROUP_RETRO, false);
						rcs_status = 6;
					}
					else if (rcs_status == 6) rcs_status = 3;					
				}
				return 1;

				//switch RCS pulsed mode
			case OAPI_KEY_F:
				if (pulsed_status == 0) pulsed_status = 1;	//switch to pulsed mode
				else if (pulsed_status == 1) pulsed_status = 2;						//switch off
				else pulsed_status = 0;						//switch to continuous mode
				return 1;

				//orbital orientation autopilot
			case OAPI_KEY_O:
				if (ori_ap_status == 0 && sun_ap_status == 0 && igla_status == 0 && BDUS_status == BDUS_ON && hAttached == NULL) {
					ori_ap_status = 1;		//switch orbital orientation mode on
					rcs_status = 1;			//force DO mode
				}
				else if (ori_ap_status != 0) {
					ori_ap_status = 0;		//switch orbital orientation mode off, regardless of current state
					ori_ap_aux = 0;
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
				}
				return 1;

				//solar orientation autopilot
			case OAPI_KEY_K:
				if (sun_ap_status == 0 && ori_ap_status == 0 && igla_status == 0 && BDUS_status == BDUS_ON && hAttached == NULL) {
					sun_ap_status = 1;		//switch solar orientation mode on
					rcs_status = 1;			//force DO mode
				}
				else if (sun_ap_status != 0) {
					sun_ap_status = 0;		//switch solar orientation mode off, regardless of current state
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
				}
				return 1;
			
				//Igla system on/off
			case OAPI_KEY_I:
				if (sun_ap_status == 0 && ori_ap_status == 0 && igla_status == 0 && hAttached == NULL) {
					igla_status = 1;		//switch igla mode on
					rcs_status = 1;		  //force DO mode
				}
				else if (igla_status != 0) {
					IglaOff();								//switch igla mode off, regardless of current state
				}
				return 1;

				//lights on/off
			case OAPI_KEY_L:
				if (antennae_status == 2 && beac1.active == false && !no_power) {
					beac1.active = true;
					spotlight1->Activate(true);
					spotlight2->Activate(true);
					beac2.active = true;
					beac3.active = true;
					beac4.active = true;
					beac6.active = true;
					spotlight_power = 0.11;
				}
				else if (!no_power) {
					beac1.active = false;
					spotlight1->Activate(false);
					spotlight2->Activate(false);
					beac2.active = false;
					beac3.active = false;
					beac4.active = false;
					beac6.active = false;
					spotlight_power = 0;
				}
				return 1;

				//Jettison Modules
			case OAPI_KEY_J:
				if (sep_status == 0 && hAttached == NULL) sep_status = 1;
				else if (sep_status == 2) sep_status = 3;
				return 1;

				//Vzor mode
			case OAPI_KEY_C:
				if (vzor_status == 0 || vzor_status == 2) {
					vzor_status = 1;
					if (gcEnabled()) {
						if (vzor_c_on) {
							vzor_c_on = 2;	// intermediate state to render a blank before setting to zero
							gcCustomCameraOnOff(hCam_vzor_fwd, false);
							gcCustomCameraOnOff(hCam_vzor_nad, false);
						}
					}
					//SetCameraDefaultDirection(_V(0, -0.994522, 0.104528));
					//oapiCameraSetCockpitDir(0, 0, false);
				}
				else if (vzor_status == 1 || vzor_status == 3) {
					vzor_status = 0;
					if (gcEnabled()) {
						if (vzor_c_on) {
							vzor_c_on = 2;	// intermediate state to render a blank before setting to zero
							gcCustomCameraOnOff(hCam_vzor_fwd, false);
							gcCustomCameraOnOff(hCam_vzor_nad, false);
						}
					}
					//SetCameraDefaultDirection(_V(0, 0, 1));
					//oapiCameraSetCockpitDir(0, 0, false);
				}
				return 1;

			case OAPI_KEY_E:
				if(engine_status == 0) engine_status = 1;
				else if (engine_status == 1) engine_status = 3;
				else if (engine_status == 2) engine_status = 3;
				else if (engine_status == 3) engine_status = 1;
				return 1;

			case OAPI_KEY_D:
				if (main_chute_status == 0) {
					main_chute_status = 1;
					if (skin_status == 0) skin_status = 1;
					SetCrossSections(_V(150, 150, 1000));
					DelAirfoil(airfoil_v);
					DelAirfoil(airfoil_h);
					airfoil_v = CreateAirfoil2(LIFT_VERTICAL, _V(0, 0, 0), vliftPre, 26, 1000, 1);
					airfoil_h = CreateAirfoil2(LIFT_HORIZONTAL, _V(0, 0, 0), hlift, 26, 1000, 1);
				}
				else if (main_chute_status == 1) {
					main_chute_status = 3;
					SetCrossSections(_V(3.7, 3.7, 3.79));
					DelAirfoil(airfoil_v);
					DelAirfoil(airfoil_h);
					airfoil_v = CreateAirfoil2(LIFT_VERTICAL, _V(0, -0.0005, 0.008), vliftPre, 2.17, 6.82, 0.1);
					airfoil_h = CreateAirfoil2(LIFT_HORIZONTAL, _V(0, 0, 0), hlift, 2.17, 6.82, 0.1);
				}
				else if (main_chute_status == 2) {
					main_chute_status = 3;
					SetCrossSections(_V(3.7, 3.7, 3.79));
					DelAirfoil(airfoil_v);
					DelAirfoil(airfoil_h);
					airfoil_v = CreateAirfoil2(LIFT_VERTICAL, _V(0, -0.0005, 0.008), vliftPre, 2.17, 6.82, 0.1);
					airfoil_h = CreateAirfoil2(LIFT_HORIZONTAL, _V(0, 0, 0), hlift, 2.17, 6.82, 0.1);
				}
				else if (main_chute_status == 3) {
					main_chute_status = 1;
					SetCrossSections(_V(150, 150, 1000));
					DelAirfoil(airfoil_v);
					DelAirfoil(airfoil_h);
					airfoil_v = CreateAirfoil2(LIFT_VERTICAL, _V(0, 0, 0), vliftPre, 26, 1000, 1);
					airfoil_h = CreateAirfoil2(LIFT_HORIZONTAL, _V(0, 0, 0), hlift, 26, 1000, 1);
				}
				return 1;

			}
			return 0;
		}
	}
}


// ==============================================================
// Custom functions
// ==============================================================

void Soyuz7k_T::vliftPre (double aoa, double M, double Re, double *cl, double *cm, double *cd)
{
	int i;
	/*
	const int nabsc = 19;
	// Angle of attack                   -180,     -160,     -140,     -120,     -100,     -80,     -60,     -40,      -20,     0,      20,      40,      60,      80,     100,     120,     140,     160,     180,
	static const double AOA[nabsc] = { -180 * RAD, -160 * RAD, -140 * RAD, -120 * RAD, -100 * RAD, -80 * RAD, -60 * RAD, -40 * RAD,  -20 * RAD, 0 * RAD,  20 * RAD,  40 * RAD,  60 * RAD,  80 * RAD, 100 * RAD, 120 * RAD, 140 * RAD, 160 * RAD, 180 * RAD };
	static const double CL[nabsc] = {      0,    -0.27,    -0.13,      0,        0,       0,       0,       0,        0,     0,       0,       0,       0,       0,       0,       0,    0.13,   0.27,   0 };
	static const double CM[nabsc] = { +0.00014,  -0.00000,   -0.00014,   -0.005,    -0.006,   -0.006,  -0.005, -0.003,  -0.002,     0, +0.002, +0.003, +0.005, +0.006, +0.006, +0.005, 0.005, 0.005, 0.005 };
	*/
	
	const int nabsc = 37;
	// Angle of attack                   
	static const double AOA[nabsc] = { -180 * RAD, -170 * RAD, -160 * RAD, -150 * RAD, -140 * RAD, -130 * RAD, -120 * RAD, -110 * RAD, -100 * RAD, -90 * RAD, -80 * RAD, -70 * RAD, -60 * RAD, -50 * RAD, -40 * RAD,  -30 * RAD, -20 * RAD, -10 * RAD, 0 * RAD, 10 * RAD, 20 * RAD, 30 * RAD, 40 * RAD, 50 * RAD, 60 * RAD, 70 * RAD, 80 * RAD, 90 * RAD, 100 * RAD, 110 * RAD, 120 * RAD, 130 * RAD, 140 * RAD, 150 * RAD, 160 * RAD, 170 * RAD, 180 * RAD };
	static const double CL[nabsc] = { 0, -0.135, -0.28, -0.405, -0.54, -0.405, -0.2, -0.04, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.04, 0.2, 0.405, 0.54, 0.405, 0.27, 0.135, 0};
	static const double CM[nabsc] = { 0.0315, 0.0165, 0.0015, -0.0135, -0.0285, -0.0725, -0.0975, -0.1225, -0.1475, -0.1725, -0.1975, -0.2225, -0.2475, -0.2725, -0.2975, -0.3225, -0.3475, -0.3725, -0.3975, -0.4225, -0.4475, 0.4525, 0.4275,	0.4025,	0.3775,	0.3525,	0.3275,	0.3025,	0.2775,	0.2525,	0.2275,	0.2025,	0.1775,	0.1525,	0.1275,	0.1025,	0.0775};


	for (i = 0; i < nabsc - 1 && AOA[i + 1] < aoa; i++);
	double f = (aoa - AOA[i]) / (AOA[i + 1] - AOA[i]);
	*cl = CL[i] + (CL[i + 1] - CL[i]) * f;  // aoa-dependent lift coefficient
	*cm = (CM[i] + (CM[i + 1] - CM[i]) * f)*(M);  // aoa-dependent moment coefficient
	double saoa = sin(aoa);
	double pd = 0.5 + 0.01 * saoa * saoa;  // profile drag
	*cd = pd + oapiGetInducedDrag(*cl, 1.05, 0.4);
	// profile drag + (lift-)induced drag
}

void Soyuz7k_T::vliftSUS(double aoa, double M, double Re, double* cl, double* cm, double* cd)
{
	int i;
	
	const int nabsc = 19;
	// Angle of attack                   -180,     -160,     -140,     -120,     -100,     -80,     -60,     -40,      -20,     0,      20,      40,      60,      80,     100,     120,     140,     160,     180,
	static const double AOA[nabsc] = { -180 * RAD, -160 * RAD, -140 * RAD, -120 * RAD, -100 * RAD, -80 * RAD, -60 * RAD, -40 * RAD,  -20 * RAD, 0 * RAD,  20 * RAD,  40 * RAD,  60 * RAD,  80 * RAD, 100 * RAD, 120 * RAD, 140 * RAD, 160 * RAD, 180 * RAD };
	static const double CL[nabsc] = { 0, -0.28, -0.54, -0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 0.54, 0.27, 0};
	static const double CM[nabsc] = { +0.000147,  +0.000007,   -0.000133,   -0.005,    -0.006,   -0.006,  -0.005, -0.003,  -0.002,     0, +0.002, +0.003, +0.005, +0.006, +0.006, +0.005, 0.005, 0.005, 0.005 };
	


	for (i = 0; i < nabsc - 1 && AOA[i + 1] < aoa; i++);
	double f = (aoa - AOA[i]) / (AOA[i + 1] - AOA[i]);
	*cl = CL[i] + (CL[i + 1] - CL[i]) * f;  // aoa-dependent lift coefficient
	*cm = (CM[i] + (CM[i + 1] - CM[i]) * f);  // aoa-dependent moment coefficient
	double saoa = sin(aoa);
	double pd = 0.5 + 0.01 * saoa * saoa;  // profile drag
	*cd = pd + oapiGetInducedDrag(*cl, 1.05, 0.4);
	// profile drag + (lift-)induced drag
}

void Soyuz7k_T::vliftDrogue(double aoa, double M, double Re, double* cl, double* cm, double* cd)
{
	int i;

	const int nabsc = 19;
	// Angle of attack                   -180,     -160,     -140,     -120,     -100,     -80,     -60,     -40,      -20,     0,      20,      40,      60,      80,     100,     120,     140,     160,     180,
	static const double AOA[nabsc] = { -180 * RAD, -160 * RAD, -140 * RAD, -120 * RAD, -100 * RAD, -80 * RAD, -60 * RAD, -40 * RAD,  -20 * RAD, 0 * RAD,  20 * RAD,  40 * RAD,  60 * RAD,  80 * RAD, 100 * RAD, 120 * RAD, 140 * RAD, 160 * RAD, 180 * RAD };
	static const double CL[nabsc] = { 0, -0.28, -0.54, -0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 0.54, 0.27, 0 };
	static const double CM[nabsc] = { +0.000147,  +0.000007,   -0.000133,   -0.005,    -0.006,   -0.006,  -0.005, -0.003,  -0.002,     0, +0.002, +0.003, +0.005, +0.006, +0.006, +0.005, 0.005, 0.005, 0.005 };



	for (i = 0; i < nabsc - 1 && AOA[i + 1] < aoa; i++);
	double f = (aoa - AOA[i]) / (AOA[i + 1] - AOA[i]);
	*cl = CL[i] + (CL[i + 1] - CL[i]) * f;  // aoa-dependent lift coefficient
	*cm = (CM[i] + (CM[i + 1] - CM[i]) * f);  // aoa-dependent moment coefficient
	double saoa = sin(aoa);
	double pd = 0.3 + 0.01 * saoa * saoa;  // profile drag
	*cd = pd + oapiGetInducedDrag(*cl, 1.05, 0.4);
	// profile drag + (lift-)induced drag
}

void Soyuz7k_T::vliftNull(double aoa, double M, double Re, double* cl, double* cm, double* cd)
{
	int i;
	const int nabsc = 19;
	// Angle of attack                   -180,     -160,     -140,     -120,     -100,     -80,     -60,     -40,      -20,     0,      20,      40,      60,      80,     100,     120,     140,     160,     180,
	static const double AOA[nabsc] = { -180 * RAD, -160 * RAD, -140 * RAD, -120 * RAD, -100 * RAD, -80 * RAD, -60 * RAD, -40 * RAD,  -20 * RAD, 0 * RAD,  20 * RAD,  40 * RAD,  60 * RAD,  80 * RAD, 100 * RAD, 120 * RAD, 140 * RAD, 160 * RAD, 180 * RAD };
	static const double CL[nabsc] = { 0,    0,    0,      0,        0,       0,       0,       0,        0,     0,       0,       0,       0,       0,       0,       0,    0,   0,   0 };
	static const double CM[nabsc] = { 0,  0,   0,   0,    0,   0,  0, 0,  0,     0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


	for (i = 0; i < nabsc - 1 && AOA[i + 1] < aoa; i++);
	double f = (aoa - AOA[i]) / (AOA[i + 1] - AOA[i]);
	*cl = CL[i] + (CL[i + 1] - CL[i]) * f;  // aoa-dependent lift coefficient
	*cm = CM[i] + (CM[i + 1] - CM[i]) * f;  // aoa-dependent moment coefficient
	double saoa = sin(aoa);
	double pd = 0.5 + 0.01 * saoa * saoa;  // profile drag
	*cd = pd + oapiGetInducedDrag(*cl, 1.05, 0.4);
	// profile drag + (lift-)induced drag
}

void Soyuz7k_T::hlift(double beta, double M, double Re, double* cl, double* cm, double* cd) {
	int i;
	const int nabsc = 19;
	// Angle of attack                   -180,     -160,     -140,     -120,     -100,     -80,     -60,     -40,      -20,     0,      20,      40,      60,      80,     100,     120,     140,     160,     180,
	static const double AOA[nabsc] = { -180 * RAD, -160 * RAD, -140 * RAD, -120 * RAD, -100 * RAD, -80 * RAD, -60 * RAD, -40 * RAD,  -20 * RAD, 0 * RAD,  20 * RAD,  40 * RAD,  60 * RAD,  80 * RAD, 100 * RAD, 120 * RAD, 140 * RAD, 160 * RAD, 180 * RAD };
	static const double CL[nabsc] = { 0, -0.27, -0.54, -0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 0.54, 0.27, 0 };
	static const double CM[nabsc] = { 0.00000,   -0.00014,   -0.005,    -0.006,   -0.006,  -0.005, -0.003,  -0.002,     0, +0.002, +0.003, +0.005, +0.006, +0.006, +0.005, 0.005, 0.005, +0.00014, 0 };

	for (i = 0; i < nabsc - 1 && AOA[i + 1] < beta; i++);
	double f = (beta - AOA[i]) / (AOA[i + 1] - AOA[i]);
	*cl = CL[i] + (CL[i + 1] - CL[i]) * f;  // aoa-dependent lift coefficient
	*cm = CM[i] + (CM[i + 1] - CM[i]) * f;  // aoa-dependent moment coefficient
	double saoa = sin(beta);
	double pd = 0.5 + 0.01 * saoa * saoa;  // profile drag
	*cd = pd + oapiGetInducedDrag(*cl, 1.05, 0.4);
	// profile drag + (lift-)induced drag
}

void Soyuz7k_T::hliftDrogue(double beta, double M, double Re, double* cl, double* cm, double* cd) {
	int i;
	const int nabsc = 19;
	// Angle of attack                   -180,     -160,     -140,     -120,     -100,     -80,     -60,     -40,      -20,     0,      20,      40,      60,      80,     100,     120,     140,     160,     180,
	static const double AOA[nabsc] = { -180 * RAD, -160 * RAD, -140 * RAD, -120 * RAD, -100 * RAD, -80 * RAD, -60 * RAD, -40 * RAD,  -20 * RAD, 0 * RAD,  20 * RAD,  40 * RAD,  60 * RAD,  80 * RAD, 100 * RAD, 120 * RAD, 140 * RAD, 160 * RAD, 180 * RAD };
	static const double CL[nabsc] = { 0, -0.27, -0.54, -0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 0.54, 0.27, 0 };
	static const double CM[nabsc] = { 0.00000,   -0.00014,   -0.005,    -0.006,   -0.006,  -0.005, -0.003,  -0.002,     0, +0.002, +0.003, +0.005, +0.006, +0.006, +0.005, 0.005, 0.005, +0.00014, 0 };

	for (i = 0; i < nabsc - 1 && AOA[i + 1] < beta; i++);
	double f = (beta - AOA[i]) / (AOA[i + 1] - AOA[i]);
	*cl = CL[i] + (CL[i + 1] - CL[i]) * f;  // aoa-dependent lift coefficient
	*cm = CM[i] + (CM[i + 1] - CM[i]) * f;  // aoa-dependent moment coefficient
	double saoa = sin(beta);
	double pd = 0.3 + 0.01 * saoa * saoa;  // profile drag
	*cd = pd + oapiGetInducedDrag(*cl, 1.05, 0.4);
	// profile drag + (lift-)induced drag
}

void Soyuz7k_T::hliftNull(double aoa, double M, double Re, double* cl, double* cm, double* cd)
{
	int i;
	const int nabsc = 19;
	// Angle of attack                   -180,     -160,     -140,     -120,     -100,     -80,     -60,     -40,      -20,     0,      20,      40,      60,      80,     100,     120,     140,     160,     180,
	static const double AOA[nabsc] = { -180 * RAD, -160 * RAD, -140 * RAD, -120 * RAD, -100 * RAD, -80 * RAD, -60 * RAD, -40 * RAD,  -20 * RAD, 0 * RAD,  20 * RAD,  40 * RAD,  60 * RAD,  80 * RAD, 100 * RAD, 120 * RAD, 140 * RAD, 160 * RAD, 180 * RAD };
	static const double CL[nabsc] = { 0,    0,    0,      0,        0,       0,       0,       0,        0,     0,       0,       0,       0,       0,       0,       0,    0,   0,   0 };
	static const double CM[nabsc] = { 0,  0,   0,   0,    0,   0,  0, 0,  0,     0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


	for (i = 0; i < nabsc - 1 && AOA[i + 1] < aoa; i++);
	double f = (aoa - AOA[i]) / (AOA[i + 1] - AOA[i]);
	*cl = CL[i] + (CL[i + 1] - CL[i]) * f;  // aoa-dependent lift coefficient
	*cm = CM[i] + (CM[i + 1] - CM[i]) * f;  // aoa-dependent moment coefficient
	double saoa = sin(aoa);
	double pd = 0.5 + 0.01 * saoa * saoa;  // profile drag
	*cd = pd + oapiGetInducedDrag(*cl, 1.05, 0.4);
	// profile drag + (lift-)induced drag
}

void Soyuz7k_T::SetRCSMode(int mode) {
	//transition between DPO and DO thruster groupings
	if (mode == DPO_MODE) {
		SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
		SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
		SetThrusterGroupLevel(THGROUP_ATT_UP, 0);
		SetThrusterGroupLevel(THGROUP_ATT_DOWN, 0);
		SetThrusterGroupLevel(THGROUP_ATT_LEFT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_FORWARD, 0);
		SetThrusterGroupLevel(THGROUP_ATT_BACK, 0);
		SetThrusterGroupLevel(THGROUP_RETRO, 0);
		DelThrusterGroup(THGROUP_ATT_YAWLEFT, false);
		DelThrusterGroup(THGROUP_ATT_YAWRIGHT, false);
		DelThrusterGroup(THGROUP_ATT_PITCHUP, false);
		DelThrusterGroup(THGROUP_ATT_PITCHDOWN, false);
		DelThrusterGroup(THGROUP_ATT_BANKLEFT, false);
		DelThrusterGroup(THGROUP_ATT_BANKRIGHT, false);

		th_group[0] = th_DPO[6];
		th_group[1] = th_DPO[9];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_BANKLEFT);

		th_group[0] = th_DPO[7];
		th_group[1] = th_DPO[8];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_BANKRIGHT);

		th_group[0] = th_DPO[2];
		th_group[1] = th_DPO[5];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_PITCHDOWN);

		th_group[0] = th_DPO[1];
		th_group[1] = th_DPO[4];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_PITCHUP);

		th_group[0] = th_trans_fwd[1];
		th_group[1] = th_retro[0];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_YAWLEFT);

		th_group[0] = th_trans_fwd[0];
		th_group[1] = th_retro[1];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_YAWRIGHT);
		
		th_group[0] = th_retro[0];
		th_group[1] = th_retro[1];
		CreateThrusterGroup(th_group, 2, THGROUP_RETRO);
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_BACK);

		th_group[0] = th_trans_fwd[0];
		th_group[1] = th_trans_fwd[1];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_FORWARD);

		th_group[0] = th_DPO[3];
		th_group[1] = th_correct[4];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_UP);

		th_group[0] = th_DPO[0];
		th_group[1] = th_correct[5];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_DOWN);

		th_group[0] = th_DPO[8];
		th_group[1] = th_DPO[9];
		th_group[2] = th_correct[0];
		th_group[3] = th_correct[1];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_LEFT);

		th_group[0] = th_DPO[6];
		th_group[1] = th_DPO[7];
		th_group[2] = th_correct[2];
		th_group[3] = th_correct[3];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_RIGHT);

		//stable state
		rcs_status = 0;
	}
	else {
		SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
		SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
		SetThrusterGroupLevel(THGROUP_ATT_UP, 0);
		SetThrusterGroupLevel(THGROUP_ATT_DOWN, 0);
		SetThrusterGroupLevel(THGROUP_ATT_LEFT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_FORWARD, 0);
		SetThrusterGroupLevel(THGROUP_ATT_BACK, 0);
		SetThrusterGroupLevel(THGROUP_RETRO, 0);
		DelThrusterGroup(THGROUP_ATT_YAWLEFT, false);
		DelThrusterGroup(THGROUP_ATT_YAWRIGHT, false);
		DelThrusterGroup(THGROUP_ATT_PITCHUP, false);
		DelThrusterGroup(THGROUP_ATT_PITCHDOWN, false);
		DelThrusterGroup(THGROUP_ATT_BANKLEFT, false);
		DelThrusterGroup(THGROUP_ATT_BANKRIGHT, false);
		DelThrusterGroup(THGROUP_ATT_UP, false);
		DelThrusterGroup(THGROUP_ATT_DOWN, false);
		DelThrusterGroup(THGROUP_ATT_LEFT, false);
		DelThrusterGroup(THGROUP_ATT_RIGHT, false);
		DelThrusterGroup(THGROUP_ATT_FORWARD, false);
		DelThrusterGroup(THGROUP_ATT_BACK, false); 
		DelThrusterGroup(THGROUP_RETRO, false); 

		th_group[0] = th_DO[4];
		th_group[1] = th_DO[5];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_BANKLEFT);

		th_group[0] = th_DO[2];
		th_group[1] = th_DO[3];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_BANKRIGHT);

		th_group[0] = th_DO[1];
		CreateThrusterGroup(th_group, 1, THGROUP_ATT_PITCHDOWN);

		th_group[0] = th_DO[0];
		CreateThrusterGroup(th_group, 1, THGROUP_ATT_PITCHUP);

		th_group[0] = th_DO[6];
		CreateThrusterGroup(th_group, 1, THGROUP_ATT_YAWLEFT);

		th_group[0] = th_DO[7];
		CreateThrusterGroup(th_group, 1, THGROUP_ATT_YAWRIGHT);
		
		//stable state
		rcs_status = 2;
	}
}

void Soyuz7k_T::JettisonModule() {

	if (sep_status == 1) {
		char name[256];
		if (CM_jett == 0) {
			DelDock(hDock);
			strcpy(name, GetName());
		}
		else { 
			strcpy(name, "noCM_");
			strcat(name, GetName());
		}
		strcat(name, "_BO");
		VESSELSTATUS vs;
		GetStatus(vs);
		VECTOR3 rvel;
		Local2Rel(_V(0, 0, 3.15), vs.rpos);
		GlobalRot(_V(0, 0, 0.82), rvel);
		vs.rvel = vs.rvel + rvel;
		vs.eng_main = vs.eng_hovr = 0.0;
		vs.status = 0;
		vs.vrot.x = vs.vrot.x + (0.5 * RAD);
		vs.vrot.y = vs.vrot.y;
		vs.vrot.z = vs.vrot.z + (1.0 * RAD);
		OBJHANDLE hBO;
		hBO = oapiCreateVessel(name, "Soyuz_7k\\Soyuz7k_T_BO", vs);
		
		m_pXRSound->PlayWav(ValveBlow, false, 1.0);
		
		DelMesh(mesh[2]);
		sep_status = 2;
		SetSize(S7K_SIZE - 1.5);
		SetEmptyMass(S7K_EMPTYMASS - 1350);
		ShiftCG(_V(0, 0, -0.5));
		mesh_shift_z = 1.13;
		DelBeacon(&beac1);
		DelBeacon(&beac2);
		DelBeacon(&beac6);
		DelLightEmitter(spotlight1);
		DelLightEmitter(spotlight2);
		DelAttachment(attach1);
		DelAnimation(anim_igla_sides);
		if (CM_jett == 0) DelAnimation(anim_dock_probe);
		DelAnimation(anim_igla_gimbal);
		

		//update the vzor camera positions
		if (gcEnabled()) {
			//setup cameras position and nadir direction 
			VECTOR3 campos = { 0.280216, -1.47587, 0.5 + mesh_shift_z };
			VECTOR3 camdir = { 0, -0.994522, 0.104528 };
			VECTOR3 camup = { 0, 0.104528, 0.994522 };
			hCam_vzor_nad = gcSetupCustomCamera(hCam_vzor_nad, GetHandle(), campos, camdir, camup, 7.5 * RAD, hSurfVzor);
			hCam_vzor_periph = gcSetupCustomCamera(hCam_vzor_periph, GetHandle(), campos, camdir, camup, 77 * RAD, hSurfVzor3);
			//setup forward direction
			camdir = { 0, 0, 1 };
			camup = { 0, 1, 0 };
			hCam_vzor_fwd = gcSetupCustomCamera(hCam_vzor_fwd, GetHandle(), campos, camdir, camup, 7.5 * RAD, hSurfVzor2);

			if (vzor_status == 3) {
				gcCustomCameraOnOff(hCam_vzor_nad, true);
				gcCustomCameraOnOff(hCam_vzor_fwd, false);
			}
			else if (vzor_status == 2) {
				gcCustomCameraOnOff(hCam_vzor_fwd, true);
				gcCustomCameraOnOff(hCam_vzor_nad, false);
			}
		}
	}
	else if (sep_status == 3) {

		OBJHANDLE hPAO, hVzor, hCabling;
		
		char name1[256];
		strcpy(name1, GetName());
		strcat(name1, "_PAO");
		VESSELSTATUS vs1;
		GetStatus(vs1);
		VECTOR3 rvel1;
		Local2Rel(_V(0, 0, -1.35), vs1.rpos);
		GlobalRot(_V(0, 0, -0.58), rvel1);
		vs1.rvel = vs1.rvel + rvel1;
		vs1.eng_main = vs1.eng_hovr = 0.0;
		vs1.status = 0;
		vs1.vrot.x = vs1.vrot.x - (0.1 * RAD);
		vs1.vrot.y = vs1.vrot.y;
		vs1.vrot.z = vs1.vrot.z - (1.0 * RAD);
		hPAO = oapiCreateVessel(name1, "Soyuz_7k\\Soyuz7k_T_PAO", vs1);
		
		char name2[256];
		strcpy(name2, GetName());
		strcat(name2, "_Vzor");
		VESSELSTATUS vs2;
		GetStatus(vs2);
		VECTOR3 rvel2;
		Local2Rel(_V(-0.5, -1.5, 1.1), vs2.rpos);
		GlobalRot(_V(0, -5.0, 0), rvel2);
		vs2.rvel = vs2.rvel + rvel2;
		vs2.eng_main = vs2.eng_hovr = 0.0;
		vs2.status = 0;
		vs2.vrot.x = vs2.vrot.x;
		vs2.vrot.y = vs2.vrot.y;
		vs2.vrot.z = vs2.vrot.z - (1.0 * RAD);
		hVzor = oapiCreateVessel(name2, "Soyuz_7k\\Soyuz7k_T_Vzor", vs2);

		char name3[256];
		strcpy(name3, GetName());
		strcat(name3, "_cabling");
		VESSELSTATUS vs3;
		GetStatus(vs3);
		VECTOR3 rvel3;
		Local2Rel(_V(0, -1.1, 1.1), vs3.rpos);
		GlobalRot(_V(0, -5.4, 0), rvel3);
		vs3.rvel = vs3.rvel + rvel3;
		vs3.eng_main = vs3.eng_hovr = 0.0;
		vs3.status = 0;
		vs3.vrot.x = vs3.vrot.x - (6.0 * RAD);
		vs3.vrot.y = vs3.vrot.y;
		vs3.vrot.z = vs3.vrot.z - (0.5 * RAD);
		hCabling = oapiCreateVessel(name3, "Soyuz_7k\\Soyuz7k_T_cabling", vs3);

		m_pXRSound->PlayWav(ValveBlow, false, 1.0);

		DelMesh(mesh[0]);
		DelMesh(mesh[3]);
		DelMesh(mesh[4]);
		DelMesh(mesh[5]);
		DelMesh(mesh[6]);
		DelMesh(mesh[7]);
		DelMesh(mesh[8]);
		DelMesh(mesh[9]);
		DelMesh(mesh[10]);
		DelMesh(mesh[11]);
		DelMesh(mesh[12]);
		DelAnimation(anim_ion);
		DelBeacon(&beac3);
		DelBeacon(&beac4);
		sep_status = 4;

		//delete no longer needed Vzor cameras
		if (gcEnabled()) {
			gcDeleteCustomCamera(hCam_vzor_fwd);
			gcDeleteCustomCamera(hCam_vzor_nad);
			gcDeleteCustomCamera(hCam_vzor_periph);
		}

		//battery reconfig
		BB_status = 0;
		BB_power = 0;
		SA_BB_status = 1;
		SA_BB_power = MAX_SA_POWER;
		RB_status = 0;
		RB_power = 0.0;
		ReconfigureSA();
		SUS_status = 1;
	}
	else if (sep_status == 5) {
		
		OBJHANDLE hHeatshield;

		char name1[256];
		strcpy(name1, GetName());
		strcat(name1, "_heatshield");
		VESSELSTATUS vs1;
		GetStatus(vs1);
		VECTOR3 rvel1;
		Local2Rel(_V(0, -0.09, 0.48), vs1.rpos);
		GlobalRot(_V(0, 0, -0.6), rvel1);
		vs1.rvel = vs1.rvel + rvel1;
		vs1.eng_main = vs1.eng_hovr = 0.0;
		vs1.status = 0;
		vs1.vrot.x = vs1.vrot.x;
		vs1.vrot.y = vs1.vrot.y;
		vs1.vrot.z = vs1.vrot.z;
		hHeatshield = oapiCreateVessel(name1, "Soyuz_7k\\Soyuz7k_T_heatshield", vs1);

		m_pXRSound->PlayWav(ValveBlow, false, 1.0);

		DelMesh(mesh[13]);
		SetEmptyMass(GetEmptyMass() - 400);
		sep_status = 6;
	}
	else sep_status = 0;
}

void Soyuz7k_T::ReconfigureSA() {
	// reconfigures the vessel for descent module solo flight

	SetSize(2.5);
	SetEmptyMass(3055);
	
	SetPMI(S7K_PMI_SA);
	SetCrossSections(S7K_CS_SA);
	SetRotDrag(S7K_RD);

	DelAnimation(anim_antennae);

	DefineAnimations(1);

	rcs_status = 4;

	combined_power = 0;
	recharge = 0;

	// modes off
	BDUS_status = BDUS_OFF;
	pulsed_status = 2;
	if (ori_ap_status != 0 || inertial_status != 0 || modes_off != 1) {
		ori_ap_status = 0;		//switch orbital orientation mode off, regardless of current state
		ori_ap_aux = 0;
		inertial_status = 0;	//switch inertial orientation mode off, regardless of current state
		accel_z_status = 0;		//switch off longitudinal accelerometer
		accel_z = 0;			//reset integrating accelerometer
		SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
		SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
	}

	//AKSP is armed upon Separation command
	if (!AKSP_status) AKSP_status = 1;

	DelThrusterGroup(THGROUP_ATT_YAWLEFT, true);
	DelThrusterGroup(THGROUP_ATT_YAWRIGHT, true);
	DelThrusterGroup(THGROUP_ATT_PITCHUP, true);
	DelThrusterGroup(THGROUP_ATT_PITCHDOWN, true);
	DelThrusterGroup(THGROUP_ATT_BANKLEFT, true);
	DelThrusterGroup(THGROUP_ATT_BANKRIGHT, true);
	DelThrusterGroup(THGROUP_ATT_UP, true);
	DelThrusterGroup(THGROUP_ATT_DOWN, true);
	DelThrusterGroup(THGROUP_ATT_LEFT, true);
	DelThrusterGroup(THGROUP_ATT_RIGHT, true);
	DelThrusterGroup(THGROUP_ATT_FORWARD, true);
	DelThrusterGroup(THGROUP_ATT_BACK, true);
	DelThrusterGroup(THGROUP_MAIN, true);
	DelThrusterGroup(THGROUP_RETRO, true);

	DelThruster(th_main[0]);
	DelThruster(th_main[1]);
	DelThruster(th_retro[0]);
	DelThruster(th_retro[1]);

	for (int i = 0; i < 4; i++) {
		DelThruster(th_trans_fwd[i]);
	}

	for (int i = 0; i < 10; i++) {
		DelThruster(th_DPO[i]);
	}

	for (int i = 0; i < 18; i++) {
		DelThruster(th_DO[i]);
	}

	for (int i = 0; i < 6; i++) {
		DelThruster(th_correct[i]);
	}

	DelPropellantResource(hpr);
	DelPropellantResource(hpr_dpo);
	DelPropellantResource(hpr_do);
	hpr_sa = CreatePropellantResource(S7K_FUELSAMASS);
	hpr_land = CreatePropellantResource(7);

	mesh_shift_y = 0.0;
	mesh_shift_z = 0.7;
	ShiftCG(_V(0, mesh_shift_y, mesh_shift_z));
	mesh_shift_z = mesh_shift_z - 0.27;
	
	
	//soft land
	th_land = CreateThruster(_V(0, 0, -0.5), _V(0, 0, 1), 45000, hpr_land, S7K_ISP);
	
	//AddExhaust(th_land, 50, 3, th_tex);
	SURFHANDLE tex2 = oapiRegisterExhaustTexture("Exhaust2");
	landing_exhaust.tex = oapiRegisterParticleTexture("Contrail2");
	
	AddExhaust(th_land, 1.6, 0.2, _V(0.7, 0.5, -0.5), _V(0, 0, -1), tex2);
	AddExhaust(th_land, 1.6, 0.2, _V(-0.7, 0.5, -0.5), _V(0, 0, -1), tex2);
	AddExhaust(th_land, 1.6, 0.2, _V(-0.7, -0.5, -0.5), _V(0, 0, -1), tex2);
	AddExhaust(th_land, 1.6, 0.2, _V(0.7, -0.5, -0.5), _V(0, 0, -1), tex2);
	
	AddExhaustStream(th_land, _V(0, 0, -0.5), &landing_contrail);
	AddExhaustStream(th_land, _V(0, 0, -0.5), &landing_exhaust);

	CreateThrusterGroup(&th_land, 1, THGROUP_MAIN);

	//pitch
	th_SA[0] = CreateThruster(_V(0, -0.68, 1.54), _V(0, -0.743, -0.669), S7K_MAXSADOTH, hpr_sa, S7K_SIOISP);
	th_SA[1] = CreateThruster(_V(0, -0.815, 1.41), _V(0, 0.743, 0.669), S7K_MAXSADOTH, hpr_sa, S7K_SIOISP);

	//yaw 
	//th_SA[2] = CreateThruster(_V(0.3, -0.535, 1.505), _V(-0.99, -0.099, 0.099), S7K_MAXSADOTH, hpr_sa, S7K_SIOISP);
	th_SA[2] = CreateThruster(_V(0.3, -0.535, 1.505), _V(-1, 0, 0), S7K_MAXSADOTH, hpr_sa, S7K_SIOISP);
	//th_SA[3] = CreateThruster(_V(-0.3, -0.535, 1.505), _V(0.99, -0.099, 0.099), S7K_MAXSADOTH, hpr_sa, S7K_SIOISP);
	th_SA[3] = CreateThruster(_V(-0.3, -0.535, 1.505), _V(1, 0, 0), S7K_MAXSADOTH, hpr_sa, S7K_SIOISP);

	//roll (0.371, -0.928, -0.363) unnorm
	//th_SA[4] = CreateThruster(_V(1.03, 0, 0), _V(0, -0.939921, -0.341392), S7K_MAXSADOTH, hpr_sa, S7K_SIOISP);
	th_SA[4] = CreateThruster(_V(1.03, 0, 0), _V(0, -0.934069, -0.357093), S7K_MAXSADOTH, hpr_sa, S7K_SIOISP);
	//th_SA[5] = CreateThruster(_V(-1.03, 0, 0), _V(0, -0.939921, -0.341392), S7K_MAXSADOTH, hpr_sa, S7K_SIOISP);
	th_SA[5] = CreateThruster(_V(-1.03, 0, 0), _V(0, -0.934069, -0.357093), S7K_MAXSADOTH, hpr_sa, S7K_SIOISP);

	for (int i = 0; i < 6; i++) {
		AddExhaustStream(th_SA[i], &DO_exhaust);
	}

	CreateThrusterGroup(&th_SA[0], 1, THGROUP_ATT_PITCHDOWN);
	CreateThrusterGroup(&th_SA[1], 1, THGROUP_ATT_PITCHUP);
	CreateThrusterGroup(&th_SA[2], 1, THGROUP_ATT_YAWLEFT);
	CreateThrusterGroup(&th_SA[3], 1, THGROUP_ATT_YAWRIGHT);
	CreateThrusterGroup(&th_SA[4], 1, THGROUP_ATT_BANKRIGHT);
	CreateThrusterGroup(&th_SA[5], 1, THGROUP_ATT_BANKLEFT);
	
	// airfoil definitions
	DelAirfoil(airfoil_v);
	DelAirfoil(airfoil_h);
	airfoil_v = CreateAirfoil2(LIFT_VERTICAL, _V(0, -mesh_shift_y, 0), vliftPre, 2.17, 3.82, 1.05);
	airfoil_h = CreateAirfoil2(LIFT_HORIZONTAL, _V(0, 0, 0), hlift, 2.17, 3.82, 1.05);
}

void Soyuz7k_T::IglaOff() {
	igla_status = 0;
	igla_ap_aux_lat = 0;
	igla_ap_aux_vert = 0;
	igla_ap_aux_pitch = 0;
	igla_ap_aux_yaw = 0;
	igla_ap_aux_roll = 0;
	igla_ap_lat = 0;
	igla_ap_vert = 0;
	igla_ap_pitch = 0;
	igla_ap_yaw = 0;
	igla_ap_roll = 0;
	igla_idle = 1;
	burn_seq = 0;
	SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
	SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
	SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
	SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
	SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
	SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
	SetThrusterGroupLevel(THGROUP_ATT_UP, 0);
	SetThrusterGroupLevel(THGROUP_ATT_DOWN, 0);
	SetThrusterGroupLevel(THGROUP_ATT_LEFT, 0);
	SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 0);
	SetThrusterGroupLevel(THGROUP_ATT_FORWARD, 0);
	SetThrusterGroupLevel(THGROUP_RETRO, 0);
	SetAnimation(anim_igla_gimbal, 0);
	if (!man_appr) target_Vessel->ActivateIgla(0);
}

void Soyuz7k_T::IglaData(double SimDT) {
	// calculates essential data for Igla rendezvous and approach


	//Get target vessel position in ecliptic
	if (target_Vessel != NULL) {
		target_Vessel->Local2Global(_V(0, 0, 0), tgt_globpos);		//P1 in global
		target_Vessel->Local2Global(_V(-1.5, 0, 0), tgt_globPRef);	//P2 in global

	}
	//Target position transformed to local Soyuz coordinates
	Global2Local(tgt_globpos, tgt_relpos);		//P1 in local
	Global2Local(tgt_globPRef, tgt_relPRef);	//P2 in local
	
	//Range as vector magnitude
	tgt_range = sqrtl(pow(tgt_relpos.x, 2) + pow(tgt_relpos.y, 2) + pow(tgt_relpos.z, 2));
	
	if (GetPropellantMass(hpr_dpo) == 0)
		IglaOff();

	//Igla range is limited
	if (tgt_range > IGLA_RANGE || target_Vessel == NULL) {
		igla_status = 0;
		man_appr = 0;
	}
	else if (igla_status > 2) {

		switch (burn_seq) {


		case 0:
			if (tgt_range > 8000 && igla_LOS_burn == 1) {
				speed_target = -24;
				if (tgt_relvel.z > -15) burn_seq = 9;
				else burn_seq = 10;
			}
			else if (tgt_range > 4000 && tgt_range <= 8000) {
				speed_target = -11;
				if (tgt_relvel.z > -7) burn_seq = 1;
				else {
					burn_seq = 2;
					igla_LOS_burn = 2;
				}
			}
			else if (tgt_range > 200 && tgt_range <= 4000) {
				speed_target = -2.0;
				burn_seq = 1;
			}
			else if (target_oriented == 2 && tgt_range <= 200) {
				speed_target = -0.30;
				burn_seq = 1;
			}
			break;

		case 2:
			if ((tgt_range <= 2500 && abs(speed_target) == 11 && tgt_relvel.z < -4.0 && igla_status == 4) || (tgt_range <= 2500 && abs(speed_target) == 11 && tgt_relvel.y < -3.0 && (igla_status == 11 || igla_status == 13))) {
				speed_target = 2;
				burn_seq = 3;
			}
			else if (tgt_range <= 2500 && tgt_relvel.z < 0) burn_seq = 4;
			break;

		case 4:
			if (tgt_range <= 1500 && tgt_relvel.z < -2.0) {
				speed_target = -2.0;
				burn_seq = 5;
			}
			else if (tgt_range <= 800 && tgt_relvel.z < 0) burn_seq = 6;
			else if (tgt_range <= 1500 && tgt_relvel.z >= -0.5) {
				burn_seq = 13;
				speed_target = -2.0;
			}
			break;

		case 6:
			if (tgt_range <= 200 && tgt_relvel.z < -0.30) {
				speed_target = -0.30;
				burn_seq = 7;
			}
			break;
		
		case 8:
			//fine tuning of speed on close approach
			if (tgt_range < 100 && tgt_relvel.z < -0.35 && igla_idle == 1) burn_seq = 6;
			break;

		case 10:
			if ((tgt_range <= 6500 && speed_target == -24 && tgt_relvel.z < -12.0 && igla_status == 4) || (tgt_range <= 6500 && speed_target == -24 && tgt_relvel.y < -12.0 && (igla_status == 11 || igla_status == 13))) {
				speed_target = 11;
				burn_seq = 11;
			}
			break;
		}
	}
	
	//Relative orientation
	tgt_unitvect.x = tgt_relpos.x / tgt_range;
	tgt_unitvect.y = tgt_relpos.y / tgt_range;
	tgt_unitvect.z = tgt_relpos.z / tgt_range;

	tgt_err_yaw = asin(-tgt_unitvect.x) * DEG;
	tgt_err_pitch = asin(tgt_unitvect.y) * DEG;
	tgt_err_roll = asin((tgt_relPRef.y - tgt_relpos.y) / 1.5) * DEG;

	//if target is behind
	if (tgt_unitvect.z < 0) {
		if (tgt_unitvect.x < 0) tgt_err_yaw = 180 - tgt_err_yaw;
		else tgt_err_yaw = -180 - tgt_err_yaw;

		if (tgt_unitvect.y > 0) tgt_err_pitch = 180 - tgt_err_pitch;
		else tgt_err_pitch = -180 - tgt_err_pitch;
	}

	//roll special cases
	if (tgt_relpos.x > tgt_relPRef.x) {
		if (tgt_relpos.y <= tgt_relPRef.y) tgt_err_roll = 180 - tgt_err_roll;
		else tgt_err_roll = -180 - tgt_err_roll;
	}


	//Calculate relative velocity on all axes, sampled
	if (relvel_sample == 2) {
		
		//distance travelled over accumulated time
		tgt_relvel.x = (tgt_relpos.x - last_tgtpos.x) / acc_SimDT;
		tgt_relvel.y = (tgt_relpos.y - last_tgtpos.y) / acc_SimDT;
		tgt_relvel.z = (tgt_relpos.z - last_tgtpos.z) / acc_SimDT;

		//Range rate as velocity vector magnitude
		tgt_range_rate = sqrtl(pow(tgt_relvel.x, 2) + pow(tgt_relvel.y, 2) + pow(tgt_relvel.z, 2));

		//Reset
		last_tgtpos = tgt_relpos;
		acc_SimDT = 0;
		relvel_sample = 0;
	}
	relvel_sample++;
	acc_SimDT += SimDT;

	if (tgt_range > 1000) {
		//get motion of LOS angle to +y
		VECTOR3 LOS_motion_norm = tgt_relvel;
		LOS_motion_norm.z = 0;
		normalise(LOS_motion_norm);
		if (tgt_err_pitch < 10 && tgt_err_pitch > -10 && tgt_err_yaw < 10 && tgt_err_yaw > -10) roll_drift_vector = acos(dotp(_V(LOS_motion_norm.x, LOS_motion_norm.y, 0), _V(0, -1, 0))) * DEG;
	}

	if (!ssvp_power && tgt_range < 45.0) {
		ssvp_power = 1;
	}

	if (igla_status == 12 && igla_ap_aux_roll > 0) {
		intz += avel.z * SimDT;
	}

	//Get target docking port position in ecliptic
	if (target_Vessel != NULL) target_Vessel->Local2Global(_V(0, 0, 7.514), tgt_globpos);

	//Target position transformed to local Soyuz coordinates
	Global2Local(tgt_globpos, tgt_relpos);

	tgt_relpos.z -= 4.4;
	tgt_range = sqrtl(pow(tgt_relpos.x, 2) + pow(tgt_relpos.y, 2) + pow(tgt_relpos.z, 2));

	if (tgt_range_rate > 2 && IRS_speed_scale != 40) {
		IRS_speed_scale = 39;	// 40-1; act as transition flag
	}
	else if (tgt_range_rate <= 2 && IRS_speed_scale != 2) {
		IRS_speed_scale = 1;	// 2-1; act as transition flag
	}

	if (tgt_range <= 500 && IRS_range_scale != 500) {
		IRS_range_scale = 498;
	}
	else if (tgt_range > 500 && tgt_range <= 5000 && IRS_range_scale != 5000) {
		IRS_range_scale = 4999;
	}
	else if (tgt_range > 5000 && tgt_range <= 50000 && IRS_range_scale != 50000) {
		IRS_range_scale = 49999;
	}

	// igla antenna gimballing towards target
	if (igla_status > 2 && tgt_err_pitch >= 0) {
		igla_gimbal_proc = tgt_err_pitch / 180;
		SetAnimation(anim_igla_gimbal, igla_gimbal_proc);
	}
	else if (igla_status > 2 && tgt_err_pitch >= -180 && tgt_err_pitch < -175) SetAnimation(anim_igla_gimbal, 1.0);
	else SetAnimation(anim_igla_gimbal, 0);

	
}

void Soyuz7k_T::Igla() {
	// Igla autopilot proper, sequential execution

	//pitch flag 1 is down, flag 2 is up, flag 0 is off
	//yaw flag 1 is right, flag 2 is left, flag 0 is off
	

	//Orientation
	//neg pitch, pitch down
	//neg yaw, yaw right
	if (igla_status == 1){
		
		if (igla_ap_aux_yaw == 0 && tgt_err_yaw > -2 && tgt_err_yaw < 2) igla_ap_aux_yaw = 4;
		

		//start yaw orientation
		if (igla_ap_aux_yaw == 0 && tgt_err_yaw < -2.0) {
			SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
			igla_ap_yaw = 1;
			igla_ap_aux_yaw = 1;
			igla_idle = 0;
		}
		else if (igla_ap_aux_yaw == 0 && tgt_err_yaw > 2.0) {
			SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
			igla_ap_yaw = 2;
			igla_ap_aux_yaw = 1;
			igla_idle = 0;
		}

		//runtime
		if (igla_ap_aux_yaw == 1) {
			if (igla_ap_yaw == 1) SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
			else if (igla_ap_yaw == 2) SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);

			//stop conditions
			if (igla_ap_yaw == 1 && avel.y <= -0.66) {
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				igla_ap_yaw = 0;
				igla_ap_aux_yaw = 2;
				igla_idle = 1;
			}
			else if (igla_ap_yaw == 2 && avel.y >= 0.66) {
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				igla_ap_yaw = 0;
				igla_ap_aux_yaw = 2;
				igla_idle = 1;
			}
			else if (tgt_err_yaw > -3 && tgt_err_yaw < 3) {
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				igla_ap_aux_yaw = 2;
				igla_idle = 1;
			}
		}

		if (igla_ap_aux_yaw == 2 && tgt_err_yaw > -3.2 && tgt_err_yaw < 3.2) {
			ActivateNavmode(1);
			igla_ap_aux_yaw = 3;
			igla_idle = 0;
		}

		if (igla_ap_aux_yaw == 3 && GetNavmodeState(1) == false) {
			igla_ap_aux_yaw = 4;
			igla_idle = 1;
		}

		if (igla_ap_aux_yaw == 4) {
			if (igla_ap_aux_pitch == 0 && tgt_err_pitch > -2 && tgt_err_pitch < 2) {
				igla_ap_aux_pitch = 4;
				igla_ap_aux_yaw = 5;
			}

			if (igla_ap_aux_pitch == 0 && tgt_err_pitch < -2.0) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
				igla_ap_pitch = 1;
				igla_ap_aux_pitch = 1;
				igla_idle = 0;
			}
			else if (igla_ap_aux_pitch == 0 && tgt_err_pitch > 2.0) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
				igla_ap_pitch = 2;
				igla_ap_aux_pitch = 1;
				igla_idle = 0;
			}

			if (igla_ap_aux_pitch == 1) {
				if (igla_ap_pitch == 1) SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
				else if (igla_ap_pitch == 2) SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);

				//stop conditions
				if (igla_ap_pitch == 1 && avel.x <= -0.66) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					igla_ap_pitch = 0;
					igla_ap_aux_pitch = 2;
					igla_idle = 1;
				}
				else if (igla_ap_pitch == 2 && avel.x >= 0.66) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					igla_ap_pitch = 0;
					igla_ap_aux_pitch = 2;
					igla_idle = 1;
				}
				else if (tgt_err_pitch > -2.4 && tgt_err_pitch < 2.4) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					igla_ap_aux_pitch = 2;
					igla_idle = 1;
				}
			}
			
			if (igla_ap_aux_pitch == 2 && tgt_err_pitch > -3.5 && tgt_err_pitch < 3.5) {
				ActivateNavmode(1);
				igla_ap_aux_pitch = 3;
				igla_idle = 0;
			}

			if (igla_ap_aux_pitch == 3 && GetNavmodeState(1) == false) {
				igla_ap_aux_pitch = 4;
				igla_ap_aux_yaw = 5;
				igla_idle = 1;
			}

		}

		if (igla_ap_aux_pitch == 4 && igla_ap_aux_yaw == 5) {
			igla_status = 2;
			igla_ap_aux_pitch = 0;
			igla_ap_aux_yaw = 0;
		}
	}
	//Activate Igla on target vessel
	else if (igla_status == 2) {
		target_Vessel->ActivateIgla(1);
		if (tgt_range < 2000) {
			igla_status = 3;	//proceed with DPO approach
			igla_LOS_burn = 3;
		}
		else {
			rcs_status = 3;
			igla_status = 12;	//transition to roll align for motion of line of sight cancel with SKDU
		}
	}
	
	//translation null
	//+x -> trans right
	//+y -> trans up
	else if (igla_status == 3) {
		
		if ((igla_ap_aux_vert == 0 && abs(tgt_relvel.y) <= 0.014) || GetPropellantMass(hpr_dpo) == 0) igla_ap_aux_vert = 2;
		if ((igla_ap_aux_lat == 0 && abs(tgt_relvel.x) <= 0.014) || GetPropellantMass(hpr_dpo) == 0) igla_ap_aux_lat = 2;

		if (igla_ap_aux_vert == 0 && tgt_relvel.y > 0) {
			SetThrusterGroupLevel(THGROUP_ATT_UP, 1);
			igla_ap_vert = 1;
			igla_ap_aux_vert = 1;
		}
		else if (igla_ap_aux_vert == 0 && tgt_relvel.y < 0) {
			SetThrusterGroupLevel(THGROUP_ATT_DOWN, 1);
			igla_ap_vert = 2;
			igla_ap_aux_vert = 1;
		}
		if (igla_ap_aux_lat == 0 && tgt_relvel.x > 0) {
			SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 1);
			igla_ap_lat = 1;
			igla_ap_aux_lat = 1;
		}
		else if (igla_ap_aux_lat == 0 && tgt_relvel.x < 0) {
			SetThrusterGroupLevel(THGROUP_ATT_LEFT, 1);
			igla_ap_lat = 2;
			igla_ap_aux_lat = 1;
		}

		if (igla_ap_aux_vert == 1) {
			if (igla_ap_vert == 1) SetThrusterGroupLevel(THGROUP_ATT_UP, 1);
			else if (igla_ap_vert == 2) SetThrusterGroupLevel(THGROUP_ATT_DOWN, 1);
		
			//stop conditions
			if (igla_ap_vert == 1 && tgt_relvel.y <= 0) {
				SetThrusterGroupLevel(THGROUP_ATT_UP, 0);
				igla_ap_vert = 0;
				igla_ap_aux_vert = 2;
			}
			else if (igla_ap_vert == 2 && tgt_relvel.y >= 0) {
				SetThrusterGroupLevel(THGROUP_ATT_DOWN, 0);
				igla_ap_vert = 0;
				igla_ap_aux_vert = 2;
			}
		}
		if (igla_ap_aux_lat == 1) {
			if (igla_ap_lat == 1) SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 1);
			else if (igla_ap_lat == 2) SetThrusterGroupLevel(THGROUP_ATT_LEFT, 1);

			//stop conditions
			if (igla_ap_lat == 1 && tgt_relvel.x <= 0) {
				SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 0);
				igla_ap_lat = 0;
				igla_ap_aux_lat = 2;
			}
			else if (igla_ap_lat == 2 && tgt_relvel.x >= 0) {
				SetThrusterGroupLevel(THGROUP_ATT_LEFT, 0);
				igla_ap_lat = 0;
				igla_ap_aux_lat = 2;
			}
		}

		if (igla_ap_aux_vert == 2 && igla_ap_aux_lat == 2) {
			igla_status = 4;
			igla_ap_aux_vert = 0;
			igla_ap_aux_lat = 0;
		}

	}
	else if (igla_status == 4) {

		if (igla_ap_aux_yaw == 0 && tgt_err_yaw > -igla_4_limit && tgt_err_yaw < igla_4_limit) igla_ap_aux_yaw = 4;

		//start yaw orientation
		if (igla_ap_aux_yaw == 0 && tgt_err_yaw < -igla_4_limit) {
			SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
			igla_ap_yaw = 1;
			igla_ap_aux_yaw = 1;
			last_tgt_err_yaw = tgt_err_yaw;
			igla_idle = 0;
		}
		else if (igla_ap_aux_yaw == 0 && tgt_err_yaw > igla_4_limit) {
			SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
			igla_ap_yaw = 2;
			igla_ap_aux_yaw = 1;
			last_tgt_err_yaw = tgt_err_yaw;
			igla_idle = 0;
		}

		//runtime
		if (igla_ap_aux_yaw == 1) {
			if (igla_ap_yaw == 1) SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
			else if (igla_ap_yaw == 2) SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);

			//stop conditions
			if (igla_ap_yaw == 1 && avel.y <= -0.66) {
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				igla_ap_yaw = 0;
				igla_ap_aux_yaw = 2;
				igla_idle = 1;
			}
			else if (igla_ap_yaw == 2 && avel.y >= 0.66) {
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				igla_ap_yaw = 0;
				igla_ap_aux_yaw = 2;
				igla_idle = 1;
			}
			else if ((tgt_err_yaw > (last_tgt_err_yaw / 2) && last_tgt_err_yaw < 0) || (tgt_err_yaw < (last_tgt_err_yaw / 2) && last_tgt_err_yaw > 0)) {
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				igla_ap_aux_yaw = 2;
				igla_ap_yaw = 0;
				igla_idle = 1;
			}
		}

		if (igla_ap_aux_yaw == 2) {

			if (igla_ap_yaw == 0 && tgt_err_yaw > -3.2 && tgt_err_yaw < 3.2) {
				igla_ap_yaw = 3;
			}

			else if (igla_ap_yaw == 3) {
				ActivateNavmode(1);
				igla_ap_aux_yaw = 3;
				igla_idle = 0;
			}
		}

		if (igla_ap_aux_yaw == 3 && GetNavmodeState(1) == false) {
			igla_ap_aux_yaw = 4;
			igla_idle = 1;
		}

		if (igla_ap_aux_yaw == 4) {
			if (igla_ap_aux_pitch == 0 && tgt_err_pitch > -igla_4_limit && tgt_err_pitch < igla_4_limit) {
				igla_ap_aux_pitch = 4;
				igla_ap_aux_yaw = 5;
			}

			if (igla_ap_aux_pitch == 0 && tgt_err_pitch < -igla_4_limit) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
				igla_ap_pitch = 1;
				igla_ap_aux_pitch = 1;
				last_tgt_err_pitch = tgt_err_pitch;
				igla_idle = 0;
			}
			else if (igla_ap_aux_pitch == 0 && tgt_err_pitch > igla_4_limit) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
				igla_ap_pitch = 2;
				igla_ap_aux_pitch = 1;
				last_tgt_err_pitch = tgt_err_pitch;
				igla_idle = 0;
			}

			if (igla_ap_aux_pitch == 1) {
				if (igla_ap_pitch == 1) SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
				else if (igla_ap_pitch == 2) SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);

				//stop conditions
				if (igla_ap_pitch == 1 && avel.x <= -0.66) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					igla_ap_pitch = 0;
					igla_ap_aux_pitch = 2;
					igla_idle = 1;
				}
				else if (igla_ap_pitch == 2 && avel.x >= 0.66) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					igla_ap_pitch = 0;
					igla_ap_aux_pitch = 2;
					igla_idle = 1;
				}
				else if ((tgt_err_pitch > (last_tgt_err_pitch / 2) && last_tgt_err_pitch < 0) || (tgt_err_pitch < (last_tgt_err_pitch / 2) && last_tgt_err_pitch > 0)) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					igla_ap_aux_pitch = 2;
					igla_ap_pitch = 3;
					igla_idle = 1;
				}
			}

			if (igla_ap_aux_pitch == 2) {

				if (igla_ap_pitch == 0 && tgt_err_pitch > -3.5 && tgt_err_pitch < 3.5) {
					igla_ap_pitch = 3;
				}

				else if (igla_ap_pitch == 3) {
					ActivateNavmode(1);
					igla_ap_aux_pitch = 3;
					igla_idle = 0;
				}
			}

			if (igla_ap_aux_pitch == 3 && GetNavmodeState(1) == false) {
				igla_ap_aux_pitch = 4;
				igla_ap_aux_yaw = 5;
				igla_idle = 1;
			}

		}

		if (igla_ap_aux_pitch == 4 && igla_ap_aux_yaw == 5) {
			if (target_oriented == 2 && tgt_range < 2000 && (tgt_err_roll < -0.5 || tgt_err_roll > 0.5)) {
				igla_status = 5;
				rcs_status = 3;
			}
			else if (burn_seq == 9 || burn_seq == 1 || burn_seq == 5 || burn_seq == 7 || burn_seq == 13) igla_status = 7;
			else if (burn_seq == 11 || burn_seq == 3) {
				igla_status = 6;
				rcs_status = 3;
			}
			else if ((burn_seq == 0 && igla_LOS_burn == 0) || (burn_seq == 2 && igla_LOS_burn == 2) || (burn_seq == 10 && igla_LOS_burn == 1 && tgt_range <= 11000)) {
				if (tgt_relvel.y < 0.5) {
					igla_status = 9;
					rcs_status = 3;
				}
				else igla_LOS_burn++;
			}
			else if (igla_LOS_burn == 3) igla_status = 3;		//track this condition, LOS_burn 4 if needed
			else {
				igla_status = 12;
				rcs_status = 3;
			}
			igla_ap_aux_pitch = 0;
			igla_ap_aux_yaw = 0;
		}

	}
	else if (igla_status == 5) {
	
		//start roll orientation
		if (igla_ap_aux_roll == 0 && tgt_err_roll < -0.5) {
			SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
			igla_ap_roll = 1;
			igla_ap_aux_roll = 1;
			last_tgt_err_roll = tgt_err_roll;
			igla_idle = 0;
		}
		else if (igla_ap_aux_roll == 0 && tgt_err_roll > 0.5) {
			SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
			igla_ap_roll = 2;
			igla_ap_aux_roll = 1;
			last_tgt_err_roll = tgt_err_roll;
			igla_idle = 0;
		}
		
		//runtime
		if (igla_ap_aux_roll == 1) {
			if (igla_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
			else if (igla_ap_roll == 2) SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);

			//stop conditions
			if (igla_ap_roll == 1 && avel.z >= 2) {
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
				igla_ap_roll = 0;
				igla_ap_aux_roll = 2;
				igla_idle = 1;
			}
			else if (igla_ap_roll == 2 && avel.z <= -2) {
				SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
				igla_ap_roll = 0;
				igla_ap_aux_roll = 2;
				igla_idle = 1;
			}
			else if ((tgt_err_roll > (last_tgt_err_roll / 2) && last_tgt_err_roll < 0) || (tgt_err_roll < (last_tgt_err_roll / 2) && last_tgt_err_roll > 0)) {
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
				igla_ap_aux_roll = 2;
				igla_ap_roll = 0;
				igla_idle = 1;
			}
		}

		if (igla_ap_aux_roll == 2) {

			if (igla_ap_roll == 0 && tgt_err_roll > -27 && tgt_err_roll < 27) {
				igla_ap_roll = 3;
			}

			else if (igla_ap_roll == 3) {
				ActivateNavmode(1);
				igla_ap_aux_roll = 3;
				igla_idle = 0;
			}
		}

		if (igla_ap_aux_roll == 3 && GetNavmodeState(1) == false) {
			rcs_status = 1;
			igla_ap_aux_roll = 0;
			igla_ap_roll = 0;
			igla_status = 3;
			igla_idle = 1;
		}
	}

	//flip retro phase, pitch down
	else if (igla_status == 6) {
		
		if (igla_ap_aux_pitch == 0) {
			if (avel.x <= -1.80) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
				igla_ap_aux_pitch = 1;
			}
			else SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
		}
		else if (igla_ap_aux_pitch == 1 && tgt_err_pitch >= 172.8) {
			ActivateNavmode(1);
			igla_ap_aux_pitch = 2;
		}
		else if (igla_ap_aux_pitch == 2 && !GetNavmodeState(1)) {
			igla_ap_aux_pitch = 0;
			igla_status = 7;
		}
	}
	//burn phase (main or retro depending on flag) - SKD preferred outside docking zone
	else if (igla_status == 7) {
		
		if (burn_seq == 5 || burn_seq == 7) {												//docking zone DPO retro brake burn
			if (tgt_relvel.z >= speed_target || GetPropellantMass(hpr_dpo) == 0) {			//negative z on approach facing target
				SetThrusterGroupLevel(THGROUP_RETRO, 0);
				igla_status = 3;
				if (burn_seq == 5) burn_seq = 6;
				else burn_seq = 8;
			}
			else {
				SetThrusterGroupLevel(THGROUP_RETRO, 1);
			}
		}
		else {
			if (tgt_relvel.z <= speed_target) {			//negative z on approach facing target, positive z on approach facing away
				SetThrusterGroupLevel(THGROUP_MAIN, 0);
				if (burn_seq == 3) {
					igla_status = 8;
					rcs_status = 3;
					burn_seq = 4;
				}
				else if (burn_seq == 11 && igla_LOS_burn == 2) {
					if (tgt_relvel.y > 0.5) igla_status = 10;
					else {
						igla_status = 8;
						igla_LOS_burn++;
					}
					rcs_status = 3;
					burn_seq = 2;
				}
				else if (burn_seq == 9 && igla_LOS_burn == 0) {
					if (tgt_relvel.y < 0.5) {
						igla_status = 9;
						rcs_status = 3;
					}
					else {
						igla_status = 4;
						igla_LOS_burn++;
					}
					burn_seq = 10;
				}
				else if (burn_seq == 9) {
					igla_status = 4;
					burn_seq = 10;
				}
				else if (burn_seq == 13) {
					burn_seq = 4;
					igla_status = 4;
				}
				else {
					igla_status = 4;
					rcs_status = 1;
					burn_seq = 2;
				}
			}
			else SetThrusterGroupLevel(THGROUP_MAIN, 1);
		}
	}
	//state 8, flip back phase, pitch up
	else if (igla_status == 8) {

		if (igla_ap_aux_pitch == 0) {
			if (avel.x >= 1.80) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
				igla_ap_aux_pitch = 1;
			}
			else SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
		}
		else if (igla_ap_aux_pitch == 1 && tgt_err_pitch <= 7.2) {
			ActivateNavmode(1);
			igla_ap_aux_pitch = 2;
		}
		else if (igla_ap_aux_pitch == 2 && !GetNavmodeState(1)) {
			igla_ap_aux_pitch = 0;
			if (tgt_range < 2000) igla_status = 3;
			else igla_status = 4;					
			rcs_status = 1;
		}
	}
	//state 9, pitch down to 90 degrees
	else if (igla_status == 9) {

		if (igla_ap_aux_pitch == 0) {
			if (avel.x <= -1.80) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
				igla_ap_aux_pitch = 1;
			}
			else SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
		}
		else if (igla_ap_aux_pitch == 1 && tgt_err_pitch >= 82.8) {
			ActivateNavmode(1);
			igla_ap_aux_pitch = 2;
		}
		else if (igla_ap_aux_pitch == 2 && !GetNavmodeState(1)) {
			igla_ap_aux_pitch = 0;
			igla_status = 11;
			rcs_status = 1;
		}
	}
	//state 10, pitch up to 90 degrees
	else if (igla_status == 10) {

		if (igla_ap_aux_pitch == 0) {
			if (avel.x >= 1.80) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
				igla_ap_aux_pitch = 1;
			}
			else SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
		}
		else if (igla_ap_aux_pitch == 1 && tgt_err_pitch <= 97.2) {
			ActivateNavmode(1);
			igla_ap_aux_pitch = 2;
		}
		else if (igla_ap_aux_pitch == 2 && !GetNavmodeState(1)) {
			igla_ap_aux_pitch = 0;
			igla_status = 11; 
			rcs_status = 1;
		}
	}

	//burn state for motion of line of sight
	else if (igla_status == 11) {

		if (tgt_relvel.z <= 0 || GetPropellantMass(hpr) == 0) {			
			SetThrusterGroupLevel(THGROUP_MAIN, 0);
			//if (burn_seq == 11 || burn_seq == 3) igla_status = 6;
			//else {
				//igla_status = 8;
				//rcs_status = 3;
			//}
			//if (igla_LOS_burn == 1) igla_status = 13;
			//else {
				if (burn_seq == 11 || burn_seq == 3) igla_status = 6;
				else igla_status = 8;
				rcs_status = 3;
			//}
			igla_LOS_burn++;
		}
		else {
			SetThrusterGroupLevel(THGROUP_MAIN, 1);
		}
	}

	//roll to align motion of line of sight to +y
	else if (igla_status == 12) {

		if (igla_ap_aux_roll == 0 && roll_drift_vector < 3) igla_ap_aux_roll = 3.5;	//used to be < 0.5

		//start roll orientation to align motion of line of sight to Soyuz +y
		//negative x component makes the shortest roll a roll to the right
		else if (igla_ap_aux_roll == 0 && roll_drift_vector > 0.5 && tgt_relvel.x < 0) {
			SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
			igla_ap_roll = 1;
			igla_ap_aux_roll = 1;
			last_tgt_err_roll = roll_drift_vector;
			igla_idle = 0;
		}
		else if (igla_ap_aux_roll == 0 && roll_drift_vector > 0.5 && tgt_relvel.x > 0) {
			SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
			igla_ap_roll = 2;
			igla_ap_aux_roll = 1;
			last_tgt_err_roll = roll_drift_vector;
			igla_idle = 0;
		}

		//runtime
		if (igla_ap_aux_roll == 1) {
			if (igla_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
			else if (igla_ap_roll == 2) SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);

			//stop conditions
			if (igla_ap_roll == 1 && avel.z >= 2.0) {
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
				igla_ap_roll = 0;
				igla_ap_aux_roll = 2;
				igla_idle = 1;
			}
			else if (igla_ap_roll == 2 && avel.z <= -2.0) {
				SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
				igla_ap_roll = 0;
				igla_ap_aux_roll = 2;
				igla_idle = 1;
			}
			else if (abs(intz) > (last_tgt_err_roll / 2)) {
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
				igla_ap_aux_roll = 2;
				igla_ap_roll = 0;
				igla_idle = 1;
			}
		}

		if (igla_ap_aux_roll == 2) {

			if (igla_ap_roll == 0 && (last_tgt_err_roll - abs(intz)) < 27 ) {
				igla_ap_roll = 3;
			}

			else if (igla_ap_roll == 3) {
				ActivateNavmode(1);
				igla_ap_aux_roll = 3;
				igla_idle = 0;
			}
		}

		if (igla_ap_aux_roll == 3 && GetNavmodeState(1) == false) {
			rcs_status = 1;
			igla_ap_aux_roll = 0;
			igla_ap_roll = 0;
			igla_status = 4;
			igla_idle = 1;
			intz = 0;
		}
	}
	else if (igla_status == 13) {

		if ((igla_ap_aux_lat == 0 && abs(tgt_relvel.x) <= 0.014) || GetPropellantMass(hpr_dpo) == 0) igla_ap_aux_lat = 2;


		if (igla_ap_aux_lat == 0 && tgt_relvel.x > 0) {
			SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 1);
			igla_ap_lat = 1;
			igla_ap_aux_lat = 1;
		}
		else if (igla_ap_aux_lat == 0 && tgt_relvel.x < 0) {
			SetThrusterGroupLevel(THGROUP_ATT_LEFT, 1);
			igla_ap_lat = 2;
			igla_ap_aux_lat = 1;
		}


		if (igla_ap_aux_lat == 1) {
			if (igla_ap_lat == 1) SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 1);
			else if (igla_ap_lat == 2) SetThrusterGroupLevel(THGROUP_ATT_LEFT, 1);

			//stop conditions
			if (igla_ap_lat == 1 && tgt_relvel.x <= 0) {
				SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 0);
				igla_ap_lat = 0;
				igla_ap_aux_lat = 2;
			}
			else if (igla_ap_lat == 2 && tgt_relvel.x >= 0) {
				SetThrusterGroupLevel(THGROUP_ATT_LEFT, 0);
				igla_ap_lat = 0;
				igla_ap_aux_lat = 2;
			}
		}

		if (igla_ap_aux_lat == 2) {
			igla_ap_aux_lat = 0;
			if (burn_seq == 11 || burn_seq == 3) igla_status = 6;
			else igla_status = 8;
			rcs_status = 3;
		}

	}

}

void Soyuz7k_T::Inertial(double MJD, double SimDT) {
	
	// integrate angular velocities over time step SimDT, simulates both free gyros and bdus in integration mode
	intx += avel.x * SimDT;
	inty += avel.y * SimDT;
	intz += avel.z * SimDT;

	// larger than 6 degree deviation in any axis cancels SOUD modes, with ELS warning
	if (((abs(intx) > 6 || abs(inty) > 6 || abs(intz) > 6) && modes_off == 0 && BDUS_integr) || ((abs(intx) > 8 || abs(inty) > 8 || abs(intz) > 8) && modes_off == 0 && !BDUS_integr)) {
		modes_off = 1;
		avaria = 1;
		m_pXRSound->PlayWav(AlarmSound, true, 1.0);
		signal_start = MJD;
	}

	// attitude hold process
	if (inertial_status == 1 && modes_off == 0) {
		if (rcs_status != 1 && rcs_status != 2) rcs_status = 1;

		if (ori_ap_aux == 0 && intz <= att_hold_limit / 2 && intz >= -att_hold_limit / 2) inertial_status = 2;
		else {
			if (ori_ap_aux == 0 && intz > att_hold_limit / 2) {
				SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
				ori_ap_aux = 1;
				ori_ap_roll = 1;
				last_roll = intz;
				if (oapiGetTimeAcceleration() != 1) oapiSetTimeAcceleration(1);
			}
			else if (ori_ap_aux == 0 && intz < -att_hold_limit / 2) {
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
				ori_ap_aux = 1;
				ori_ap_roll = 2;
				last_roll = intz;
				if (oapiGetTimeAcceleration() != 1) oapiSetTimeAcceleration(1);
			}
			if (ori_ap_aux == 1) {
				if (ori_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
				else if (ori_ap_roll == 2) SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);

				if (ori_ap_roll == 1 && avel.z <= -0.450) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					ori_ap_aux = 2;
					ori_ap_roll = 0;
				}
				else if (ori_ap_roll == 2 && avel.z >= 0.450) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					ori_ap_aux = 2;
					ori_ap_roll = 0;
				}
				else if ((intz > (last_roll / 2) && last_roll < 0) || (intz < (last_roll / 2) && last_roll > 0)) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					ori_ap_aux = 2;
					ori_ap_roll = 3;
				}
			}
			//kill rotation in roll 
			if (ori_ap_aux == 2) {
				if (ori_ap_roll == 0 && intz > -3.7 && intz < 3.7) {
					ori_ap_roll = 3;
				}
				if (ori_ap_roll == 3) {
					ActivateNavmode(1);
					ori_ap_aux = 3;
					ori_ap_roll = 0;
				}
				
			}
			if (ori_ap_aux == 3 && GetNavmodeState(1) == false) {
				inertial_status = 2;
				ori_ap_aux = 0;
			}
		}
	}
	else if (inertial_status == 2 && modes_off == 0) {
		if (rcs_status != 1 && rcs_status != 2) rcs_status = 1;

		if (ori_ap_aux_yaw == 0 && inty > -att_hold_limit && inty < att_hold_limit) ori_ap_aux_yaw = 4;

		//start yaw orientation
		if (ori_ap_aux_yaw == 0 && inty > att_hold_limit) {
			SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
			ori_ap_roll = 1;
			ori_ap_aux_yaw = 1;
			last_slip = inty;
			if (oapiGetTimeAcceleration() != 1) oapiSetTimeAcceleration(1);
		}
		else if (ori_ap_aux_yaw == 0 && inty < -att_hold_limit) {
			SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
			ori_ap_roll = 2;
			ori_ap_aux_yaw = 1;
			last_slip = inty;
			if (oapiGetTimeAcceleration() != 1) oapiSetTimeAcceleration(1);
		}

		//runtime
		if (ori_ap_aux_yaw == 1) {
			if (ori_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
			else if (ori_ap_roll == 2) SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);

			//stop conditions
			if (ori_ap_roll == 1 && avel.y <= -0.66) {
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				ori_ap_aux_yaw = 2;
			}
			else if (ori_ap_roll == 2 && avel.y >= 0.66) {
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				ori_ap_aux_yaw = 2;
			}
			else if ((inty > (last_slip / 2) && last_slip < 0) || (inty < (last_slip / 2) && last_slip > 0)) {
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				ori_ap_aux_yaw = 2;
			}
		}

		// kill rotation in yaw 
		if (ori_ap_aux_yaw == 2 && inty > -3.2 && inty < 3.2) {
			ActivateNavmode(1);
			ori_ap_aux_yaw = 3;
		}

		if (ori_ap_aux_yaw == 3 && GetNavmodeState(1) == false) {
			ori_ap_aux_yaw = 4;
		}

		if (ori_ap_aux_yaw == 4) {
			if (ori_ap_aux_pitch == 0 && intx > -att_hold_limit && intx < att_hold_limit) {
				ori_ap_aux_pitch = 3;
			}

			if (ori_ap_aux_pitch == 0 && intx > att_hold_limit) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
				ori_ap_roll = 1;
				ori_ap_aux_pitch = 1;
				last_pitch = intx;
			}
			else if (ori_ap_aux_pitch == 0 && intx < -att_hold_limit) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
				ori_ap_roll = 2;
				ori_ap_aux_pitch = 1;
				last_pitch = intx;
			}

			if (ori_ap_aux_pitch == 1) {
				if (ori_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
				else if (ori_ap_roll == 2) SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);

				//stop conditions
				if (ori_ap_roll == 1 && avel.x <= -0.66) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					ori_ap_roll = 0;
					ori_ap_aux_pitch = 2;
				}
				else if (ori_ap_roll == 2 && avel.x >= 0.66) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					ori_ap_roll = 0;
					ori_ap_aux_pitch = 2;
				}
				else if ((intx > (last_pitch / 2) && last_pitch < 0) || (intx < (last_pitch / 2) && last_pitch > 0)) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					ori_ap_aux_pitch = 2;
					ori_ap_roll = 3;
				}
			}

			if (ori_ap_aux_pitch == 2) {
				if (ori_ap_roll == 0 && intx > -3.5 && intx < 3.5) {
					ori_ap_roll = 3;
				}
				if (ori_ap_roll == 3) {
					ActivateNavmode(1);
					ori_ap_aux_pitch = 3;
				}
			}

			if (ori_ap_aux_pitch == 3 && GetNavmodeState(1) == false) {
				ori_ap_aux_pitch = 0;
				ori_ap_aux_yaw = 0;
				ori_ap_roll = 0;
				inertial_status = 1;
			}

		}

	}

}

void Soyuz7k_T::Uncage() {
	intx = 0;
	inty = 0;
	intz = 0;
	ori_ap_aux = 0;
}

void Soyuz7k_T::PVU(double SimDT, int scale) {

	// increment timer
	PVU_timer += scale * SimDT;

	// round down to closest integer minute elapsed
	int temp = (int) floor(PVU_timer / 60);
	if (PVU_minute != temp) {
		if (temp > 85) {
			PVU_timer = 0.0;
			PVU_minute = 0;
		}
		else PVU_minute = temp;
		IKP_update = 1;
	}
	

	// program sequencing
	if (program_on == 1) {
		// MANEVR
		if (PVU_cmd_seq == 0 && PVU_minute >= 43 && !PVU_cmd_off) {
			BDUS_status = BDUS_ON;
			PVU_cmd_seq++;
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 1 && PVU_minute >= 44 && !PVU_cmd_off) {
			//orb_ori_dir = 0;		// prograde setting
			ori_ap_status = 1;		// switch orbital orientation mode on
			rcs_status = 1;			// force DO mode
			PVU_cmd_seq++;			// orient
			PVU_cmd_seq++;			// inerts 1
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 3 && PVU_minute >= 54 && !PVU_cmd_off) {
			PVU_cmd_seq++;			// nadduv skdu
			PVU_cmd_seq++;			// inerts 2
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 5 && PVU_minute >= 64 && !PVU_cmd_off) {
			accel_z_status = 1;		// integrating accelerometer on and Dv reset
			accel_z = 0.0;
			PVU_cmd_seq++;			// integr accel
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 6 && PVU_minute >= 70 && !PVU_cmd_off) {
			if (ori_ap_status != 0 || sun_ap_status != 0) {
				ori_ap_status = 0;		//switch orbital orientation mode off, regardless of current state
				sun_ap_status = 0;		//switch solar orientation mode off, regardless of current state
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
				SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
				ActivateNavmode(1);
			}
			inertial_status = 1;
			Uncage();
			PVU_cmd_seq++;			// inerts orient
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 7 && PVU_minute >= 70 && !PVU_cmd_off) {
			SetThrusterGroupLevel(THGROUP_MAIN, 1);
			PVU_cmd_seq++;			// SKDU
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 8 && accel_z >= dv_accel && !PVU_cmd_off) {
			SetThrusterGroupLevel(THGROUP_MAIN, 0);
			PVU_cmd_seq++;			// SKDU OFF
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 9 && PVU_minute >= 85) {
			PVU_timer = 0;
			program_on = 0;
			PVU_minute = 0;
			PVU_cmd_seq = 0;
			inertial_status = 0;
			SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
			SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
			BDUS_status = BDUS_OFF;
			accel_z_status = 0;
			accel_z = 0;
			IKP_update = 1;
		}
	}
	else if (program_on == 2) {
		// SPUSK 2
		if (PVU_cmd_seq == 0 && PVU_minute >= 33 && !PVU_cmd_off) {
			BDUS_status = BDUS_ON;
			PVU_cmd_seq++;
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 1 && PVU_minute >= 34 && !PVU_cmd_off) {
			orb_ori_dir = 1;		// retrograde setting
			ori_ap_status = 1;		// switch orbital orientation mode on
			rcs_status = 1;			// force DO mode
			PVU_cmd_seq++;			// orient
			PVU_cmd_seq++;			// inerts 1
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 3 && PVU_minute >= 44 && !PVU_cmd_off) {
			PVU_cmd_seq++;			// inerts 2
			PVU_cmd_seq++;			// nadduv skdu
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 5 && PVU_minute >= 54 && !PVU_cmd_off) {
			accel_z_status = 1;		// integrating accelerometer on and Dv reset
			accel_z = 0.0;
			PVU_cmd_seq++;			// integr accel
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 6 && PVU_minute >= 59 && !PVU_cmd_off) {
			PVU_cmd_seq++;			// SUS
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 7 && PVU_minute >= 60 && !PVU_cmd_off) {
			if (ori_ap_status != 0 || sun_ap_status != 0) {
				ori_ap_status = 0;		//switch orbital orientation mode off, regardless of current state
				sun_ap_status = 0;		//switch solar orientation mode off, regardless of current state
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
				SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
				ActivateNavmode(1);
			}
			inertial_status = 1;
			Uncage();
			PVU_cmd_seq++;			// inerts orient
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 8 && PVU_minute >= 60 && !PVU_cmd_off) {
			SetThrusterGroupLevel(THGROUP_MAIN, 1);
			PVU_cmd_seq++;			// SKDU
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 9 && accel_z >= dv_accel && !PVU_cmd_off) {	
			SetThrusterGroupLevel(THGROUP_MAIN, 0);
			PVU_cmd_seq++;			// SKDU OFF
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 10 && PVU_minute >= 66 && !PVU_cmd_off) {
			PVU_cmd_seq++;			// DUS SUS
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 11 && PVU_minute >= 72 && !PVU_cmd_off) {
			// stop all SOUD modes and trigger separation
			if (sep_status == 0 && hAttached == NULL) {
				BDUS_status = BDUS_OFF;
				pulsed_status = 2;
				if (ori_ap_status != 0 || inertial_status != 0 || modes_off != 1) {
					ori_ap_status = 0;		// switch orbital orientation mode off, regardless of current state
					ori_ap_aux = 0;
					inertial_status = 0;	// switch inertial orientation mode off, regardless of current state
					accel_z_status = 0;		// switch off longitudinal accelerometer
					accel_z = 0;			// reset integrating accelerometer
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
				}
				sep_status = 1;
			}
			else if (sep_status == 2) {
				sep_status = 3;
				PVU_cmd_seq++;			// Razdel
			}
			IKP_update = 1;
		}
		if (PVU_cmd_seq == 12 && PVU_minute >= 85) {
			PVU_timer = 0;
			program_on = 0;
			PVU_minute = 0;
			PVU_cmd_seq = 0;
			inertial_status = 0;
			SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
			SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
			IKP_update = 1;
			BDUS_status = BDUS_OFF;
			accel_z_status = 0;
			accel_z = 0;
		}

	}

}

void Soyuz7k_T::AccelZ(double SimDT) {
	// function accumulates in accel_z the delta-v in m/s along z axis since moment of accelerometer activation

	VECTOR3 accel_th, accel_dr;
	double accel;
	// get force vectors acting on the ship
	GetThrustVector(accel_th);
	GetDragVector(accel_dr);
	// get total force along z axis in Newton, excluding weight
	accel = ((accel_th.z >= 0) ? accel_th.z : 0) + ((accel_dr.z >= 0) ? accel_dr.z : 0);

	// update total delta-v along z axis with velocity change over one timestep from acceleration in m/s^2
	// acceleration = force [N] / total mass [kg]
	accel_z += (accel / GetMass()) * SimDT;

	// while the accelerometer is on, displays incremental velocity on BTSI
	if (!manual_settings) {
		btsi_5_counter = accel_z;
		btsi_5_flag = 2;
	}

}

void Soyuz7k_T::AccelSUS(double SimDT) {
	// function accumulates in accel_z the delta-v in m/s along z axis since moment of accelerometer activation

	VECTOR3 accel_th, accel_dr, accel;
	// get force vectors acting on the ship
	GetThrustVector(accel_th);
	GetDragVector(accel_dr);
	// get total force along in Newton, excluding weight
	accel = accel_th + accel_dr;

	// update total delta-v with velocity change over one timestep from acceleration in m/s^2
	// acceleration = force [N] / total mass [kg]
	double temp = sqrt(pow(accel.x, 2) + pow(accel.y, 2) + pow(accel.z, 2));
	accel_SUS += (temp / GetMass()) * SimDT;
}

void Soyuz7k_T::SUS(double SimDT) {

	//Maintain pitch rates below 2 deg/s on pre-entry phase
	if (SUS_status == 1) {

		// yaw and roll limits to trigger ballistic descent
		if (GetSlipAngle() > 54 * DEG || GetSlipAngle() < -54 * DEG) ballistic = 1;
		else if (GetBank() > 173 * DEG || GetBank() < -173 * DEG) ballistic = 1;

		// signal for pitch thruster pulse command - deadband of 2 deg/s sets K1x = 0.5
		if (x_duration == 0) {
			Ux = K1x * abs(avel.x);
			if (Ux >= 1.0) {
				if (Ux >= 2.0) x_duration = 1.4;
				else x_duration = 1.340 * Ux - 1.280;
				oapiSetTimeAcceleration(1);
				SUS_timestep_accum_x = 0;
				ori_ap_aux = 0;
			}
		}
		else if (x_duration == 2) {
			SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
			SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
			x_duration = 0;
		}
		// corrective pulse
		else {
			if (ori_ap_aux == 0 && avel.x < 0) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
				SUS_timestep_accum_x += SimDT;
				ori_ap_pitch = 1;
				ori_ap_aux = 1;
			}
			else if (ori_ap_aux == 0 && avel.x > 0) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
				SUS_timestep_accum_x += SimDT;
				ori_ap_pitch = 2;
				ori_ap_aux = 1;
			}
			else if (ori_ap_aux == 1) {
				double thrustLevel;
				double R = x_duration - SUS_timestep_accum_x;
				if (R < SimDT) {
					thrustLevel = R / SimDT;
					x_duration = 2;
					ori_ap_aux = 0;
				}
				else {
					thrustLevel = 1;
					if (R == SimDT) {
						x_duration = 2;
						ori_ap_aux = 0;
					}
				}
				if (ori_ap_pitch == 1) SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, thrustLevel);
				else SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, thrustLevel);
				SUS_timestep_accum_x += SimDT;
			}
		}
		
	}

	//Entry interface reached when 25.6 m/s deccel is measured
	if (SUS_status == 1 && accel_SUS >= 25.6) {
		SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
		SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
		SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
		SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
		ori_ap_pitch = 0;
		ori_ap_aux = 0;
		SUS_status = 2;
		x_duration = 0.0;
		y_duration = 0.0;
		z_duration = 0.0;
		//add KSU "Atmosphere" logic
		ballistic = 1; //test - force ballistic mode
		ActivateNavmode(1);

		// airfoil re-definitions
		DelAirfoil(airfoil_v);
		DelAirfoil(airfoil_h);
		airfoil_v = CreateAirfoil2(LIFT_VERTICAL, _V(0, -mesh_shift_y, 0), vliftSUS, 2.17, 3.82, 1.05);
		airfoil_h = CreateAirfoil2(LIFT_HORIZONTAL, _V(0, 0, 0), hliftNull, 2.17, 3.82, 1.05);
	}
	
	if (SUS_status == 2 && accel_SUS >= 7200) {
		SUS_status = 3;
	}

	if (GetNavmodeState(1) && SUS_status == 2 && (abs(avel.x) + abs(avel.y) + abs(avel.z)) < 0.005) {
		DeactivateNavmode(1);
	}

	//ballistic descent mode handling
	if (ballistic && SUS_status == 2 && GetNavmodeState(1) == false) {
		
		
		//ballistic roll start/update - thrusters when below 12.5 deg/s
		if (ori_ap_aux == 0 && avel.z <= 12.5) {
			SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
			ori_ap_aux = 1;
		}
		else if (ori_ap_aux == 1) {
			SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);

			//stop condition at 13 deg/s, to be maintained
			if (avel.z >= 13) {
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
				ori_ap_aux = 0;
			}
		}
	}
	else if (ballistic && SUS_status == 3) {
		//ballistic roll start/update - thrusters when below 12.5 deg/s
		if (ori_ap_aux == 0 && avel.z <= 12.5) {
			SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
			ori_ap_aux = 1;
		}
		else if (ori_ap_aux == 1) {
			SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);

			//stop condition at 13 deg/s, to be maintained
			if (avel.z >= 13) {
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
				ori_ap_aux = 0;
			}
		}
	}
	

}

void Soyuz7k_T::ClockHandling(double MJD) {

	// clock handling - animations and time keeping
	if (clock_enable == 1) {
		last_MJD_clock = MJD;
		clock_enable = 2;
	}
	else if (clock_enable == 2) {
		if (delta_MJD <= 0.0000002887 && (MJD - last_MJD_clock) >= 0.000005775) { //0.000005785
			clk_sec_proc += 0.00833333333333333334;
			clk_min_proc += 0.00013888888888888888;
			clk_hour_proc += 0.000004629629629629628;
			if (clk_sec_proc >= 1) clk_sec_proc -= 1.00;
			if (clk_min_proc >= 1) clk_min_proc -= 1.00;
			if (clk_hour_proc >= 1) clk_hour_proc -= 1.00;
			last_MJD_clock = MJD;
			SetAnimation(anim_clk_sec, clk_sec_proc);
			SetAnimation(anim_clk_min, clk_min_proc);
			SetAnimation(anim_clk_hour, clk_hour_proc);
			m_pXRSound->PlayWav(ClockTick, false, 0.5);
		}
		else if (delta_MJD > 0.0000002887 && delta_MJD <= 0.000002887 && (MJD - last_MJD_clock) >= 0.00005775) {
			clk_sec_proc += 0.0833333333333333334;
			clk_min_proc += 0.0013888888888888888;
			clk_hour_proc += 0.00004629629629629628;
			if (clk_sec_proc >= 1) clk_sec_proc -= 1.00;
			if (clk_min_proc >= 1) clk_min_proc -= 1.00;
			if (clk_hour_proc >= 1) clk_hour_proc -= 1.00;
			last_MJD_clock = MJD;
			SetAnimation(anim_clk_sec, clk_sec_proc);
			SetAnimation(anim_clk_min, clk_min_proc);
			SetAnimation(anim_clk_hour, clk_hour_proc);
		}
		else if (delta_MJD > 0.000002887 && delta_MJD <= 0.00002887 && (MJD - last_MJD_clock) >= 0.0005775) {
			clk_sec_proc += 0.833333333333333334;
			clk_min_proc += 0.013888888888888888;
			clk_hour_proc += 0.0004629629629629628;
			if (clk_sec_proc >= 1) clk_sec_proc -= 1.00;
			if (clk_min_proc >= 1) clk_min_proc -= 1.00;
			if (clk_hour_proc >= 1) clk_hour_proc -= 1.00;
			last_MJD_clock = MJD;
			SetAnimation(anim_clk_sec, clk_sec_proc);
			SetAnimation(anim_clk_min, clk_min_proc);
			SetAnimation(anim_clk_hour, clk_hour_proc);
		}
	}

	// cronometer handling
	if (crono_enable == 0) {
		SetAnimation(anim_crono_sec, 0.0);
		SetAnimation(anim_crono_min, 0.0);
		SetAnimation(anim_crono_hour, 0.0);
	}
	else if (crono_enable == 1) {
		last_MJD_crono = MJD;
		crono_enable = 2;
	}
	else if (crono_enable == 2) {
		if (delta_MJD <= 0.0000002887 && (MJD - last_MJD_crono) >= 0.000005775) { //0.000005785
			crono_sec_proc += 0.00833333333333333334;
			crono_min_proc += 0.00013888888888888888;
			crono_hour_proc += 0.000004629629629629628;
			if (crono_sec_proc >= 1) crono_sec_proc -= 1.00;
			if (crono_min_proc >= 1) crono_min_proc -= 1.00;
			if (crono_hour_proc >= 1) crono_hour_proc -= 1.00;
			last_MJD_crono = MJD;
			SetAnimation(anim_crono_sec, crono_sec_proc);
			SetAnimation(anim_crono_min, crono_min_proc);
			SetAnimation(anim_crono_hour, crono_hour_proc);
		}
		else if (delta_MJD > 0.0000002887 && delta_MJD <= 0.000002887 && (MJD - last_MJD_crono) >= 0.00005775) {
			crono_sec_proc += 0.0833333333333333334;
			crono_min_proc += 0.0013888888888888888;
			crono_hour_proc += 0.00004629629629629628;
			if (crono_sec_proc >= 1) crono_sec_proc -= 1.00;
			if (crono_min_proc >= 1) crono_min_proc -= 1.00;
			if (crono_hour_proc >= 1) crono_hour_proc -= 1.00;
			last_MJD_crono = MJD;
			SetAnimation(anim_crono_sec, crono_sec_proc);
			SetAnimation(anim_crono_min, crono_min_proc);
			SetAnimation(anim_crono_hour, crono_hour_proc);
		}
		else if (delta_MJD > 0.000002887 && delta_MJD <= 0.00002887 && (MJD - last_MJD_crono) >= 0.0005775) {
			crono_sec_proc += 0.833333333333333334;
			crono_min_proc += 0.013888888888888888;
			crono_hour_proc += 0.0004629629629629628;
			if (crono_sec_proc >= 1) crono_sec_proc -= 1.00;
			if (crono_min_proc >= 1) crono_min_proc -= 1.00;
			if (crono_hour_proc >= 1) crono_hour_proc -= 1.00;
			last_MJD_crono = MJD;
			SetAnimation(anim_crono_sec, crono_sec_proc);
			SetAnimation(anim_crono_min, crono_min_proc);
			SetAnimation(anim_crono_hour, crono_hour_proc);
		}
	}

	// Mission Elapsed Time display calcs

	MET_MJD = MJD - start_MJD;

	if (MET_MJD > 0) {
		int temp = (int)floor(MET_MJD / 0.000694444403);	//divide by minute in Julian to get total minutes elapsed
		if (MET_minute != temp) MET_update = 3;
		MET_minute = temp;
		MET_day = MET_minute / 1440;
		MET_hour = (MET_minute / 60);
		if (MET_hour > 23) MET_hour -= 24 * MET_day;
		MET_minute %= 60;
	}
	else {
		MET_day = MET_hour = MET_minute = 0;
	}

}

void Soyuz7k_T::Electrical(double SimDT) {


	// Update power consumption depending on active systems
	if (!combined_power) power_modifier = base_power + ikp_power + ssvp_power + spotlight_power + clock_power;
	else power_modifier = 0; 
	
	
	// Recharge from space station or solar panels - TEMP power
	if (recharge) {
		if (BB_power < MAX_BB_POWER) BB_power += 0.884 * SimDT;
		else BB_power = MAX_BB_POWER;
		if (RB_power < MAX_RB_POWER) RB_power += 0.884 * SimDT;
		else RB_power = MAX_RB_POWER;

		charge_current = 884.0 / 34.0;

		// charging of both batteries has finished
		if (BB_power == MAX_BB_POWER && RB_power == MAX_RB_POWER) {
			recharge = 0;
		} 
	}
	else charge_current = 0;

	// Calculate active battery's voltage and current, and decrement battery energy by current consumption per timestep until empty
	// Power modifier is updated when systems are powered ON or OFF
	if (BB_status) {
		if (BB_power > 0.1 * MAX_BB_POWER)
			BB_voltage = 2.333 * (BB_power / MAX_BB_POWER) + 31.77;
		else
			BB_voltage = 90.00 * (BB_power / MAX_BB_POWER) + 23.00;
		if (BB_power > 0) {
			BB_current = (power_modifier * 1000) / BB_voltage;
			BB_power -= power_modifier * SimDT;
		}
		else {
			BB_power = 0;
			BB_current = 0;
		}
	}
	else if (SA_BB_status) {
		if (SA_BB_power > 0.1 * MAX_SA_POWER)
			SA_BB_voltage = 2.333 * (SA_BB_power / MAX_SA_POWER) + 31.77;
		else
			SA_BB_voltage = 90.00 * (SA_BB_power / MAX_SA_POWER) + 23.00;
		if (SA_BB_power > 0) {
			SA_BB_current = (power_modifier * 1000) / SA_BB_voltage;
			SA_BB_power -= power_modifier * SimDT;
		}
		else {
			SA_BB_power = 0;
			SA_BB_current = 0;
		}
	}
	else if (RB_status) {
		if (RB_power > 0.1 * MAX_RB_POWER)
			RB_voltage = 2.333 * (RB_power / MAX_RB_POWER) + 31.77;
		else
			RB_voltage = 90.00 * (RB_power / MAX_RB_POWER) + 23.00;
		if (RB_power > 0) {
			RB_current = (power_modifier * 1000) / RB_voltage;
			RB_power -= power_modifier * SimDT;
		}
		else {
			RB_power = 0;
			RB_current = 0;
		}
	}
	else {
		BB_current = 0;
		BB_voltage = 0;
		RB_current = 0;
		RB_voltage = 0;
		SA_BB_current = 0;
		SA_BB_voltage = 0;
	}

	// Check if power is currently available and act accordingly
	if (((BB_status && BB_power == 0 && sep_status <= 2) || (RB_status && RB_power == 0 && sep_status <= 2) || (SA_BB_status && SA_BB_power == 0 && sep_status > 2) || (!BB_status && !RB_status && sep_status <= 2)) && !combined_power) {
		no_power = 1;				
		spotlightVC->Activate(false);
		beac1.active = false;
		spotlight1->Activate(false);
		spotlight2->Activate(false);
		beac2.active = false;
		beac3.active = false;
		beac4.active = false;
		beac6.active = false;
		IglaOff();
		BDUS_status = BDUS_OFF;
		pulsed_status = 2;
		if (ori_ap_status != 0 || inertial_status != 0 || sun_ap_status != 0) {
			ori_ap_status = 0;		//switch orbital orientation mode off, regardless of current state
			ori_ap_aux = 0;
			sun_ap_status = 0;
			inertial_status = 0;	//switch inertial orientation mode off, regardless of current state
			accel_z_status = 0;		//switch off longitudinal accelerometer
			accel_z = 0;			//reset integrating accelerometer
			SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
			SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
			SetThrusterGroupLevel(THGROUP_MAIN, 0);
			SetThrusterGroupLevel(THGROUP_RETRO, 0); 
		}
	}
	else if (no_power)
		no_power = 0;
}

// ==============================================================
// API callback interface
// ==============================================================

void Soyuz7k_T::clbkPostStep(double SimT, double SimDT, double MJD) {

	// if ((BB_status && BB_power > 0 && sep_status <= 2) || (RB_status && RB_power > 0 && sep_status <= 2) || (SA_BB_status && SA_BB_power > 0 && sep_status > 2)) {
		if (sep_status >= 4) {
			GetAngularVelSUS();
		}
		else {
			GetAngularVel();
		}
	//}  // for the "if battery" condition
}

void Soyuz7k_T::clbkPreStep(double SimT, double SimDT, double MJD) {

	// calculate battery status
	Electrical(SimDT);

	if (GetDynPressure() >= 13000 && skin_status == 0 && sep_status == 4) skin_status = 1;	// trigger charred skin
	// replace SA with re-entry charred version
	if (skin_status == 1) {
		SetMeshVisibilityMode(mesh[17], MESHVIS_EXTERNAL);
		SetMeshVisibilityMode(mesh[1], MESHVIS_NEVER);
		skin_status = 2;
	}

	// camera (vzor view) processing
	if (sep_status < 3) {
		if (oapiCockpitMode() != COCKPIT_VIRTUAL && oapiCameraInternal() && oapiGetFocusObject() == GetHandle()) {
			if (oapiCameraAperture() != 7.5 * RAD) {
				oapiCameraSetAperture(7.5 * RAD);
				SetCameraOffset(_V(0.277, -1.48, 0.5));
				SetCameraRotationRange(0, 0, 0, 0);
			}

			if (vzor_status == 2 || vzor_status == 1) {
				SetCameraDefaultDirection(_V(0, 0, 1));
				oapiCameraSetCockpitDir(0, 0, false);
			}
			else {
				SetCameraDefaultDirection(_V(0, -0.994522, 0.104528));
				oapiCameraSetCockpitDir(0, 0, false);
			}
		}
		
		if (vzor_status == 1) {				//switch to vzor nadir

			double da = SimDT * VZOR_SPEED;
			vzor_timer = vzor_timer + da;

			if (vzor_c_on == 1) {
				vzor_c_on = 2;	// intermediate state to render a blank before setting to zero
				gcCustomCameraOnOff(hCam_vzor_fwd, false);
				gcCustomCameraOnOff(hCam_vzor_nad, false);
			}

			if (vzor_timer >= 1) {
				vzor_timer = 1;
				vzor_status = 3;
				if (gcEnabled()) {
					vzor_c_on = 1;
					gcCustomCameraOnOff(hCam_vzor_nad, true);
					gcCustomCameraOnOff(hCam_vzor_fwd, false);
					oapiVCTriggerRedrawArea(-1, VZOR_CENTRAL);
				}
			}
		}
		else if (vzor_status == 0) {		//switch to vzor fwd

			double da = SimDT * VZOR_SPEED;
			vzor_timer = vzor_timer - da;

			if (vzor_c_on == 1) {
				vzor_c_on = 2;	// intermediate state to render a blank before setting to zero
				gcCustomCameraOnOff(hCam_vzor_fwd, false);
				gcCustomCameraOnOff(hCam_vzor_nad, false);
			}

			if (vzor_timer <= 0) {
				vzor_timer = 0;
				vzor_status = 2;
				if (gcEnabled()) {
					vzor_c_on = 1;
					gcCustomCameraOnOff(hCam_vzor_nad, false);
					gcCustomCameraOnOff(hCam_vzor_fwd, true);
					oapiVCTriggerRedrawArea(-1, VZOR_CENTRAL);
				}
			}
		}
	}


	// if battery available condition, else do nothing
	if (!no_power) {

		// processing for Descent Module specific functions during reentry phase
		if (sep_status >= 4) {
		
			if (sep_status == 4 && SUS_status > 0) {
				if (accel_SUS < 7200) AccelSUS(SimDT);
				SUS(SimDT);
			}

			altitude = GetAltitude(ALTMODE_GROUND);
			if (!ballistic && altitude <= 10500 && sep_status == 4) {
				ballistic = 1;								// engage ballistic mode at 10.5 km
			}
			if (altitude <= 5500 && sep_status == 4 && AKSP_status) {
				// jettison heatshield
				sep_status = 5;								
				// turn off ballistic mode
				ballistic = 0;								
				ori_ap_aux = 0;
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
			
			}
			
			if (altitude <= 2.7 && AKSP_status) {
				SetThrusterGroupLevel(THGROUP_MAIN, 1);				// fire soft landing engines
				SetAnimation(main_chute_deploy, 0.0);				// remove chute
				main_chute_status = 4;
			}

			// drain propellant after heat shield separation - null net torque
			if (sep_status == 6 && GetPropellantMass(hpr_sa)) {
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
				SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
				SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
			}

			// just in case it stops working, used to be else if and after soft landing if
			if (skin_status == 2) {
				//drogue chute animation
				if (drogue_chute_status == 1)
				{
					double da = SimDT * MAIN_CHUTE_SPEED;
					drogue_chute_proc = drogue_chute_proc + da;
					if (drogue_chute_proc > 1)
					{
						drogue_chute_proc = 1;
						drogue_chute_status = 2;
					}
					SetAnimation(drogue_chute_deploy, drogue_chute_proc);
					SetCrossSections(_V((3.9 + 3 * drogue_chute_proc), (3.83 + 3 * drogue_chute_proc), (3.82 + 21 * drogue_chute_proc)));
					EditAirfoil(airfoil_v, 01111, _V(0, 0, main_chute_proc), vliftDrogue, (2 + 5 * drogue_chute_proc), (3.82 + 19 * drogue_chute_proc), 1);
					EditAirfoil(airfoil_h, 01111, _V(0, 0, main_chute_proc), hliftDrogue, (2 + 5 * drogue_chute_proc), (3.82 + 19 * drogue_chute_proc), 1);
				}
				else if (drogue_chute_status == 2)
				{
					SetAnimation(drogue_chute_deploy, 1.0);
				}
				else if (drogue_chute_status == 3)
				{
					double da = SimDT * MAIN_CHUTE_SPEED;
					drogue_chute_proc = drogue_chute_proc - da;
					if (drogue_chute_proc < 0)
					{
						drogue_chute_proc = 0;
						drogue_chute_status = 0;
					}
					SetAnimation(drogue_chute_deploy, drogue_chute_proc);
					SetCrossSections(_V((3.9 + 3 * drogue_chute_proc), (3.83 + 3 * drogue_chute_proc), (3.82 + 21 * drogue_chute_proc)));
					EditAirfoil(airfoil_v, 01111, _V(0, 0, main_chute_proc), vliftDrogue, (2 + 5 * drogue_chute_proc), (3.82 + 19 * drogue_chute_proc), 1);
					EditAirfoil(airfoil_h, 01111, _V(0, 0, main_chute_proc), hliftDrogue, (2 + 5 * drogue_chute_proc), (3.82 + 19 * drogue_chute_proc), 1);
				}
				else if (drogue_chute_status == 0) {
					if (GetAltitude(ALTMODE_GROUND) <= 10500 && AKSP_status) {	// trigger drogue chute deploy at 10.5 km AGL
						drogue_chute_status = 1;
						DelMesh(mesh[14]);									// chute cover jettison
						SetEmptyMass(GetEmptyMass() - 50);
						SetRotDrag(_V(1, 1, 0.001));
					}
					SetAnimation(drogue_chute_deploy, 0.0);
				}
			
			
				//main chute animation
				if (main_chute_status == 1)
				{
					double da = SimDT * MAIN_CHUTE_SPEED;
					main_chute_proc = main_chute_proc + da;
					if (main_chute_proc > 1)
					{
						main_chute_proc = 1;
						main_chute_status = 2;
					}
					SetAnimation(main_chute_deploy, main_chute_proc);
					SetCrossSections(_V((3.9 + 166 * main_chute_proc), (3.83 + 166 * main_chute_proc), (3.82 + 1000 * main_chute_proc)));
					EditAirfoil(airfoil_v, 01111, _V(0, 0, 0), vliftNull, 30 * main_chute_proc, 1000 * main_chute_proc, 1);
					EditAirfoil(airfoil_h, 01111, _V(0, 0, 0), hliftNull, 30 * main_chute_proc, 1000 * main_chute_proc, 1);
				}
				else if (main_chute_status == 2)
				{
					SetAnimation(main_chute_deploy, 1.0);
				}
				else if (main_chute_status == 3)
				{
					double da = SimDT * MAIN_CHUTE_SPEED;
					main_chute_proc = main_chute_proc - da;
					if (main_chute_proc < 0)
					{
						main_chute_proc = 0;
						main_chute_status = 0;
					}
					SetAnimation(main_chute_deploy, main_chute_proc);
					SetCrossSections(_V((3.9 + 166 * main_chute_proc), (3.83 + 166 * main_chute_proc), (3.82 + 1000 * main_chute_proc)));
					EditAirfoil(airfoil_v, 01111, _V(0, 0, 0), vliftNull, 30 * main_chute_proc, 1000 * main_chute_proc, 1);
					EditAirfoil(airfoil_h, 01111, _V(0, 0, 0), hliftNull, 30 * main_chute_proc, 1000 * main_chute_proc, 1);
				}
				else if (main_chute_status == 0){
					if (GetAltitude(ALTMODE_GROUND) <= 8500 && AKSP_status) {	// trigger main chute deploy at 8 km AGL
						main_chute_status = 1;
						SetRotDrag(_V(3, 3, 0.001));
						SetAnimation(drogue_chute_deploy, 0.0);
						drogue_chute_status = 4;
						drogue_chute_proc = 0.0;
					}
					SetAnimation(main_chute_deploy, 0.0);
				}
			}
			if ((main_chute_status != 0 || drogue_chute_status != 0) && oapiGetTimeAcceleration() > 10) oapiSetTimeAcceleration(10);
		}
	
		if (!(spotlightVC->IsActive()) && VClight_status) spotlightVC->Activate(true);


		

		// BO sep / PAO sep / heatshield sep
		if (sep_status == 1 || sep_status == 3 || sep_status == 5) JettisonModule();
	
		GetElements(NULL, el, &orbparam, 0, FRAME_EQU);

		// Integrating Accelerometer
		if (accel_z_status) AccelZ(SimDT);

		// Program Timing Device
		if (IKP_check || !program_on) PVU(SimDT, 60);
		else PVU(SimDT, 1);

		// Timer for display refresh rate - 2 Hz
		refresh_timer += SimDT;
		if (refresh_timer >= 0.5) {
			refresh_timer = 0;
			KSU_update = 1;
		}

		// Get current coordinates for VC animation
		GetEquPos(longitude, latitude, radius);

		if (longitude < 0) long_proc = (2 * PI + longitude) / (2 * PI);
		else long_proc = longitude / (2 * PI);

		if (latitude < 0) lat_proc = (PI + latitude) / (PI);
		else lat_proc = latitude / (PI);

		// Lat & Long animations
		SetAnimation(anim_long, long_proc);
		SetAnimation(anim_lat, lat_proc);

		// switch active engine
		if (engine_status == 3 && sep_status < 3) {
			// main engine
			th_group[0] = th_main[0];
			CreateThrusterGroup(th_group, 1, THGROUP_MAIN);
			engine_status = 0;
		}
		else if (engine_status == 1 && sep_status < 3) {
			// backup engine
			th_group[0] = th_main[1];
			CreateThrusterGroup(th_group, 1, THGROUP_MAIN);
			engine_status = 2;
		}

	
		// timed antennae deploy after launch
		hAttached = GetAttachmentStatus(attach1);
		if (hAttached == NULL && timer == 0) {
			timer = MJD;
			thermal_status = 1;
		}

		if (antennae_status == 0 && timer != 0 && (MJD - timer) >= 0.0006944) antennae_status = 1;

		// time handling
		delta_MJD = MJD - last_MJD;

		if (last_MJD_clock == 0) last_MJD_clock = MJD;
		if (last_MJD_crono == 0) last_MJD_crono = MJD;

		ClockHandling(MJD);

		last_MJD = MJD;

		// 16 cylinder faces -> each column advance is a 360/16=0.0625 anim proc increment
		// KSUl cylinder animation
		if (KSUl_cyl_rotating) {
			KSUl_cyl_proc += SimDT * CYL_SPEED;
			KSUl_rot_acc += SimDT * CYL_SPEED;
			if (KSUl_cyl_proc >= 1.0) KSUl_cyl_proc -= 1.0;		// overflow case
			if (KSUl_rot_acc >= KSUl_rot_tgt) {					// if cylinder has rotated required angular distance
				KSUl_cyl_proc = (double)KSUl_curr_col * 0.0625;	// fix cylinder rotation at exact value
				KSUl_cyl_rotating = 0;							// stop rotation
			}
			SetAnimation(anim_KSUl_cyl, KSUl_cyl_proc);
		}
		// KSUp cylinder animation
		if (KSUp_cyl_rotating) {
			KSUp_cyl_proc += SimDT * CYL_SPEED;
			KSUp_rot_acc += SimDT * CYL_SPEED;
			if (KSUp_cyl_proc >= 1.0) KSUp_cyl_proc -= 1.0;		// overflow case
			if (KSUp_rot_acc >= KSUp_rot_tgt) {					// if cylinder has rotated required angular distance
				KSUp_cyl_proc = (double)KSUp_curr_col * 0.0625;	// fix cylinder rotation at exact value
				KSUp_cyl_rotating = 0;							// stop rotation
			}
			SetAnimation(anim_KSUp_cyl, KSUp_cyl_proc);
		}

		// BTSI counter 5 knob processing
		if (btsi_5_flag == -1 || btsi_5_flag == 1) {
			double da = SimDT * 16.8;
			dv_accel_proc = dv_accel_proc + da * btsi_5_flag;
			if (dv_accel_proc >= 1) {
				dv_accel_proc -= 1;
				btsi_5_counter += 0.044;
				if (btsi_5_counter > 1000) btsi_5_counter -= 1000.0;
			}
			else if (dv_accel_proc <= 0) {
				dv_accel_proc += 1;
				btsi_5_counter -= 0.044;
				if (btsi_5_counter < 0) btsi_5_counter += 1000.0;
			}
			btsi_5_flag = 2;
		}
		//reading frequency of 16.66 Hz, 0.044 m/s every 60 ms
		else if (btsi_5_read) {
			double da = SimDT * 17.0;
			dv_accel_proc = dv_accel_proc - da;
			if (dv_accel_proc <= 0) {
				dv_accel_proc += 1;
				btsi_5_counter -= 0.044;
				if (btsi_5_counter < 0.0) {
					btsi_5_counter = 0.0;
					btsi_5_read = 0;
					dv_accel = btsi_5_buffer;
				}
			}
			btsi_5_flag = 2;
		}

		 
		// ugol posadki knob processing
		// simulate continuous button turning
		if (ink_ugol_flag == -1 || ink_ugol_flag == 1) {
			double da = SimDT * 0.035;
			ugol_pos_proc = ugol_pos_proc + da * ink_ugol_flag;
			if (ugol_pos_proc >= 1) ugol_pos_proc -= 1;
			else if (ugol_pos_proc < 0) ugol_pos_proc += 1;

			//get the integer value between 0 and 359 from the decimal above
			ugol_pos = round(ugol_pos_proc * 360);
			if (ugol_pos == 360) ugol_pos = 0;
		
			if (ink_select_status == 2) {
				int diff = ugol_pos - last_ugol_pos;
				if (diff > 0) {
					//advance orbital track by ugol_pos degrees
					globe_o_proc -= diff / 360.0;
					//get decimal minutes equivalent to how long ugol_pos degrees take to travel in orbital track
					double x = ink_period * diff / 360.0;
					//percentage of rotation around the poles corresponding to previous calculated time x
					globe_e_proc += x / 1415.87;
					//save ugol on activation to track real-time changes in MP mode
					last_ugol_pos = ugol_pos;
				}
				else if (diff < 0) {
					//move back orbital track by ugol_pos degrees
					globe_o_proc += abs(diff) / 360.0;
					//get decimal minutes equivalent to how long ugol_pos degrees take to travel in orbital track
					double x = ink_period * abs(diff) / 360.0;
					//percentage of rotation around the poles corresponding to previous calculated time x
					globe_e_proc -= x / 1415.87;
					last_ugol_pos = ugol_pos;
				}
			}
			ink_ugol_flag = 2;
		}

		// orbit counter knob processing
		if (ink_vit_flag == -1 || ink_vit_flag == 1) {
			double da = SimDT * 2;
			orbit_count += da * ink_vit_flag;
			if (orbit_count >= 1000) orbit_count -= 1000;
			else if (orbit_count < 0) orbit_count += 1000;
			ink_vit_flag = 2;
		}

		// globe o knob being adjusted
		if (ink_o_flag != 0) {
			//globe orbital track animation
			double da = SimDT * 0.04;
			globe_o_proc = globe_o_proc + da * ink_o_flag;
			if (globe_o_proc > 1) globe_o_proc -= 1;
			else if (globe_o_proc < 0) globe_o_proc += 1;
			SetAnimation(anim_globe_o, globe_o_proc);
		
			//actual knob animation
			da = SimDT * 0.08;
			o_knb_proc = o_knb_proc + da * ink_o_flag;
			if (o_knb_proc > 1) o_knb_proc -= 1;
			else if (o_knb_proc < 0) o_knb_proc += 1;
			SetAnimation(anim_o_knb, o_knb_proc);

			ink_o_flag = 0;

			// correct globe e animation axis with rotation matrix around x for the initial vector for the poles in the mesh file
			double angGlobeO = 2 * PI * globe_o_proc;
			globe_e_axis.x = GLOBE_E_AXIS_INIT.x;
			globe_e_axis.y = GLOBE_E_AXIS_INIT.y * cos(angGlobeO) - GLOBE_E_AXIS_INIT.z * sin(angGlobeO);
			globe_e_axis.z = GLOBE_E_AXIS_INIT.y * sin(angGlobeO) + GLOBE_E_AXIS_INIT.z * cos(angGlobeO);
			globe_e->axis = globe_e_axis;
		}
		// globe e knob being adjusted
		else if (ink_e_flag != 0) {
			//globe daily rotation animation
			double da = SimDT * 0.04;
			globe_e_proc = globe_e_proc + da * ink_e_flag;
			if (globe_e_proc > 1) globe_e_proc -= 1;
			else if (globe_e_proc < 0) globe_e_proc += 1;
			SetAnimation(anim_globe_e, globe_e_proc);

			//actual knob animation
			da = SimDT * 0.08;
			e_knb_proc = e_knb_proc + da * ink_e_flag;
			if (e_knb_proc > 1) e_knb_proc -= 1;
			else if (e_knb_proc < 0) e_knb_proc += 1;
			SetAnimation(anim_e_knb, e_knb_proc);

			ink_e_flag = 0;
		}
		// normal globe motion
		else if (ink_select_status != 0) {
		
			//orbital track speed is dynamic, from value in orbital period counter
			double da_o = SimDT * globe_o_speed;
			//daily rotation speed fixed for 23h 35m 52s
			double da_e = SimDT * GLOBE_E_SPEED;

			//globe orbital track animation
			globe_o_proc = globe_o_proc - da_o;
			orbit_count = orbit_count + da_o;
			ink_vit_flag = 2;
			if (globe_o_proc > 1) globe_o_proc -= 1;
			else if (globe_o_proc < 0) globe_o_proc += 1;
			if (orbit_count >= 1000) orbit_count -= 1000;
			SetAnimation(anim_globe_o, globe_o_proc);

			// correct globe e animation axis with rotation matrix around x for the initial vector for the poles in the mesh file
			double angGlobeO = 2 * PI * globe_o_proc;
			globe_e_axis.x = GLOBE_E_AXIS_INIT.x;
			globe_e_axis.y = GLOBE_E_AXIS_INIT.y * cos(angGlobeO) - GLOBE_E_AXIS_INIT.z * sin(angGlobeO);
			globe_e_axis.z = GLOBE_E_AXIS_INIT.y * sin(angGlobeO) + GLOBE_E_AXIS_INIT.z * cos(angGlobeO);
			globe_e->axis = globe_e_axis;

			//globe daily rotation animation
			globe_e_proc = globe_e_proc + da_e;
			if (globe_e_proc > 1) globe_e_proc -= 1;
			else if (globe_e_proc < 0) globe_e_proc += 1;
			SetAnimation(anim_globe_e, globe_e_proc);
		}


		if (sep_status < 3) {

			//thermal sensor automatic separation
			if (!thermals_off && hAttached == NULL) {
				if (sep_status == 0 && GetAltitude() < 110000) sep_status = 1;
				else if (sep_status == 2 && GetAltitude() < 110000) {
					sep_status = 3;
					thermals_off = 0;
				}
			}

			main_prop = oapiGetPropellantMass(hpr);
			dpo_prop = oapiGetPropellantMass(hpr_dpo);
			do_prop = oapiGetPropellantMass(hpr_do);
			do_res_prop = oapiGetPropellantMass(hpr_do_res);

			if (sep_status == 0) {
				if (engine_status == 0) deltav = 278 * G * log((main_prop + dpo_prop + do_prop + do_res_prop + S7K_EMPTYMASS) / (S7K_EMPTYMASS + dpo_prop + do_prop + do_res_prop));
				else deltav = 270 * G * log((main_prop + dpo_prop + do_prop + do_res_prop + S7K_EMPTYMASS) / (S7K_EMPTYMASS + dpo_prop + do_prop + do_res_prop));
			}
			else {
				if (engine_status == 0) deltav = 278 * G * log((main_prop + dpo_prop + do_prop + do_res_prop + (S7K_EMPTYMASS - 1350)) / ((S7K_EMPTYMASS - 1350) + dpo_prop + do_prop + do_res_prop));
				else deltav = 270 * G * log((main_prop + dpo_prop + do_prop + do_res_prop + (S7K_EMPTYMASS - 1350)) / ((S7K_EMPTYMASS - 1350) + dpo_prop + do_prop + do_res_prop));
			}
			if (deltav < 150.0 && fuel_alarm == 0) {
				avaria = 1;
				fuel_alarm = 1;
				m_pXRSound->PlayWav(AlarmSound, true, 1.0);
			}

			thrust_rot = GetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN) + GetThrusterGroupLevel(THGROUP_ATT_PITCHUP) + GetThrusterGroupLevel(THGROUP_ATT_YAWLEFT) + GetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT) + GetThrusterGroupLevel(THGROUP_ATT_BANKLEFT) + GetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT);
			thrust_trans_dpo = GetThrusterGroupLevel(THGROUP_ATT_LEFT) + GetThrusterGroupLevel(THGROUP_ATT_RIGHT) + GetThrusterGroupLevel(THGROUP_ATT_UP) + GetThrusterGroupLevel(THGROUP_ATT_DOWN) + GetThrusterGroupLevel(THGROUP_ATT_FORWARD) + GetThrusterGroupLevel(THGROUP_ATT_BACK);
			thrust_main = GetThrusterGroupLevel(THGROUP_MAIN);

			// SKDU manual commands
			if (command_skdu_on == 1) {
				command_skdu_on = 2;
			}
			else if (command_skdu_on == 2) {
				SetThrusterGroupLevel(THGROUP_MAIN, 1.0);
			}
			else if (command_skdu_on == 3) {
				SetThrusterGroupLevel(THGROUP_MAIN, 0.0);
				command_skdu_on = 0;
			}

			//switch to DPO
			if (rcs_status == 3) SetRCSMode(DPO_MODE);

			//switch to DO
			else if (rcs_status == 1) SetRCSMode(DO_MODE);

			if (sep_status == 0 && igla_status != 0) {
				if (tgt_range > 1000) igla_4_limit = 0.35;
				else igla_4_limit = 0.2;
				if (target_oriented != 2 && target_Vessel != NULL) target_oriented = target_Vessel->GetIglaState();
				IglaData(SimDT);
				//Igla();
				if (target_Vessel->GetIglaState() == 0) target_Vessel->ActivateIgla(1);
			}
			if (sep_status == 0 && man_appr) {
				IglaData(SimDT);
			}
		
			
			// calculate sun relative vectors for orientation
			// sun is origin of global coordinate system
			Global2Local(_V(0, 0, 0), sun_localpos);
			sun_mag = sqrtl(pow(sun_localpos.x, 2) + pow(sun_localpos.y, 2) + pow(sun_localpos.z, 2));

			sun_unitvect.x = sun_localpos.x / sun_mag;
			sun_unitvect.y = sun_localpos.y / sun_mag;
			sun_unitvect.z = sun_localpos.z / sun_mag;

			// dot product for angle between two vectors, sun and ship +Y
			sun_err = acos(dotp(sun_unitvect, _V(0, 1, 0)));

			// angle to ship X axis for roll align
			sun_err_plusx = acos(dotp(sun_unitvect, _V(1, 0, 0)));
			sun_err_minusx = acos(dotp(sun_unitvect, _V(-1, 0, 0)));
			sun_err_plusz = acos(dotp(sun_unitvect, _V(0, 0, 1)));
			sun_err_minusz = acos(dotp(sun_unitvect, _V(0, 0, -1)));

			//RO mode
			if (pulsed_status == 0 && BDUS_status) {

				if (rotation_command[0] == 1) {
					xflag = 1;
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
				}
				else if (rotation_command[1] == 1) {
					xflag = 2;
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
				}
				if (rotation_command[2] == 1) {
					yflag = 1;
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				}
				else if (rotation_command[3] == 1) {
					yflag = 2;
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				}
				if (rotation_command[4] == 1) {
					zflag = 1;
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
				}
				else if (rotation_command[5] == 1) {
					zflag = 2;
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
				}

				//input release
				if (rotation_command[0] == 4) {
					xflag = 3;
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
				}
				else if (rotation_command[1] == 4) {
					xflag = 4;
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
				}
				if (rotation_command[2] == 4) {
					yflag = 3;
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				}
				else if (rotation_command[3] == 4) {
					yflag = 4;
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				}
				if (rotation_command[4] == 4) {
					zflag = 3;
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
				}
				else if (rotation_command[5] == 4) {
					zflag = 4;
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
				}

				//start rotation
				if (xflag == 1) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
					rotation_command[0] = 2;
				}
				else if (xflag == 2) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
					rotation_command[1] = 2;
				}
				if (yflag == 1) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
					rotation_command[2] = 2;
				}
				else if (yflag == 2) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
					rotation_command[3] = 2;
				}
				if (zflag == 1) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
					rotation_command[4] = 2;
				}
				else if (zflag == 2) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
					rotation_command[5] = 2;
				}

				//start nulling
				if (xflag == 3) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
					rotation_command[0] = 0;
				}
				else if (xflag == 4) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
					rotation_command[1] = 0;
				}
				if (yflag == 3) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
					rotation_command[2] = 0;
				}
				else if (yflag == 4) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
					rotation_command[3] = 0;
				}
				if (zflag == 3) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
					rotation_command[4] = 0;
				}
				else if (zflag == 4) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
					rotation_command[5] = 0;
				}


				//start coast
				if ((xflag == 1 && avel.x >= 0.5 && RO_detent[0] == 2) || (xflag == 1 && avel.x >= 3.0 && RO_detent[0] == 1)) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					xflag = 0;
					rotation_command[0] = 3;
				}
				else if ((xflag == 2 && avel.x <= -0.5 && RO_detent[1] == 2) || (xflag == 2 && avel.x <= -3.0 && RO_detent[1] == 1)) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					xflag = 0;
					rotation_command[1] = 3;
				}
				if ((yflag == 1 && avel.y >= 1.0 && RO_detent[2] == 2) || (yflag == 1 && avel.y >= 3.0 && RO_detent[2] == 1)) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					yflag = 0;
					rotation_command[2] = 3;
				}
				else if ((yflag == 2 && avel.y <= -1.0 && RO_detent[3] == 2) || (yflag == 2 && avel.y <= -3.0 && RO_detent[3] == 1)) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					yflag = 0;
					rotation_command[3] = 3;
				}
				if ((zflag == 1 && avel.z <= -0.5 && RO_detent[4] == 2) || (zflag == 1 && avel.z <= -3.0 && RO_detent[4] == 1)) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					zflag = 0;
					rotation_command[4] = 3;
				}
				else if ((zflag == 2 && avel.z >= 0.5 && RO_detent[5] == 2) || (zflag == 2 && avel.z >= 3.0 && RO_detent[5] == 1)) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					zflag = 0;
					rotation_command[5] = 3;
				}


				//thrusters off
				if (xflag == 3 && avel.x <= 0.0) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					xflag = 0;
					//rotation_command[0] = 0;
				}
				else if (xflag == 4 && avel.x >= 0.0) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					xflag = 0;
					//rotation_command[1] = 0;
				}
				if (yflag == 3 && avel.y <= 0.0) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					yflag = 0;
					//rotation_command[2] = 0;
				}
				else if (yflag == 4 && avel.y >= 0.0) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					yflag = 0;
					//rotation_command[3] = 0;
				}
				if (zflag == 3 && avel.z >= 0.0) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					zflag = 0;
					//rotation_command[4] = 0;
				}
				else if (zflag == 4 && avel.z <= 0.0) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					zflag = 0;
					//rotation_command[5] = 0;
				}

			}

			//DPO pulsed mode
			else if (pulsed_status == 1 && rcs_status == 0 && BDUS_status) {

				if (rotation_command[0] == 0 && rotation_command[1] == 0 && xflag == 0) last_omega.x = avel.x;
				if (rotation_command[2] == 0 && rotation_command[3] == 0 && yflag == 0) last_omega.y = avel.y;
				if (rotation_command[4] == 0 && rotation_command[5] == 0 && zflag == 0) last_omega.z = avel.z;
				if (rotation_command[0] == 1 || rotation_command[1] == 1 || rotation_command[2] == 1 || rotation_command[3] == 1 || rotation_command[4] == 1 || rotation_command[5] == 1) {
					if (rotation_command[0] == 1) {
						xflag = 1;
					}
					else if (rotation_command[1] == 1) {
						xflag = 2;
					}
					if (rotation_command[2] == 1) {
						yflag = 1;
					}
					else if (rotation_command[3] == 1) {
						yflag = 2;
					}
					if (rotation_command[5] == 1) {
						zflag = 1;
					}
					else if (rotation_command[4] == 1) {
						zflag = 2;
					}
				}

				double diffx = avel.x - last_omega.x;
				double diffy = avel.y - last_omega.y;
				double diffz = avel.z - last_omega.z;

				if (xflag == 1 && diffx >= 0.06) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					xflag = 0;
					rotation_command[0] = 0;
				}
				else if (xflag == 2 && diffx <= -0.06) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					xflag = 0;
					rotation_command[1] = 0;
				}
				if (yflag == 1 && diffy >= 0.06) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					yflag = 0;
					rotation_command[2] = 0;
				}
				else if (yflag == 2 && diffy <= -0.06) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					yflag = 0;
					rotation_command[3] = 0;
				}
				if (zflag == 1 && diffz >= 0.077) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					zflag = 0;
					rotation_command[5] = 0;
				}
				else if (zflag == 2 && diffz <= -0.077) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					zflag = 0;
					rotation_command[4] = 0;
				}


				if (xflag == 1) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
					rotation_command[0] = 0;
				}
				else if (xflag == 2) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
					rotation_command[1] = 0;
				}
				if (yflag == 1) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
					rotation_command[2] = 0;
				}
				else if (yflag == 2) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
					rotation_command[3] = 0;
				}
				if (zflag == 1) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
					rotation_command[5] = 0;
				}
				else if (zflag == 2) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
					rotation_command[4] = 0;
				}
			}
		
			//DO pulsed mode
			else if (pulsed_status == 1 && rcs_status == 2 && BDUS_status) {


				if (rotation_command[0] == 0 && rotation_command[1] == 0 && xflag == 0) last_omega.x = avel.x;
				if (rotation_command[2] == 0 && rotation_command[3] == 0 && yflag == 0) last_omega.y = avel.y;
				if (rotation_command[4] == 0 && rotation_command[5] == 0 && zflag == 0) last_omega.z = avel.z;
				if (rotation_command[0] == 1 || rotation_command[1] == 1 || rotation_command[2] == 1 || rotation_command[3] == 1 || rotation_command[4] == 1 || rotation_command[5] == 1) {
					if (rotation_command[0] == 1) {
						xflag = 1;
					}
					else if (rotation_command[1] == 1) {
						xflag = 2;
					}
					if (rotation_command[2] == 1) {
						yflag = 1;
					}
					else if (rotation_command[3] == 1) {
						yflag = 2;
					}
					if (rotation_command[5] == 1) {
						zflag = 1;
					}
					else if (rotation_command[4] == 1) {
						zflag = 2;
					}
				}

				double diffx = avel.x - last_omega.x;
				double diffy = avel.y - last_omega.y;
				double diffz = avel.z - last_omega.z;

				if (xflag == 1 && diffx >= 0.007) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					xflag = 0;
					rotation_command[0] = 0;
				}
				else if (xflag == 2 && diffx <= -0.007) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					xflag = 0;
					rotation_command[1] = 0;
				}
				if (yflag == 1 && diffy >= 0.006) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
					yflag = 0;
					rotation_command[2] = 0;
				}
				else if (yflag == 2 && diffy <= -0.006) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
					yflag = 0;
					rotation_command[3] = 0;
				}
				if (zflag == 1 && diffz >= 0.030) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
					zflag = 0;
					rotation_command[5] = 0;
				}
				else if (zflag == 2 && diffz <= -0.030) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
					zflag = 0;
					rotation_command[4] = 0;
				}


				if (xflag == 1) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
					rotation_command[0] = 0;
				}
				else if (xflag == 2) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
					rotation_command[1] = 0;
				}
				if (yflag == 1) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
					rotation_command[2] = 0;
				}
				else if (yflag == 2) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
					rotation_command[3] = 0;
				}
				if (zflag == 1) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
					rotation_command[5] = 0;
				}
				else if (zflag == 2) {
					SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
					rotation_command[4] = 0;
				}
			}

			// Inertial Attitude Hold
			if (inertial_status != 0) {
				Inertial(MJD, SimDT);
			}
			// Timer on Modes OFF signal
			if (modes_off != 0) {
				if ((MJD - signal_start) >= 0.000115741046) 
					modes_off = 0;
				ori_ap_status = 0;		//switch orbital orientation mode off, regardless of current state
				ori_ap_aux = 0;
				inertial_status = 0;	//switch inertial orientation mode off, regardless of current state
				BDUS_status = BDUS_OFF;
				pulsed_status = 2;
				DeactivateNavmode(1);
				SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
				SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
			}

			//Orbital Orientation Mode
			//if (ori_ap_status == 0) ori_ap_aux = 0;

			//Roll search
			if (ori_ap_status == 1) {
				

				if (ori_ap_aux == 0 && GetBank() * DEG <= 2 && GetBank() * DEG >= -2) ori_ap_status = 2;
				else {

					if (ori_ap_aux == 0 && avel.z < 0.450) {
						SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 0;
					}
					else if (ori_ap_aux == 0 && avel.z > 0.450) {
						SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 1;
					}
					if (ori_ap_aux == 1) {
						if (ori_ap_roll == 0) SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
						else if (ori_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);

						if (ori_ap_roll == 0 && avel.z >= 0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
							ori_ap_aux = 2;
							ori_ap_roll = 2;
						}
						else if (ori_ap_roll == 1 && avel.z <= 0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
							ori_ap_aux = 2;
							ori_ap_roll = 2;
						}
						else if (GetBank() * DEG <= (avel.z / 0.450 * 3.5) && GetBank() * DEG >= (avel.z / 0.450 * -3.5)) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
							SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
							ActivateNavmode(1);
							ori_ap_aux = 3;
							ori_ap_roll = 2;
						}
					}
					if (ori_ap_aux == 2) {
						if (GetBank() * DEG <= 3.7 && GetBank() * DEG >= -3.7) {
							ActivateNavmode(1);
							ori_ap_aux = 3;
						}
					}
					if (ori_ap_aux == 3 && GetNavmodeState(1) == false) {
						ori_ap_status = 2;
						ori_ap_aux = 0;
					}

				}

			}
			//Pitch nulling
			else if (ori_ap_status == 2) {

				if (ori_ap_aux == 0 && GetPitch() * DEG <= 2 && GetPitch() * DEG >= -2) ori_ap_status = 3;
				else {

					if (ori_ap_aux == 0 && GetPitch() * DEG < -2) {
						SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 0;
					}
					else if (ori_ap_aux == 0 && GetPitch() * DEG > 2) {
						SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 1;
					}
					if (ori_ap_aux == 1) {
						if (ori_ap_roll == 0) SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
						else if (ori_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);

						if (ori_ap_roll == 0 && avel.x >= 0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
							ori_ap_aux = 2;
							ori_ap_roll = 2;
						}
						else if (ori_ap_roll == 1 && avel.x <= -0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
							ori_ap_aux = 2;
							ori_ap_roll = 2;
						}
						else if (GetPitch() * DEG <= (abs(avel.x) / 0.450 * 3.2) && GetPitch() * DEG >= (abs(avel.x) / 0.450 * -3.2)) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
							SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
							ActivateNavmode(1);
							ori_ap_aux = 3;
							ori_ap_roll = 2;
						}
					}
					if (ori_ap_aux == 2) {
						if (GetPitch() * DEG <= 3.4 && GetPitch() * DEG >= -3.4) {
							ActivateNavmode(1);
							ori_ap_aux = 3;
						}
					}
					if (ori_ap_aux == 3 && GetNavmodeState(1) == false) {
						ori_ap_status = 3;
						ori_ap_aux = 0;
					}

				}

			}
			//Yaw nulling - orb_ori_dir == 0 for prograde, orb_ori_dir == 1 for retrograde
			else if (ori_ap_status == 3) {

				if (orb_ori_dir == 0 && ori_ap_aux == 0 && GetSlipAngle() * DEG <= 2 && GetSlipAngle() * DEG >= -2) ori_ap_status = 4;
				else if (orb_ori_dir == 1 && ori_ap_aux == 0 && (GetSlipAngle() * DEG >= 178 || GetSlipAngle() * DEG <= -178)) ori_ap_status = 4;
				else {

					if (ori_ap_aux == 0 && orb_ori_dir == 0 && GetSlipAngle() * DEG < -2) {
						SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 0;
					}
					else if (ori_ap_aux == 0 && orb_ori_dir == 0 && GetSlipAngle() * DEG > 2) {
						SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 1;
					}
					else if (ori_ap_aux == 0 && orb_ori_dir == 1 && GetSlipAngle() * DEG < 178) {
						SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 0;
					}
					else if (ori_ap_aux == 0 && orb_ori_dir == 1 && GetSlipAngle() * DEG > -178) {
						SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 1;
					}

				
					if (ori_ap_aux == 1) {
						if (ori_ap_roll == 0) SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
						else if (ori_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);

						if (ori_ap_roll == 0 && avel.y <= -0.45) {
							SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
							ori_ap_aux = 2;
							ori_ap_roll = 2;
						}
						else if (ori_ap_roll == 1 && avel.y >= 0.45) {
							SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
							ori_ap_aux = 2;
							ori_ap_roll = 2;
						}
						else if (orb_ori_dir == 0 && GetSlipAngle() * DEG <= (abs(avel.y) / 0.450 * 3.4) && GetSlipAngle() * DEG >= (abs(avel.y) / 0.450 * -3.4)) {
							SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
							SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
							ActivateNavmode(1);
							ori_ap_aux = 3;
							ori_ap_roll = 2;
						}
						else if (orb_ori_dir == 1 && (GetSlipAngle() * DEG <= (abs(avel.y) / 0.450 * -176.6) || GetSlipAngle() * DEG >= (abs(avel.y) / 0.450 * 176.6))) {
							SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
							SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
							ActivateNavmode(1);
							ori_ap_aux = 3;
							ori_ap_roll = 2;
						}
					}
					if (ori_ap_aux == 2) {
						if (orb_ori_dir == 0 && GetSlipAngle() * DEG <= 3.6 && GetSlipAngle() * DEG >= -3.6) {
							ActivateNavmode(1);
							ori_ap_aux = 3;
						}
						else if (orb_ori_dir == 1 && (GetSlipAngle() * DEG <= -176.4 || GetSlipAngle() * DEG >= 176.4)) {
							ActivateNavmode(1);
							ori_ap_aux = 3;
						}
					}
					if (ori_ap_aux == 3 && GetNavmodeState(1) == false) {
						ori_ap_status = 4;
						ori_ap_aux = 0;
					}
				}
			}
			//Roll retune
			else if (ori_ap_status == 4) {

				if (ori_ap_aux == 0 && GetBank() * DEG <= 2 && GetBank() * DEG >= -2) ori_ap_status = 5;
				else {

					if (ori_ap_aux == 0 && GetBank() * DEG > 2) {
						SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 0;
					}
					else if (ori_ap_aux == 0 && GetBank() * DEG < -2) {
						SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 1;
					}
					if (ori_ap_aux == 1) {
						if (ori_ap_roll == 0) SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
						else if (ori_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);

						if (ori_ap_roll == 0 && avel.z >= 0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
							ori_ap_aux = 2;
							ori_ap_roll = 2;
						}
						else if (ori_ap_roll == 1 && avel.z <= -0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
							ori_ap_aux = 2;
							ori_ap_roll = 2;
						}
						else if (GetBank() * DEG <= (abs(avel.z) / 0.450 * 3.5) && GetBank() * DEG >= (abs(avel.z) / 0.450 * -3.5)) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
							SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
							ActivateNavmode(1);
							ori_ap_aux = 3;
							ori_ap_roll = 2;
						}
					}
					if (ori_ap_aux == 2) {
						if (GetBank() * DEG <= 3.7 && GetBank() * DEG >= -3.7) {
							ActivateNavmode(1);
							ori_ap_aux = 3;
						}
					}
					if (ori_ap_aux == 3 && GetNavmodeState(1) == false) {
						ori_ap_status = 5;
						ori_ap_aux = 0;
					}
				}
			}
			//Pitch retune
			else if (ori_ap_status == 5) {

				if (ori_ap_aux == 0 && GetPitch() * DEG <= 2 && GetPitch() * DEG >= -2) ori_ap_aux = 3;
				else {

					if (ori_ap_aux == 0 && GetPitch() * DEG < -2) {
						SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 0;
					}
					else if (ori_ap_aux == 0 && GetPitch() * DEG > 2) {
						SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 1;
					}
					if (ori_ap_aux == 1) {
						if (ori_ap_roll == 0) SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
						else if (ori_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);

						if (ori_ap_roll == 0 && avel.x >= 0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
							ori_ap_aux = 2;
							ori_ap_roll = 2;
						}
						else if (ori_ap_roll == 1 && avel.x <= -0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
							ori_ap_aux = 2;
							ori_ap_roll = 2;
						}
						else if (GetPitch() * DEG <= (abs(avel.x) / 0.450 * 3.2) && GetPitch() * DEG >= (abs(avel.x) / 0.450 * -1.6)) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
							SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
							ActivateNavmode(1);
							ori_ap_aux = 3;
							ori_ap_roll = 2;
						}
					}
					if (ori_ap_aux == 2) {
						if (GetPitch() * DEG <= 3.4 && GetPitch() * DEG >= -3.4) {
							ActivateNavmode(1);
							ori_ap_aux = 3;
						}
					}
					if (ori_ap_aux == 3 && GetNavmodeState(1) == false) {
						if (orb_ori_dir == 0) SetAngularVel(_V(-2 * PI / orbparam.T, avel.y * RAD, avel.z * RAD)); //gyro rate command to maintain pitch 
						else SetAngularVel(_V(2 * PI / orbparam.T, avel.y * RAD, avel.z * RAD)); //gyro rate command to maintain pitch 
						ori_ap_status = 6;
						ori_ap_aux = 0;
					}
				}
			}
			//att hold here
			else if (ori_ap_status == 6) {
				curr_slip = GetSlipAngle() * DEG;
				curr_pitch = GetPitch() * DEG;
				rcs_status = 1;

				if (orb_ori_dir == 0 && ori_ap_aux_yaw == 0 && curr_slip > -att_hold_limit && curr_slip < att_hold_limit) ori_ap_aux_yaw = 4;
				else if (orb_ori_dir == 1 && ori_ap_aux_yaw == 0 && (curr_slip > 180 - att_hold_limit || curr_slip < -180 + att_hold_limit)) ori_ap_aux_yaw = 4;

				//start yaw orientation
				if (ori_ap_aux_yaw == 0 && orb_ori_dir == 0 && curr_slip < -att_hold_limit) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
					ori_ap_roll = 1;
					ori_ap_aux_yaw = 1;
					last_slip = curr_slip;
					if (oapiGetTimeAcceleration() != 1) oapiSetTimeAcceleration(1);
				}
				else if (ori_ap_aux_yaw == 0 && orb_ori_dir == 0 && curr_slip > att_hold_limit) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
					ori_ap_roll = 2;
					ori_ap_aux_yaw = 1;
					last_slip = curr_slip;
					if (oapiGetTimeAcceleration() != 1) oapiSetTimeAcceleration(1);
				}
				else if (ori_ap_aux_yaw == 0 && orb_ori_dir == 1 && curr_slip < 180 - att_hold_limit) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
					ori_ap_roll = 1;
					ori_ap_aux_yaw = 1;
					last_slip = curr_slip;
					if (oapiGetTimeAcceleration() != 1) oapiSetTimeAcceleration(1);
				}
				else if (ori_ap_aux_yaw == 0 && orb_ori_dir == 1 && curr_slip > -180 + att_hold_limit) {
					SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
					ori_ap_roll = 2;
					ori_ap_aux_yaw = 1;
					last_slip = curr_slip;
					if (oapiGetTimeAcceleration() != 1) oapiSetTimeAcceleration(1);
				}

				//runtime
				if (ori_ap_aux_yaw == 1) {
					if (ori_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
					else if (ori_ap_roll == 2) SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);

					//stop conditions
					if (ori_ap_roll == 1 && avel.y <= -0.66) {
						SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
						ori_ap_aux_yaw = 2;
					}
					else if (ori_ap_roll == 2 && avel.y >= 0.66) {
						SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
						ori_ap_aux_yaw = 2;
					}
					else if ((curr_slip > (last_slip / 2) && last_slip < 0) || (curr_slip < (last_slip / 2) && last_slip > 0)) {
						SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
						SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
						ori_ap_aux_yaw = 2;
					}
				}

				// kill rotation in yaw only to maintain pitch orbital rate
				if (ori_ap_aux_yaw == 2 && (orb_ori_dir == 0 && curr_slip > -3.2 && curr_slip < 3.2) || (orb_ori_dir == 1 && (curr_slip < -176.8 || curr_slip > 176.8))) {
					if (ori_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
					else if (ori_ap_roll == 2) SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);

					//stop conditions
					if (ori_ap_roll == 1 && avel.y >= 0.0) {
						SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
						ori_ap_aux_yaw = 4;
						ori_ap_roll = 0;
					}
					else if (ori_ap_roll == 2 && avel.y <= 0.0) {
						SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
						ori_ap_aux_yaw = 4;
						ori_ap_roll = 0;
					}

				}

				if (ori_ap_aux_yaw == 4) {
					if (ori_ap_aux_pitch == 0 && curr_pitch > -att_hold_limit && curr_pitch < att_hold_limit) {
						ori_ap_aux_pitch = 3;
					}

					if (ori_ap_aux_pitch == 0 && curr_pitch < -att_hold_limit) {
						SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
						ori_ap_roll = 1;
						ori_ap_aux_pitch = 1;
						last_pitch = curr_pitch;
					}
					else if (ori_ap_aux_pitch == 0 && curr_pitch > att_hold_limit) {
						SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
						ori_ap_roll = 2;
						ori_ap_aux_pitch = 1;
						last_pitch = curr_pitch;
					}

					if (ori_ap_aux_pitch == 1) {
						if (ori_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
						else if (ori_ap_roll == 2) SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);

						//stop conditions
						if (ori_ap_roll == 1 && avel.x <= -0.66) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
							ori_ap_roll = 0;
							ori_ap_aux_pitch = 2;
						}
						else if (ori_ap_roll == 2 && avel.x >= 0.66) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
							ori_ap_roll = 0;
							ori_ap_aux_pitch = 2;
						}
						else if ((curr_pitch > (last_pitch / 2) && last_pitch < 0) || (curr_pitch < (last_pitch / 2) && last_pitch > 0)) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
							SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
							ori_ap_aux_pitch = 2;
							ori_ap_roll = 3;
						}
					}

					if (ori_ap_aux_pitch == 2) {
						if (ori_ap_roll == 0 && curr_pitch > -3.5 && curr_pitch < 3.5) {
							ori_ap_roll = 3;
						}
						else if (ori_ap_roll == 3) {
							ActivateNavmode(1);
							ori_ap_aux_pitch = 3;
						}
					}

					if (ori_ap_aux_pitch == 3 && GetNavmodeState(1) == false) {
						ori_ap_aux_pitch = 0;
						ori_ap_aux_yaw = 0;
						ori_ap_roll = 0;
						ori_ap_status = 7;
						if (orb_ori_dir == 0) SetAngularVel(_V(-2 * PI / orbparam.T, avel.y * RAD, avel.z * RAD)); //gyro rate command to maintain pitch 
						else SetAngularVel(_V(2 * PI / orbparam.T, avel.y * RAD, avel.z * RAD)); //gyro rate command to maintain pitch
					}

				}

			}
			else if (ori_ap_status == 7) {
				curr_roll = GetBank() * DEG;
				rcs_status = 1;

				if (ori_ap_aux == 0 && curr_roll <= att_hold_limit && curr_roll >= -att_hold_limit) ori_ap_status = 6;
				else {

					if (ori_ap_aux == 0 && curr_roll > att_hold_limit) {
						SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 1;
						last_roll = curr_roll;
						if (oapiGetTimeAcceleration() != 1) oapiSetTimeAcceleration(1);
					}
					else if (ori_ap_aux == 0 && curr_roll < -att_hold_limit) {
						SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
						ori_ap_aux = 1;
						ori_ap_roll = 2;
						last_roll = curr_roll;
						if (oapiGetTimeAcceleration() != 1) oapiSetTimeAcceleration(1);
					}
					if (ori_ap_aux == 1) {
						if (ori_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
						else if (ori_ap_roll == 2) SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);

						if (ori_ap_roll == 1 && avel.z >= 0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
							ori_ap_aux = 2;
						}
						else if (ori_ap_roll == 2 && avel.z <= -0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
							ori_ap_aux = 2;
						}
						else if ((curr_roll > (last_roll / 2) && last_roll < 0) || (curr_roll < (last_roll / 2) && last_roll > 0)) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
							SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
							ori_ap_aux = 2;
						}
					}
					//kill rotation in roll only to maintain pitch rate
					if (ori_ap_aux == 2 && curr_roll <= 3.7 && curr_roll >= -3.7) {
						if (ori_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
						else if (ori_ap_roll == 2) SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
					
						//stop conditions
						if (ori_ap_roll == 1 && avel.z <= 0.0) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
							ori_ap_aux = 3;
							ori_ap_roll = 0;
						}
						else if (ori_ap_roll == 2 && avel.z >= 0.0) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
							ori_ap_aux = 3;
							ori_ap_roll = 0;
						}
					
					}
					if (ori_ap_aux == 3) {
						ori_ap_status = 6;
						ori_ap_aux = 0;
					}
				}
			}
		

			//Solar Orientation Mode
			if (sun_ap_status == 0) sun_ap_aux = 0;

			//Pitch search
			else if (sun_ap_status == 1) {

				if (sun_ap_aux == 0 && sun_err * DEG < 90) sun_ap_status = 2; //sun already in view of sensor
				else {
					if (sun_ap_aux == 0 && avel.x < 0.450) {
						SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
						sun_ap_aux = 1;
						sun_ap_roll = 0;
					}
					else if (sun_ap_aux == 0 && avel.x > 0.450) {
						SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
						sun_ap_aux = 1;
						sun_ap_roll = 1;
					}
					if (sun_ap_aux == 1) {
						if (sun_ap_roll == 0) SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
						else if (sun_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);

						if (sun_ap_roll == 0 && avel.x >= 0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
							sun_ap_aux = 2;
							sun_ap_roll = 2;
						}
						else if (sun_ap_roll == 1 && avel.x <= 0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
							sun_ap_aux = 2;
							sun_ap_roll = 2;
						}
						else if (sun_err * DEG < 90) {		//sun already in view of sensor before burn is complete
							SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
							SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
							ActivateNavmode(1);
							sun_ap_aux = 3;
							sun_ap_roll = 2;
						}
					}
					if (sun_ap_aux == 2) {
						if (sun_err * DEG < 90) {			//sun in view of sensor
							ActivateNavmode(1);
							sun_ap_aux = 3;
						}
					}
					if (sun_ap_aux == 3 && GetNavmodeState(1) == false) {
						sun_ap_status = 2;
						sun_ap_aux = 0;
					}

				}

			}
			//Roll search
			else if (sun_ap_status == 2) {

				if (sun_ap_aux == 0 && sun_err * DEG < 6) sun_ap_status = 3;	//sun already within sensor margin
				else {
					//if -x smaller, bank left
					if (sun_ap_aux == 0 && sun_err_plusx * DEG < sun_err_minusx * DEG) {
						SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
						sun_ap_aux = 1;
						sun_ap_roll = 0;
					}
					else if (sun_ap_aux == 0 && sun_err_plusx * DEG > sun_err_minusx * DEG) {
						SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);
						sun_ap_aux = 1;
						sun_ap_roll = 1;
					}
					if (sun_ap_aux == 1) {
						if (sun_ap_roll == 0) SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 1);
						else if (sun_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 1);

						if (sun_ap_roll == 0 && avel.z >= 0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
							sun_ap_aux = 2;
							sun_ap_roll = 2;
						}
						else if (sun_ap_roll == 1 && avel.z <= -0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
							sun_ap_aux = 2;
							sun_ap_roll = 2;
						}
						else if (sun_err * DEG < 6 || sun_err > sun_err_last) {
							SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
							SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
							ActivateNavmode(1);
							sun_ap_aux = 3;
							sun_ap_roll = 2;
						}
					}
					if (sun_ap_aux == 2) {
						if (sun_err * DEG < 6 || sun_err > sun_err_last) {
							ActivateNavmode(1);
							sun_ap_aux = 3;
						}
					}
					if (sun_ap_aux == 3 && GetNavmodeState(1) == false) {
						sun_ap_status = 3;
						sun_ap_aux = 0;
					}

				}

			}
			//pitch tune
			else if (sun_ap_status == 3) {

				if (sun_ap_aux == 0 && sun_err * DEG < 6) sun_ap_status = 0;	//sun already within sensor margin
				else {
					//if +z smaller, pitch down
					if (sun_ap_aux == 0 && sun_err_plusz * DEG < sun_err_minusz * DEG) {
						SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
						sun_ap_aux = 1;
						sun_ap_roll = 0;
					}
					else if (sun_ap_aux == 0 && sun_err_plusz * DEG > sun_err_minusz * DEG) {
						SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
						sun_ap_aux = 1;
						sun_ap_roll = 1;
					}
					if (sun_ap_aux == 1) {
						if (sun_ap_roll == 0) SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
						else if (sun_ap_roll == 1) SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);

						if (sun_ap_roll == 0 && avel.x <= -0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
							sun_ap_aux = 2;
							sun_ap_roll = 2;
						}
						else if (sun_ap_roll == 1 && avel.x >= 0.450) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
							sun_ap_aux = 2;
							sun_ap_roll = 2;
						}
						else if (sun_err * DEG < 6) {
							SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
							SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
							ActivateNavmode(1);
							sun_ap_aux = 3;
							sun_ap_roll = 2;
						}
					}
					if (sun_ap_aux == 2) {
						if (sun_err * DEG < 6) {
							ActivateNavmode(1);
							sun_ap_aux = 3;
						}
					}
					if (sun_ap_aux == 3 && GetNavmodeState(1) == false) {
						sun_ap_status = 0;
						sun_ap_aux = 0;
					}
				}
			}
			sun_err_last = sun_err;


			//Animations step
			if (antennae_status == 1)
			{
				double da = SimDT * ANT_SPEED;
				antennae_proc = antennae_proc + da;
				if (antennae_proc > 1) {
					antennae_proc = 1;
					antennae_status = 2;
					igla_sides_status = 1;
					igla_sides_proc = 0;
				}
				SetAnimation(anim_antennae, antennae_proc);
			}
			else if (antennae_status == 2)
			{
				SetAnimation(anim_antennae, 1.0);
			}
			else if (antennae_status == 3)
			{
				double da = SimDT * ANT_SPEED;
				antennae_proc = antennae_proc - da;
				if (antennae_proc < 0) {
					antennae_proc = 0;
					antennae_status = 0;
				}
				SetAnimation(anim_antennae, antennae_proc);
			}
			else SetAnimation(anim_antennae, 0.0);


			///
			if (igla_sides_status == 1)
			{
				double da = SimDT * ANT_SPEED;
				igla_sides_proc = igla_sides_proc + da;
				if (igla_sides_proc > 1) {
					igla_sides_proc = 1;
					igla_sides_status = 2;
				}
				SetAnimation(anim_igla_sides, igla_sides_proc);
			}
			else if (igla_sides_status == 2)
			{
				SetAnimation(anim_igla_sides, 1.0);
			}
			else if (igla_sides_status == 3)
			{
				double da = SimDT * ANT_SPEED;
				igla_sides_proc = igla_sides_proc - da;
				if (igla_sides_proc < 0) {
					igla_sides_proc = 0;
					igla_sides_status = 0;
					antennae_status = 3;
					antennae_proc = 1;
				}
				SetAnimation(anim_igla_sides, igla_sides_proc);
			}
			else SetAnimation(anim_igla_sides, 0.0);

			// thermal sensors animation
			if (thermal_status == 1)
			{
				double da = SimDT * ION_SPEED;
				ion_proc = ion_proc + da;
				if (ion_proc > 1) {
					ion_proc = 1;
					thermal_status = 2;
				}
				SetAnimation(anim_ion, ion_proc);
			}
			else if (thermal_status == 2)
			{
				SetAnimation(anim_ion, 1.0);
			}
			else if (thermal_status == 3)
			{
				double da = SimDT * ION_SPEED;
				ion_proc = ion_proc - da;
				if (ion_proc < 0) {
					ion_proc = 0;
					thermal_status = 0;
				}
				SetAnimation(anim_ion, ion_proc);
			}
			else SetAnimation(anim_ion, 0.0);


			//docking probe animation - if not jettisoned
			if (hDock != NULL || (hDock == NULL && hooks_status == 5 && !CM_jett)) {
				if (sep_status == 0) hMother = GetDockStatus(hDock);
				else hMother = NULL;

				//disable Igla on docking
				if (hMother != NULL && (igla_status != 0 || man_appr)) IglaOff();
			
				if (dock_probe_status == 1 && ssvp_on)
				{
					double da = SimDT * DOCK_PROBE_SPEED;
					dock_probe_proc = dock_probe_proc + da;
					if (dock_probe_proc > 1) {
						dock_probe_proc = 1;
						dock_probe_status = 2;
						if (latches_status != 2) latches_status = 1;	//auto latch extend
					}
					SetAnimation(anim_dock_probe, dock_probe_proc);
				}
				else if (dock_probe_status == 2)
				{
					SetAnimation(anim_dock_probe, 1.0);
				}
				else if (dock_probe_status == 3 && ssvp_on)
				{
					double da = SimDT * DOCK_PROBE_SPEED;
					dock_probe_proc = dock_probe_proc - da;
					if (dock_probe_proc < 0) {
						dock_probe_proc = 0;
						dock_probe_status = 0;
					}
					SetAnimation(anim_dock_probe, dock_probe_proc);
				}
				else if (dock_probe_status == 4 && hooks_status == 2 && latches_status == 0) {	//if latches are retracted and hooks closed, continue probe retraction
					dock_probe_status = 3;
				}
				else SetAnimation(anim_dock_probe, dock_probe_proc);
			
				//Latches
				if (latches_status == 1 && ssvp_on)
				{
					double da = SimDT * LATCHES_SPEED;
					latches_proc = latches_proc + da;
					if (latches_proc > 1) {
						latches_proc = 1;
						latches_status = 2;
					}
					SetAnimation(anim_latches, latches_proc);
				}
				else if (latches_status == 2)
				{
					SetAnimation(anim_latches, 1.0);
				}
				else if (latches_status == 3 && ssvp_on)
				{
					double da = SimDT * LATCHES_SPEED;
					latches_proc = latches_proc - da;
					if (latches_proc < 0) {
						latches_proc = 0;
						latches_status = 0;
					}
					SetAnimation(anim_latches, latches_proc);
				}
				else SetAnimation(anim_latches, 0.0);

				//hooks
				if (hooks_status == 1 && ssvp_on)
				{
					double da = SimDT * HOOKS_SPEED;
					hooks_proc = hooks_proc + da;
					if (hooks_proc > 1) {
						hooks_proc = 1;
						hooks_status = 2;
						if (hMother) latches_status = 3;
					}
				}
				else if (hooks_status == 3 && ssvp_on)
				{
					double da = SimDT * HOOKS_SPEED;
					hooks_proc = hooks_proc - da;
					if (hooks_proc < 0) {
						hooks_proc = 0;
						hooks_status = 0;
						if (hMother && latches_status == 0) {
							Undock(0);
							combined_power = 0;
							recharge = 0;
							undock_commanded = 0;
							undock_signal = 1.0;
							if (CM_jett) {
								DelDock(hDock);
								hDock = NULL;
							}
						}
					}
				}

				//undocking signal
				if (undock_signal > 0) {
					double da = SimDT * TEN_SEC_SPEED;
					undock_signal = undock_signal - da;
					if (undock_signal < 0) {
						undock_signal = 0;
					}
				}


				//soft/hard dock
				if (hMother == NULL && sep_status == 0) {
					VECTOR3 new_dock_pos = S7K_DOCK_POS;
					new_dock_pos.z = new_dock_pos.z + (dock_probe_proc * new_dock_factor) + DOCKOFFSET;
					SetDockParams(hDock, new_dock_pos, _V(0, 0, 1), _V(0, 1, 0));
				}
				else if (hMother != NULL) {
					if (dock_probe_status == 1) {
						VESSEL* v = oapiGetVesselInterface(hMother);
						int nDock;
						ELEMENTS el_v;
						ORBITPARAM orbparam_v;
						nDock = v->DockCount();
						for (DWORD j = 0; j < nDock; j++) {
							DOCKHANDLE hMotherDock = v->GetDockHandle(j);
							OBJHANDLE hChild = v->GetDockStatus(hMotherDock);
							if (hChild == GetHandle() && dock_probe_proc > 0.18) {
								VECTOR3 mtr_am;
								v->GetAngularVel(mtr_am);
								v->GetElements(NULL, el_v, &orbparam_v, 0, FRAME_EQU);
								Undock(0);
								VECTOR3 new_dock_pos = S7K_DOCK_POS;
								new_dock_pos.z = new_dock_pos.z + (dock_probe_proc * new_dock_factor) + DOCKOFFSET;
								SetDockParams(hDock, new_dock_pos, _V(0, 0, 1), _V(0, 1, 0));
								Dock(hMother, 0, j, 2);
								v->SetAngularVel(mtr_am);
								v->SetElements(NULL, el_v, &orbparam_v, 0, FRAME_EQU);
							}
						}

					}
					else if (dock_probe_status == 3) {
						VESSEL* v = oapiGetVesselInterface(hMother);
						ELEMENTS el_v;
						ORBITPARAM orbparam_v;
						int nDock;
						nDock = v->DockCount();
						for (DWORD j = 0; j < nDock; j++) {
							DOCKHANDLE hMotherDock = v->GetDockHandle(j);
							OBJHANDLE hChild = v->GetDockStatus(hMotherDock);
							if (hChild == GetHandle() && dock_probe_proc > 0.18) {
								VECTOR3 mtr_am, globvel;
								v->GetAngularVel(mtr_am);
								v->GetElements(NULL, el_v, &orbparam_v, 0, FRAME_EQU);
								Undock(0);
								VECTOR3 new_dock_pos = S7K_DOCK_POS;
								new_dock_pos.z = new_dock_pos.z + (dock_probe_proc * new_dock_factor) + DOCKOFFSET;
								SetDockParams(hDock, new_dock_pos, _V(0, 0, 1), _V(0, 1, 0));
								Dock(hMother, 0, j, 2);
								v->SetElements(NULL, el_v, &orbparam_v, 0, FRAME_EQU);
							}
							else if (hChild == GetHandle() && hooks_status == 0) {
								dock_probe_status = 4;
								hooks_status = 1;
							}
							//else if (hChild == GetHandle()) {
								//dock_probe_status = 5;
							//}
						}

					}
					else if ((dock_probe_status == 2 && latches_status != 2) || (dock_probe_status == 0 && hooks_status != 2 && undock_commanded == 0)) {
						Undock(0);
						combined_power = 0;
						recharge = 0;
					}
				}
			}
		}

	}  //for the "if battery" condition - else force systems off
	else {
		SetThrusterGroupLevel(THGROUP_MAIN, 0);
		SetThrusterGroupLevel(THGROUP_RETRO, 0);
	}

}

// --------------------------------------------------------------
// Vessel initialisation
// --------------------------------------------------------------
DLLCLBK VESSEL *ovcInit (OBJHANDLE hvessel, int flightmodel)
{
	return new Soyuz7k_T (hvessel, flightmodel);
}

// --------------------------------------------------------------
// Vessel cleanup
// --------------------------------------------------------------
DLLCLBK void ovcExit (VESSEL *vessel)
{
	if (vessel) delete (Soyuz7k_T*)vessel;
}


//Notes:
//Approach (data from Soyuz 10):
//15 km - 24 m/s
//6 km - 27 m/s - first flip with DPO
//4 km - 11 m/s
//2.5 km - 8 m/s
//1.6 km - 8 m/s
//1.2 km - 4 m/s
//0.8 km - 4 m/s
//0.5 km - 2 m/s
//Final  - 0.3 m/s

//retro fuel consumption: ~6.3 kg/[m/s] (old ISP)

//Chertok (Soyuz 15): at 3km and 20 km, Igla fired engines; at 20 km, there is tolerance for deviation from target 