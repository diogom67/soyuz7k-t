//-----------------------------------------------------------------------------------------
//Copyright 2022 diogom
// 
//Permission is hereby granted, free of charge, to any person obtaining a copy of this softwareand associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and /or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :
//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// 
//-----------------------------------------------------------------------------------------


#define STRICT
#define ORBITER_MODULE

#include "orbitersdk.h"
#include <cstdio>
#include <cmath>
#include <string>
#include <stdio.h>



// ==============================================================
// Some vessel parameters
// ==============================================================
const double  S4_SIZE			= 17;             // mean radius [m]
const VECTOR3 S4_CS			= {60.95,46.84,14.69}; // x,y,z cross sections [m^2]
const VECTOR3 S4_PMI			= {26.82,30.97,26.34};// principal moments of inertia (mass-normalised) [m^2]
const VECTOR3 S4_RD			= {0.050,0.050,0.04};//{0.05,0.1,0.05};  // rotation drag coefficients
const double  S4_EMPTYMASS		= 16210.0;          // empty vessel mass [kg]
const double  S4_FUELMASS		= 1800.0;           // max fuel mass [kg]
const double  S4_FUELDPOMASS	= 200.0;			// max fuel mass [kg]
const double  S4_ISP			= 2746.8;			// fuel-specific impulse [m/s]
const double  S4_DPOISP			= 1147.77;			// fuel-specific impulse [m/s]
const VECTOR3 S4_COP			= {0,0,0};			// centre of pressure for airfoils [m]
const double  S4_VLIFT_C		= 2.0;				// chord length [m]
const double  S4_VLIFT_S		= 2.0;				// wing area [m^2]
const double  S4_VLIFT_A		= 2.5;				// wing aspect ratio
const double  S4_HLIFT_C		= 2.0;				// chord length [m]
const double  S4_HLIFT_S		= 1.5;				// wing area [m^2]
const double  S4_HLIFT_A		= 2.0;				// wing aspect ratio

const double  S4_MAXMAINTH  	= 4090;             
const double  S4_MAXBACKUPTH  	= 4030;             
const double  S4_MAXRCSTH_1   	= 98;
const double  S4_MAXRCSTH_2 	= 59;
const double  S4_MAXRCSTH_3    	= 20;

const VECTOR3 S4_DOCK_POS   = {0,0,7.613};      // docking port location [m]
const VECTOR3 S4_DOCK_DIR   = {0,0,1};         // docking port approach direction
const VECTOR3 S4_DOCK_ROT   = {0,1,0};        // docking port alignment direction
const double IGLA_RANGE = 15000;


VECTOR3 bscol = _V(1, 1, 1);
VECTOR3 bsred = _V(1, 0.1, 0.1);
VECTOR3 bsgreen = _V(0, 1, 0);


// Define impact convex hull
static const DWORD ntdvtx = 12;
static TOUCHDOWNVTX tdvtx[ntdvtx] = {
	{_V( 0,  -1.5, 2  ), 2e4, 1e3, 1.6, 1},
	{_V(-1,  -1.5,-1.5), 2e4, 1e3, 3.0, 1},
	{_V( 1,  -1.5,-1.5), 2e4, 1e3, 3.0, 1},
	{_V(-0.5,-0.75,3  ), 2e4, 1e3, 3.0},
	{_V( 0.5,-0.75,3  ), 2e4, 1e3, 3.0},
	{_V(-2.6,-1.1,-1.9), 2e4, 1e3, 3.0},
	{_V( 2.6,-1.1,-1.9), 2e4, 1e3, 3.0},
	{_V(-1,   1.3, 0  ), 2e4, 1e3, 3.0},
	{_V( 1,   1.3, 0  ), 2e4, 1e3, 3.0},
	{_V(-1,   1.3,-2  ), 2e4, 1e3, 3.0},
	{_V( 1,   1.3,-2  ), 2e4, 1e3, 3.0},
	{_V( 0,   0.3,-3.8), 2e4, 1e3, 3.0}
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
// Salyut4 class interface
// ==============================================================

class Salyut4: public VESSEL4 {
public:
	Salyut4 (OBJHANDLE hVessel, int flightmodel);
	~Salyut4 ();
	void clbkSetClassCaps (FILEHANDLE cfg);
	int clbkConsumeBufferedKey(DWORD key, bool down, char* kstate);
	void clbkPreStep(double SimT, double SimDT, double MJD);
	void clbkPostStep(double SimT, double SimDT, double MJD);
	void clbkLoadStateEx(FILEHANDLE scn, void* vs);
	void clbkSaveState(FILEHANDLE scn);
	bool clbkDrawHUD(int mode, const HUDPAINTSPEC * hps, oapi::Sketchpad * skp);
	void clbkPostCreation();
	void ActivateIgla(int action);
	int GetIglaState();
	

private:
	void GetAngularVel();
	static void vlift (VESSEL *v, double aoa, double M, double Re,
		void *context, double *cl, double *cm, double *cd);
	static void hlift (VESSEL *v, double aoa, double M, double Re,
		void *context, double *cl, double *cm, double *cd);
	void IglaData(double SimDT);
	void Igla();
	void IglaOff();
	int engine_status, pulsed_status, ori_ap_status, ori_ap_aux, ori_ap_roll, sun_ap_status, sun_ap_aux, sun_ap_roll;
	int xflag = 0, yflag = 0, zflag = 0, killrot = 0;
	double ion_err, IR_err, sun_err, sun_err_last, sun_err_plusx, sun_err_minusx, sun_err_plusz, sun_err_minusz;
	double tgt_range, tgt_range_rate, acc_SimDT, sun_mag, tgt_err, tgt_err_yaw, tgt_err_pitch, last_tgt_err_yaw, last_tgt_err_pitch;
	double mag;						//for sun calculation
	ATTACHMENTHANDLE attach1;
	DOCKHANDLE hDock;
	OBJHANDLE hMother;
	MESHHANDLE hMesh;
	THRUSTER_HANDLE th_main[2], th_trans_fwd[4], th_RCS[8], th_group[2];
	PROPELLANT_HANDLE hpr, hpr_dpo;
	BEACONLIGHTSPEC beac1, beac2, beac3, beac4, beac5;
	VECTOR3 localpos, unitvect;
	VECTOR3 beac1_pos, beac2_pos, beac3_pos, beac4_pos, beac5_pos;
	VECTOR3 last_omega, avel;
	VECTOR3 tgt_relpos, tgt_relvel, last_tgtpos, sun_localpos, sun_unitvect, tgt_unitvect, tgt_globpos;
	SpotLight *spotlight1;
	UINT mesh_idx;
	VESSEL* target_Vessel;
	char targetName[11];
	int igla_status, relvel_sample;
	int igla_ap_pitch, igla_ap_yaw, igla_ap_roll, igla_ap_aux_pitch, igla_ap_aux_yaw, igla_ap_aux_roll, igla_ap_aux_vert, igla_ap_aux_lat, igla_ap_vert, igla_ap_lat;
	ORBITPARAM orbparam;
	ELEMENTS el;
	PARTICLESTREAMSPEC main_exhaust = {
		0, 0.9, 60.0, S4_ISP, 0.0, 0.04, 1300.0, 2.0, PARTICLESTREAMSPEC::EMISSIVE,
		PARTICLESTREAMSPEC::LVL_PSQRT, 0.0, 0.5,
		PARTICLESTREAMSPEC::ATM_FLAT, 0.2, 0.2,
		NULL
	};
	PARTICLESTREAMSPEC DPO_exhaust = {
		0, 0.01, 60.0, S4_DPOISP, 0.0, 0.03, 600.0, 2.0, PARTICLESTREAMSPEC::EMISSIVE,
		PARTICLESTREAMSPEC::LVL_PSQRT, 0.0, 0.5,
		PARTICLESTREAMSPEC::ATM_FLAT, 0.2, 0.2,
		NULL
	};
};

Salyut4::Salyut4 (OBJHANDLE hVessel, int flightmodel)
: VESSEL4 (hVessel, flightmodel)
{
}

Salyut4::~Salyut4 ()
{
}


void Salyut4::ActivateIgla(int action) {
	 igla_status = action;
}

int Salyut4::GetIglaState() {
	return igla_status;
}

void Salyut4::clbkSaveState(FILEHANDLE scn) {

	char cbuf[256];

	// default vessel parameters
	VESSEL4::clbkSaveState(scn);

	//store DPO/DO config
	int engine_store;
	if (engine_status == 0) engine_store = 3;
	else engine_store = 1;

	sprintf(cbuf, "%d", engine_store);
	oapiWriteScenario_string(scn, "RCS", cbuf);

	//store target name
	sprintf(cbuf, "%s", targetName);
	oapiWriteScenario_string(scn, "TGT", cbuf);
}

void Salyut4::clbkLoadStateEx(FILEHANDLE scn, void* vs) {

	char* line;

	while (oapiReadScenario_nextline(scn, line)) {
		if (!_strnicmp(line, "RCS", 3)) {
			sscanf(line + 3, "%d", &engine_status);
		}
		else if (!_strnicmp(line, "TGT", 3)){
			sscanf(line + 3, "%s", targetName);
		}
		else {
			ParseScenarioLineEx(line, vs);
			// unrecognised option - pass to Orbiter's generic parser
		}
	}
}

void Salyut4::clbkPostCreation() {

	OBJHANDLE hTarget = oapiGetVesselByName(targetName);
	if(hTarget != NULL) target_Vessel = oapiGetVesselInterface(hTarget);

}

void Salyut4::GetAngularVel() {

	VESSEL3::GetAngularVel(avel);

	avel.x = avel.x * DEG;
	avel.y = avel.y * DEG;
	avel.z = avel.z * DEG;

}

void Salyut4::IglaOff() {
	igla_status = 0;
	SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
	SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
	SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
	SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
	SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
	SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
}

void Salyut4::IglaData(double SimDT) {

	//Get target docking port position in ecliptic
	if(target_Vessel != NULL) target_Vessel->Local2Global(_V(0, 0, 0), tgt_globpos);

	//Target position transformed to local Soyuz coordinates
	Global2Local(tgt_globpos, tgt_relpos);

	//Range as vector magnitude
	tgt_range = sqrtl(pow(tgt_relpos.x, 2) + pow(tgt_relpos.y, 2) + pow(tgt_relpos.z, 2));

	//Relative orientation
	tgt_unitvect.x = tgt_relpos.x / tgt_range;
	tgt_unitvect.y = tgt_relpos.y / tgt_range;
	tgt_unitvect.z = tgt_relpos.z / tgt_range;

	tgt_err_yaw = asin(-tgt_unitvect.x) * DEG;
	tgt_err_pitch = asin(tgt_unitvect.y) * DEG;

	if (tgt_unitvect.z < 0) {
		if (tgt_unitvect.x < 0) tgt_err_yaw = 180 - tgt_err_yaw;
		else tgt_err_yaw = -180 - tgt_err_yaw;

		if (tgt_unitvect.y > 0) tgt_err_pitch = 180 - tgt_err_pitch;
		else tgt_err_pitch = -180 - tgt_err_pitch;
	}

	//Get target docking port position in ecliptic
	if (target_Vessel != NULL) target_Vessel->Local2Global(_V(0, 0, 4.4), tgt_globpos);

	//Target position transformed to local Soyuz coordinates
	Global2Local(tgt_globpos, tgt_relpos);

	tgt_relpos.z -= 7.514;
	tgt_range = sqrtl(pow(tgt_relpos.x, 2) + pow(tgt_relpos.y, 2) + pow(tgt_relpos.z, 2));

}

void Salyut4::Igla() {

	//int igla_ap_pitch, igla_ap_yaw, igla_ap_roll;
	//pitch flag 1 is down, flag 2 is up, flag 0 is off
	//yaw flag 1 is right, flag 2 is left, flag 0 is off


	//Orientation
	//neg pitch, pitch down
	//neg yaw, yaw right
	if (igla_status == 1) {

		if (igla_ap_aux_yaw == 0 && tgt_err_yaw > -2 && tgt_err_yaw < 2) igla_ap_aux_yaw = 4;
		

		//start yaw orientation
		if (igla_ap_aux_yaw == 0 && tgt_err_yaw < -2.0) {
			SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
			igla_ap_yaw = 1;
			igla_ap_aux_yaw = 1;
		}
		else if (igla_ap_aux_yaw == 0 && tgt_err_yaw > 2.0) {
			SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
			igla_ap_yaw = 2;
			igla_ap_aux_yaw = 1;
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
			}
			else if (igla_ap_yaw == 2 && avel.y >= 0.66) {
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				igla_ap_yaw = 0;
				igla_ap_aux_yaw = 2;
			}
			else if (tgt_err_yaw > -4 && tgt_err_yaw < 4) {
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				igla_ap_aux_yaw = 2;
			}
		}

		if (igla_ap_aux_yaw == 2 && tgt_err_yaw > -4.5 && tgt_err_yaw < 4.5) {
			ActivateNavmode(1);
			igla_ap_aux_yaw = 3;
		}

		if (igla_ap_aux_yaw == 3 && GetNavmodeState(1) == false) {
			igla_ap_aux_yaw = 4;
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
			}
			else if (igla_ap_aux_pitch == 0 && tgt_err_pitch > 2.0) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
				igla_ap_pitch = 2;
				igla_ap_aux_pitch = 1;
			}

			if (igla_ap_aux_pitch == 1) {
				if (igla_ap_pitch == 1) SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
				else if (igla_ap_pitch == 2) SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);

				//stop conditions
				if (igla_ap_pitch == 1 && avel.x <= -0.66) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					igla_ap_pitch = 0;
					igla_ap_aux_pitch = 2;
				}
				else if (igla_ap_pitch == 2 && avel.x >= 0.66) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					igla_ap_pitch = 0;
					igla_ap_aux_pitch = 2;
				}
				else if (tgt_err_pitch > -3.4 && tgt_err_pitch < 3.4) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					igla_ap_aux_pitch = 2;
				}
			}
			
			if (igla_ap_aux_pitch == 2 && tgt_err_pitch > -3.7 && tgt_err_pitch < 3.7) {
				ActivateNavmode(1);
				igla_ap_aux_pitch = 3;
			}

			if (igla_ap_aux_pitch == 3 && GetNavmodeState(1) == false) {
				igla_ap_aux_pitch = 4;
				igla_ap_aux_yaw = 5;
			}

		}

		if (igla_ap_aux_pitch == 4 && igla_ap_aux_yaw == 5) {
			igla_status = 2;
			igla_ap_aux_pitch = 0;
			igla_ap_aux_yaw = 0;
		}


	}

	else if (igla_status == 2) {
		if (igla_ap_aux_yaw == 0 && tgt_err_yaw > -0.3 && tgt_err_yaw < 0.3) igla_ap_aux_yaw = 4;

		//start yaw orientation
		if (igla_ap_aux_yaw == 0 && tgt_err_yaw < -0.3) {
			SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 1);
			igla_ap_yaw = 1;
			igla_ap_aux_yaw = 1;
			last_tgt_err_yaw = tgt_err_yaw;
		}
		else if (igla_ap_aux_yaw == 0 && tgt_err_yaw > 0.3) {
			SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 1);
			igla_ap_yaw = 2;
			igla_ap_aux_yaw = 1;
			last_tgt_err_yaw = tgt_err_yaw;
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
			}
			else if (igla_ap_yaw == 2 && avel.y >= 0.66) {
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				igla_ap_yaw = 0;
				igla_ap_aux_yaw = 2;
			}
			else if ((tgt_err_yaw > (last_tgt_err_yaw / 2) && last_tgt_err_yaw < 0) || (tgt_err_yaw < (last_tgt_err_yaw / 2) && last_tgt_err_yaw > 0)) {
				SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
				SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
				igla_ap_aux_yaw = 2;
				igla_ap_yaw = 0;
			}
		}

		if (igla_ap_aux_yaw == 2) {

			if (igla_ap_yaw == 0 && tgt_err_yaw > -4.5 && tgt_err_yaw < 4.5) {
				igla_ap_yaw = 3;
			}

			else if (igla_ap_yaw == 3) {
				ActivateNavmode(1);
				igla_ap_aux_yaw = 3;
			}
		}

		if (igla_ap_aux_yaw == 3 && GetNavmodeState(1) == false) {
			igla_ap_aux_yaw = 4;
		}

		if (igla_ap_aux_yaw == 4) {
			if (igla_ap_aux_pitch == 0 && tgt_err_pitch > -0.3 && tgt_err_pitch < 0.3) {
				igla_ap_aux_pitch = 4;
				igla_ap_aux_yaw = 5;
			}

			if (igla_ap_aux_pitch == 0 && tgt_err_pitch < -0.3) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
				igla_ap_pitch = 1;
				igla_ap_aux_pitch = 1;
				last_tgt_err_pitch = tgt_err_pitch;
			}
			else if (igla_ap_aux_pitch == 0 && tgt_err_pitch > 0.3) {
				SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);
				igla_ap_pitch = 2;
				igla_ap_aux_pitch = 1;
				last_tgt_err_pitch = tgt_err_pitch;
			}

			if (igla_ap_aux_pitch == 1) {
				if (igla_ap_pitch == 1) SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 1);
				else if (igla_ap_pitch == 2) SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 1);

				//stop conditions
				if (igla_ap_pitch == 1 && avel.x <= -0.66) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					igla_ap_pitch = 0;
					igla_ap_aux_pitch = 2;
				}
				else if (igla_ap_pitch == 2 && avel.x >= 0.66) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					igla_ap_pitch = 0;
					igla_ap_aux_pitch = 2;
				}
				else if ((tgt_err_pitch > (last_tgt_err_pitch / 2) && last_tgt_err_pitch < 0) || (tgt_err_pitch < (last_tgt_err_pitch / 2) && last_tgt_err_pitch > 0)) {
					SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
					SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
					igla_ap_aux_pitch = 2;
					igla_ap_pitch = 3;
				}
			}

			if (igla_ap_aux_pitch == 2) {

				if (igla_ap_pitch == 0 && tgt_err_pitch > -3.7 && tgt_err_pitch < 3.7) {
					igla_ap_pitch = 3;
				}

				else if (igla_ap_pitch == 3) {
					ActivateNavmode(1);
					igla_ap_aux_pitch = 3;
				}
			}

			if (igla_ap_aux_pitch == 3 && GetNavmodeState(1) == false) {
				igla_ap_aux_pitch = 4;
				igla_ap_aux_yaw = 5;
			}

		}

		if (igla_ap_aux_pitch == 4 && igla_ap_aux_yaw == 5) {
			igla_status = 2;
			igla_ap_aux_pitch = 0;
			igla_ap_aux_yaw = 0;
		}

	}
}



// ==============================================================
// Overloaded callback functions
// ==============================================================

// --------------------------------------------------------------
// Set the capabilities of the vessel class
// --------------------------------------------------------------
void Salyut4::clbkSetClassCaps (FILEHANDLE cfg)
{
	// physical vessel parameters
	SetSize (S4_SIZE);
	SetEmptyMass (S4_EMPTYMASS+300);
	SetPMI (S4_PMI);
	SetCrossSections (S4_CS);
	SetRotDrag (S4_RD);
	SetTouchdownPoints (tdvtx, ntdvtx);

	// docking port definitions
	hDock = CreateDock(S4_DOCK_POS, S4_DOCK_DIR, S4_DOCK_ROT);

    attach1 = CreateAttachment(true, _V(0, 0, -4.497), _V(0, 0, -1), _V(1, 0, 0), "Proton");

	beac1_pos = _V(-0.2792, -1.479, 6.29);
	beac2_pos = _V(0.006, 1.7456, 4.145);
	beac3_pos = _V(9.97, -0.11, 3.4);
	beac4_pos = _V(-9.92, 0.18, 3.4);
	beac5_pos = _V(0.11, 10, 3.4);
	
	beac1.shape = BEACONSHAPE_STAR;
	beac1.pos = &beac1_pos;
	beac1.col = &bscol;
	beac1.size = 0.07;
	beac1.falloff = 0.6;
	beac1.period = 0.7;
	beac1.duration = 0.35;
	beac1.tofs = 0.1;
	beac1.active = true;
	AddBeacon(&beac1);

	
	beac2.shape = BEACONSHAPE_STAR;
	beac2.pos = &beac2_pos;
	beac2.col = &bscol;
	beac2.size = 0.1;
	beac2.falloff = 0.6;
	beac2.period = 0;
	beac2.duration = 0.35;
	beac2.tofs = 0;
	beac2.active = true;
	AddBeacon(&beac2);

	beac3.shape = BEACONSHAPE_STAR;
	beac3.pos = &beac3_pos;
	beac3.col = &bsred;
	beac3.size = 0.12;
	beac3.falloff = 0.6;
	beac3.period = 0;
	beac3.duration = 0.35;
	beac3.tofs = 0;
	beac3.active = true;
	AddBeacon(&beac3);

	beac4.shape = BEACONSHAPE_STAR;
	beac4.pos = &beac4_pos;
	beac4.col = &bsgreen;
	beac4.size = 0.12;
	beac4.falloff = 0.6;
	beac4.period = 0;
	beac4.duration = 0.35;
	beac4.tofs = 0;
	beac4.active = true;
	AddBeacon(&beac4);

	beac5.shape = BEACONSHAPE_STAR;
	beac5.pos = &beac5_pos;
	beac5.col = &bscol;
	beac5.size = 0.12;
	beac5.falloff = 0.6;
	beac5.period = 0;
	beac5.duration = 0.35;
	beac5.tofs = 0;
	beac5.active = true;
	AddBeacon(&beac5);


	COLOUR4 col_d = { 1,1,1,0 };
	COLOUR4 col_s = { 1,1,1,0 };
	COLOUR4 col_a = { 1,1,1,0 };
	spotlight1 = (SpotLight*)AddSpotLight(beac2_pos, _V(0, 0, 1), 45, 0.01, 0, 0.1, 45*RAD, 100*RAD, col_d, col_s, col_a);
	spotlight1->Activate(true);

		
	// airfoil definitions
	CreateAirfoil3 (LIFT_VERTICAL,   S4_COP, vlift, NULL, S4_VLIFT_C, S4_VLIFT_S, S4_VLIFT_A);
	CreateAirfoil3 (LIFT_HORIZONTAL, S4_COP, hlift, NULL, S4_HLIFT_C, S4_HLIFT_S, S4_HLIFT_A);
	
	pulsed_status = 0;
	engine_status = 3;
	sun_ap_status = 0;
	ori_ap_status = 0;
	ori_ap_aux = 0;
	ori_ap_roll = 2;
	igla_status = 0;
	
	// propellant resources
	hpr = CreatePropellantResource (S4_FUELMASS);
	hpr_dpo = CreatePropellantResource (S4_FUELDPOMASS);

	//main/backup
	th_main[0] = CreateThruster(_V(0, 0, -6.37), _V(0, 0, 1), S4_MAXMAINTH, hpr, S4_ISP);
	AddExhaustStream(th_main[0], &main_exhaust);

	th_main[1] = CreateThruster(_V(0, 0, -6.37), _V(0, 0, 1), S4_MAXBACKUPTH, hpr, S4_ISP);
	AddExhaustStream(th_main[1], &main_exhaust);

	// RCS engines
	th_trans_fwd[0] = CreateThruster(_V(0, 0.38, -6.37), _V(0, 1, 1), S4_MAXRCSTH_1, hpr_dpo, S4_DPOISP);
	th_trans_fwd[1] = CreateThruster(_V(0, -0.38, -6.37), _V(0, -1, 1), S4_MAXRCSTH_1, hpr_dpo, S4_DPOISP);
	th_trans_fwd[2] = CreateThruster(_V(-0.385, 0, -6.37), _V(-1, 0, 1), S4_MAXRCSTH_1, hpr_dpo, S4_DPOISP);
	th_trans_fwd[3] = CreateThruster(_V(0.385, 0, -6.37), _V(1, 0, 1), S4_MAXRCSTH_1, hpr_dpo, S4_DPOISP);
	
	th_RCS[0] = CreateThruster(_V(-0.937, 0, -6.84), _V(-1, 0, 1), S4_MAXRCSTH_2, hpr_dpo, S4_DPOISP);		//yaw right
	th_RCS[1] = CreateThruster(_V(0.937, 0, -6.84), _V(1, 0, 1), S4_MAXRCSTH_2, hpr_dpo, S4_DPOISP);		//yaw left
	th_RCS[2] = CreateThruster(_V(0, 1.21, -6.622), _V(0, -1, 0), S4_MAXRCSTH_2, hpr_dpo, S4_DPOISP);		//pitch up
	th_RCS[3] = CreateThruster(_V(0, -1.21, -6.622), _V(0, 1, 0), S4_MAXRCSTH_2, hpr_dpo, S4_DPOISP);		//pitch down
	th_RCS[4] = CreateThruster(_V(-0.333, 1.199, -6.5), _V(1, -1, 0), S4_MAXRCSTH_2, hpr_dpo, S4_DPOISP);	//roll right 1
	th_RCS[5] = CreateThruster(_V(0.333, -1.199, -6.5), _V(-1, 1, 0), S4_MAXRCSTH_2, hpr_dpo, S4_DPOISP);	//roll right 2
	th_RCS[6] = CreateThruster(_V(0.333, 1.199, -6.5), _V(-1, -1, 0), S4_MAXRCSTH_2, hpr_dpo, S4_DPOISP);	//roll left 1
	th_RCS[7] = CreateThruster(_V(-0.333, -1.199, -6.5), _V(1, 1, 0), S4_MAXRCSTH_2, hpr_dpo, S4_DPOISP);	//roll left 2
	
	for (int i = 0; i < 4; i++) {
		AddExhaustStream(th_trans_fwd[i], &DPO_exhaust);
	}
	CreateThrusterGroup(th_trans_fwd, 4, THGROUP_ATT_FORWARD);

	for (int i = 0; i < 8; i++) {
		AddExhaustStream(th_RCS[i], &DPO_exhaust);
	}

	th_group[0] = th_RCS[0];
	CreateThrusterGroup(th_group, 1, THGROUP_ATT_YAWRIGHT);

	th_group[0] = th_RCS[1];
	CreateThrusterGroup(th_group, 1, THGROUP_ATT_YAWLEFT);

	th_group[0] = th_RCS[2];
	CreateThrusterGroup(th_group, 1, THGROUP_ATT_PITCHUP);

	th_group[0] = th_RCS[3];
	CreateThrusterGroup(th_group, 1, THGROUP_ATT_PITCHDOWN);

	th_group[0] = th_RCS[4];
	th_group[1] = th_RCS[5];
	CreateThrusterGroup(th_group, 2, THGROUP_ATT_BANKRIGHT);

	th_group[0] = th_RCS[6];
	th_group[1] = th_RCS[7];
	CreateThrusterGroup(th_group, 2, THGROUP_ATT_BANKLEFT);

	// camera parameters
	SetCameraOffset (_V(0,-2.5,0));

	// associate a mesh for the visual
	hMesh = oapiLoadMeshGlobal("Soyuz_7k\\Salyut4");
	mesh_idx = AddMesh(hMesh);
	SetMeshVisibilityMode(mesh_idx, MESHVIS_ALWAYS);

}

bool Salyut4::clbkDrawHUD(int mode, const HUDPAINTSPEC* hps, oapi::Sketchpad* skp) {

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

	if (oapiCockpitMode() != COCKPIT_VIRTUAL)
	{
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				double main_prop = oapiGetPropellantMass(hpr);
				if (ori_ap_status != 0) {
					sprintf(abuf, "Orbital Orientation Mode %d", ori_ap_status);
					skp->Text((roxl + 400), (l1 + royl), abuf, strlen(abuf));
				}
				else if (sun_ap_status != 0) {
					sprintf(abuf, "Solar Orientation Mode %d", sun_ap_status);
					skp->Text((roxl + 400), (l1 + royl), abuf, strlen(abuf));
				}
				else if (igla_status != 0) {
					sprintf(abuf, "Igla %d", igla_status);
					skp->Text((roxl + 400), (l1 + royl), abuf, strlen(abuf));
				}
			}
			if (tgt_range < IGLA_RANGE /* && igla_status != 0*/) {
				{
					skp->SetTextColor(0x0066FF66);
					char abuf[256];
					if (tgt_range < 1000) sprintf(abuf, "Target range: %0.1lf m", tgt_range);
					else sprintf(abuf, "Target range: %0.2lf km", tgt_range / 1000);
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
			}

			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				double main_prop = oapiGetPropellantMass(hpr);
				sprintf(abuf, "SKDU propellant: %0.1lf kg", main_prop);
				skp->Text((roxl), (w1 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				double dpo_prop = oapiGetPropellantMass(hpr_dpo);
				sprintf(abuf, "DPO propellant: %0.1lf kg", dpo_prop);
				skp->Text((roxl), (w2 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (engine_status == 0) sprintf(abuf, "Engine: SKD");
				else sprintf(abuf, "Engine: DKD");
				skp->Text((roxl), (w3 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (beac1.active == false) sprintf(abuf, "Docking lights: OFF");
				else sprintf(abuf, "Docking lights: ON");
				skp->Text((roxl), (w4 + royl), abuf, strlen(abuf));
			}


			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "-- Angular Velocity --");
				skp->Text((roxl), (w6 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Wx: %+0.3lf deg/s", avel.x);
				skp->Text((roxl), (w7 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Wy: %+0.3lf deg/s", avel.y);
				skp->Text((roxl), (w8 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Wz: %+0.3lf deg/s", avel.z);
				skp->Text((roxl), (w9 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (pulsed_status == 0) sprintf(abuf, "RCS mode: RO");
				else sprintf(abuf, "RCS mode: Pulsed-RO");
				skp->Text((roxl), (w10 + royl), abuf, strlen(abuf));
			}


			double phi = GetSlipAngle();
			double theta = PI / 2 - GetPitch();
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
				skp->Text((roxl), (w13 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				sprintf(abuf, "Ion err: %+.0f deg", ion_err * DEG);
				skp->Text((roxl), (w14 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (IR_err * DEG < 100) sprintf(abuf, "IR err: %.0f deg", IR_err * DEG);
				else sprintf(abuf, "IR err: -- deg");
				skp->Text((roxl), (w15 + royl), abuf, strlen(abuf));
			}
			{
				skp->SetTextColor(0x0066FF66);
				char abuf[256];
				if (sun_err * DEG < 6) {
					sprintf(abuf, "45K aligned");
					skp->Text((roxl), (w16 + royl), abuf, strlen(abuf));
				}
			}
			
	}
	return true;
}

int Salyut4::clbkConsumeBufferedKey(DWORD key, bool down, char* kstate) {
	
	if (!down) return 0;
	
	switch (key) {

		//switch RCS pulsed mode
	case OAPI_KEY_F:
		if (pulsed_status == 0) pulsed_status = 1;	//switch to pulsed mode
		else pulsed_status = 0;						//switch to continuous mode
		return 1;

	case OAPI_KEY_E:
		if (engine_status == 0) engine_status = 1;	
		else if (engine_status == 1) engine_status = 3;				
		else if (engine_status == 2) engine_status = 3;		
		else engine_status = 1;		
		return 1;

		//orbital orientation autopilot
	case OAPI_KEY_O:
		if (ori_ap_status == 0 && sun_ap_status == 0) {
			ori_ap_status = 1;		//switch orbital orientation mode on
		}
		else if (ori_ap_status != 0) ori_ap_status = 0;		//switch orbital orientation mode off, regardless of current state
		return 1;

		//solar orientation autopilot
	case OAPI_KEY_K:
		if (sun_ap_status == 0 && ori_ap_status == 0) {
			sun_ap_status = 1;		//switch solar orientation mode on
		}
		else if (sun_ap_status != 0) sun_ap_status = 0;		//switch solar orientation mode off, regardless of current state
		return 1;

	case OAPI_KEY_I:
		if (igla_status == 0) igla_status = 1;	
		else {
			igla_status = 0;
			SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0);
			SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0);
			SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0);
		}
		return 1;

		//lights on/off
	case OAPI_KEY_L:
		if (beac1.active == false) {
			beac1.active = true;
			spotlight1->Activate(true);
			beac2.active = true;
			beac3.active = true;
			beac4.active = true;
			beac5.active = true;
		}
		else {
			beac1.active = false;
			spotlight1->Activate(false);
			beac2.active = false;
			beac3.active = false;
			beac4.active = false;
			beac5.active = false;
		}
		return 1;
	}

	return 0;
}


// ==============================================================
// Airfoil lift/drag functions
// ==============================================================

void Salyut4::vlift (VESSEL *v, double aoa, double M, double Re,
	void *context, double *cl, double *cm, double *cd)
{
	static const double clp[] = {  // lift coefficient from -pi to pi in 10deg steps
		-0.1,-0.5,-0.4,-0.1,0,0,0,0,0,0,0,0,0,0,-0.2,-0.6,-0.6,-0.4,0.2,0.5,0.9,0.8,0.2,0,0,0,0,0,0,0,0,0,0.1,0.4,0.5,0.3,-0.1,-0.5
	};
	static const double aoa_step = 10.0*RAD;
	double a, fidx, saoa = sin(aoa);
	a = modf((aoa+PI)/aoa_step, &fidx);
	int idx = (int)(fidx+0.5);
	*cl = clp[idx]*(1.0-a) + clp[idx+1]*a;     // linear interpolation
	*cm = 0.0; //-0.03*sin(aoa-0.1);
	*cd = 0.03 + 0.4*saoa*saoa;                // profile drag
	*cd += oapiGetInducedDrag (*cl, 1.0, 0.5); // induced drag
	*cd += oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);  // wave drag
}

void Salyut4::hlift (VESSEL *v, double aoa, double M, double Re,
	void *context, double *cl, double *cm, double *cd)
{
	static const double clp[] = {  // lift coefficient from -pi to pi in 45deg steps
		0,0.4,0,-0.4,0,0.4,0,-0.4,0,0.4
	};
	static const double aoa_step = 45.0*RAD;
	double a, fidx;
	a = modf((aoa+PI)/aoa_step, &fidx);
	int idx = (int)(fidx+0.5);
	*cl = clp[idx]*(1.0-a) + clp[idx+1]*a;     // linear interpolation
	*cm = 0.0;
	*cd = 0.03;
	*cd += oapiGetInducedDrag (*cl, 1.5, 0.6); // induced drag
	*cd += oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);  // wave drag
}



// ==============================================================
// API callback interface
// ==============================================================

void Salyut4::clbkPostStep(double SimT, double SimDT, double MJD) {

	GetAngularVel();

	if (igla_status != 0) {
		IglaData(SimDT);
	}
}

void Salyut4::clbkPreStep(double SimT, double SimDT, double MJD) {

	GetElements(NULL, el, &orbparam, 0, FRAME_EQU);

	Global2Local(_V(0, 0, 0), localpos);
	mag = sqrtl(pow(localpos.x, 2) + pow(localpos.y, 2) + pow(localpos.z, 2));

	unitvect.x = localpos.x / mag;
	unitvect.y = localpos.y / mag;
	unitvect.z = localpos.z / mag;

	sun_err = acos(dotp(unitvect, _V(0, 1, 0)));
	sun_err_plusx = acos(dotp(unitvect, _V(1, 0, 0)));
	sun_err_minusx = acos(dotp(unitvect, _V(-1, 0, 0)));
	sun_err_plusz = acos(dotp(unitvect, _V(0, 0, 1)));
	sun_err_minusz = acos(dotp(unitvect, _V(0, 0, -1)));

	if (engine_status == 3) {
		// main engine
		th_group[0] = th_main[0];
		CreateThrusterGroup(th_group, 1, THGROUP_MAIN);
		engine_status = 0;
	}
	else if (engine_status == 1) {
		// backup engine
		th_group[0] = th_main[1];
		CreateThrusterGroup(th_group, 1, THGROUP_MAIN);
		engine_status = 2;
	}

	if (igla_status == 2 && tgt_range < 3) {
		IglaOff();
		ActivateNavmode(1);
	}

	if (igla_status != 0) {
		Igla();
	}

		

}


// --------------------------------------------------------------
// Vessel initialisation
// --------------------------------------------------------------
DLLCLBK VESSEL *ovcInit (OBJHANDLE hvessel, int flightmodel)
{
	return new Salyut4 (hvessel, flightmodel);
}

// --------------------------------------------------------------
// Vessel cleanup
// --------------------------------------------------------------
DLLCLBK void ovcExit (VESSEL *vessel)
{
	if (vessel) delete (Salyut4*)vessel;
}


//Notes:
/*
Salyut 4 - . Payload: Zarya s/n 124. Mass: 18,500 kg (40,700 lb). Nation: USSR. Agency: MOM. Program: Salyut. Class: Manned. Type: Manned space station. Spacecraft: Salyut 4. Duration: 768.82 days. Decay Date: 1977-02-02 . USAF Sat Cat: 7591 . COSPAR: 1974-104A. Apogee: 251 km (155 mi). Perigee: 212 km (131 mi). Inclination: 51.6000 deg. Period: 89.10 min. Deorbited February 2, 1977.
Maneuver Summary:
211km X 250km orbit to 215km X 286km orbit. Delta V: 11 m/s
211km X 284km orbit to 276km X 344km orbit. Delta V: 35 m/s
277km X 342km orbit to 338km X 351km orbit. Delta V: 19 m/s
330km X 340km orbit to 337km X 350km orbit. Delta V: 4 m/s
337km X 349km orbit to 339km X 351km orbit. Delta V: 1 m/s
332km X 348km orbit to 348km X 355km orbit. Delta V: 6 m/s
347km X 354km orbit to 343km X 351km orbit. Delta V: 1 m/s
335km X 344km orbit to 335km X 360km orbit. Delta V: 4 m/s
335km X 360km orbit to 342km X 361km orbit. Delta V: 2 m/s
330km X 351km orbit to 344km X 353km orbit. Delta V: 4 m/s
186km X 187km orbit to 90km X 186km orbit. Delta V: 28 m/s
Total Delta V: 87/115 m/s.

RCS Coarse No x Thrust: Verniers: 4 X 98 N - Pitch: 2 X 59 N - Yaw: 2 X 59 N - Roll: 2 X 20 N. RCS Fine No x Thrust: 18 x 10 N. RCS Coarse Backup No x Thrust: 4 x 98 N. Spacecraft delta v: 320 m/s (1,040 ft/sec). Electric System: 2.00 average kW.
*/