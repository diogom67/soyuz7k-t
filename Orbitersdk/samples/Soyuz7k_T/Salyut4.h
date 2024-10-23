//-----------------------------------------------------------------------------------------
//Copyright 2022 diogom
// 
//Permission is hereby granted, free of charge, to any person obtaining a copy of this softwareand associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and /or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :
//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// 
//-----------------------------------------------------------------------------------------


#ifndef __SALYUT4
#define __SALYUT4

#include "orbitersdk.h"


class Salyut4: public VESSEL4 {
public:
	Salyut4 (OBJHANDLE hVessel, int flightmodel);
	//~Salyut4 ();
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
	PARTICLESTREAMSPEC main_exhaust = {
		0, 0.9, 60.0, 2746.8, 0.0, 0.04, 1300.0, 2.0, PARTICLESTREAMSPEC::EMISSIVE,
		PARTICLESTREAMSPEC::LVL_PSQRT, 0.0, 0.5,
		PARTICLESTREAMSPEC::ATM_FLAT, 0.2, 0.2,
		NULL
	};
	PARTICLESTREAMSPEC DPO_exhaust = {
		0, 0.01, 60.0, 1147.77, 0.0, 0.03, 600.0, 2.0, PARTICLESTREAMSPEC::EMISSIVE,
		PARTICLESTREAMSPEC::LVL_PSQRT, 0.0, 0.5,
		PARTICLESTREAMSPEC::ATM_FLAT, 0.2, 0.2,
		NULL
	};
};


void Salyut4::ActivateIgla(int action) {
	igla_status = action;
}

int Salyut4::GetIglaState() {
	return igla_status;
}


#endif