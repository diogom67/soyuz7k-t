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
#include <string>
#include <stdio.h>


// ==============================================================
// Some vessel parameters
// ==============================================================
const double  S7K_PAO_SIZE			= 3;             // mean radius [m]
const VECTOR3 S7K_PAO_CS			= {6.8,6.7,5.4}; // x,y,z cross sections [m^2]
const VECTOR3 S7K_PAO_PMI			= {5.8,7.5,2.65};// principal moments of inertia (mass-normalised) [m^2]
const VECTOR3 S7K_PAO_RD			= {0.025,0.025,0.02};//{0.05,0.1,0.05};  // rotation drag coefficients
const double  S7K_PAO_EMPTYMASS		= 2700.0;           // empty vessel mass [kg]
const VECTOR3 S7K_PAO_COP			= {0,0,0};	      // centre of pressure for airfoils [m]
const double  S7K_PAO_VLIFT_C		= 2.0;             // chord length [m]
const double  S7K_PAO_VLIFT_S		= 2.0;             // wing area [m^2]
const double  S7K_PAO_VLIFT_A		= 2.5;             // wing aspect ratio
const double  S7K_PAO_HLIFT_C		= 2.0;             // chord length [m]
const double  S7K_PAO_HLIFT_S		= 1.5;             // wing area [m^2]
const double  S7K_PAO_HLIFT_A		= 2.0;             // wing aspect ratio

const double THERM_SPEED = 0.3;

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
// Soyuz7k_T_PAO class interface
// ==============================================================

class Soyuz7k_T_PAO: public VESSEL3 {
public:
	Soyuz7k_T_PAO (OBJHANDLE hVessel, int flightmodel);
	~Soyuz7k_T_PAO ();
	void clbkSetClassCaps (FILEHANDLE cfg);
	void clbkPreStep(double SimT, double SimDT, double MJD);

private:
	static void vlift (VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd);
	static void hlift (VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd);
	void DefineAnimations();
	UINT mesh[9];
	UINT anim_therm_cov;
	int therm_cov_status;
	double therm_cov_proc;
	MGROUP_TRANSFORM* rot_thermal1, * rot_thermal2, * rot_thermal3, * rot_thermal4, * rot_thermal5, * rot_thermal6, * rot_thermal7, * rot_thermal8;
};

Soyuz7k_T_PAO::Soyuz7k_T_PAO (OBJHANDLE hVessel, int flightmodel)
: VESSEL3 (hVessel, flightmodel)
{
	DefineAnimations();
}

Soyuz7k_T_PAO::~Soyuz7k_T_PAO ()
{
	delete rot_thermal1, rot_thermal2, rot_thermal3, rot_thermal4, rot_thermal5, rot_thermal6, rot_thermal7, rot_thermal8;
}

static UINT GRP_THERMAL[4] = {0, 1, 2, 3};
static UINT GRP_THERMAL2[2] = {0, 1};
static UINT GRP_THERMAL4[3] = {0, 1, 2};
static UINT GRP_THERMAL5 = 0;

static float THERMAL_RANGE = (float)(-40.0 * RAD);

static VECTOR3 THERMAL1_REF = { -1.1078 , 0.11265 , -0.77125 };
static VECTOR3 THERMAL2_REF = { -0.62145 , 0.9513 , -0.7712 };
static VECTOR3 THERMAL3_REF = { 0.6224 , 0.95265 , -0.771 };
static VECTOR3 THERMAL4_REF = { 1.10745 , 0.1119 , -0.7712 };
static VECTOR3 THERMAL5_REF = { -0.807612 , -0.789174 , -0.7714 };
static VECTOR3 THERMAL6_REF = { 0.947299 , -0.6058 , -0.7706 };
static VECTOR3 THERMAL7_REF = { -0.0003 , -1.1103 , -0.7708 };
static VECTOR3 THERMAL8_REF = { 0.52535 , -1.00685 , -0.77075 };

static VECTOR3 THERMAL1_AXIS = { 0.0958562, 0.995395, 0 };
static VECTOR3 THERMAL2_AXIS = { 0.82585, 0.563889, 0 };
static VECTOR3 THERMAL3_AXIS = { 0.825866, -0.563867, 0 };
static VECTOR3 THERMAL4_AXIS = { 0.0958562, -0.995395, 0 };
static VECTOR3 THERMAL5_AXIS = { -0.627761, 0.778406, 0 };
static VECTOR3 THERMAL6_AXIS = { -0.464561, -0.885541, 0 };
static VECTOR3 THERMAL7_AXIS = { -1, 0, 0 };
static VECTOR3 THERMAL8_AXIS = { -0.881425, -0.472323, 0 };

void Soyuz7k_T_PAO::DefineAnimations() {

	rot_thermal1 = new MGROUP_ROTATE(1, GRP_THERMAL, 4, THERMAL1_REF, THERMAL1_AXIS, THERMAL_RANGE);
	rot_thermal2 = new MGROUP_ROTATE(2, GRP_THERMAL2, 2, THERMAL2_REF, THERMAL2_AXIS, THERMAL_RANGE);
	rot_thermal3 = new MGROUP_ROTATE(3, GRP_THERMAL2, 2, THERMAL3_REF, THERMAL3_AXIS, THERMAL_RANGE);
	rot_thermal4 = new MGROUP_ROTATE(4, GRP_THERMAL4, 3, THERMAL4_REF, THERMAL4_AXIS, THERMAL_RANGE);
	rot_thermal5 = new MGROUP_ROTATE(5, &GRP_THERMAL5, 1, THERMAL5_REF, THERMAL5_AXIS, THERMAL_RANGE);
	rot_thermal6 = new MGROUP_ROTATE(6, &GRP_THERMAL5, 1, THERMAL6_REF, THERMAL6_AXIS, THERMAL_RANGE);
	rot_thermal7 = new MGROUP_ROTATE(7, &GRP_THERMAL5, 1, THERMAL7_REF, THERMAL7_AXIS, THERMAL_RANGE);
	rot_thermal8 = new MGROUP_ROTATE(8, &GRP_THERMAL5, 1, THERMAL8_REF, THERMAL8_AXIS, THERMAL_RANGE);

	anim_therm_cov = CreateAnimation(0);
	AddAnimationComponent(anim_therm_cov, 0, 1, rot_thermal1);
	AddAnimationComponent(anim_therm_cov, 0, 1, rot_thermal2);
	AddAnimationComponent(anim_therm_cov, 0, 1, rot_thermal3);
	AddAnimationComponent(anim_therm_cov, 0, 1, rot_thermal4);
	AddAnimationComponent(anim_therm_cov, 0, 1, rot_thermal5);
	AddAnimationComponent(anim_therm_cov, 0, 1, rot_thermal6);
	AddAnimationComponent(anim_therm_cov, 0, 1, rot_thermal7);
	AddAnimationComponent(anim_therm_cov, 0, 1, rot_thermal8);
}

// ==============================================================
// Overloaded callback functions
// ==============================================================

// --------------------------------------------------------------
// Set the capabilities of the vessel class
// --------------------------------------------------------------
void Soyuz7k_T_PAO::clbkSetClassCaps (FILEHANDLE cfg)
{
	
	
	// physical vessel parameters
	SetSize (S7K_PAO_SIZE);
	SetEmptyMass (S7K_PAO_EMPTYMASS);
	SetPMI (S7K_PAO_PMI);
	SetCrossSections (S7K_PAO_CS);
	SetRotDrag (S7K_PAO_RD);
	SetTouchdownPoints (tdvtx, ntdvtx);

	// airfoil definitions
	CreateAirfoil3 (LIFT_VERTICAL,   S7K_PAO_COP, vlift, NULL, S7K_PAO_VLIFT_C, S7K_PAO_VLIFT_S, S7K_PAO_VLIFT_A);
	CreateAirfoil3 (LIFT_HORIZONTAL, S7K_PAO_COP, hlift, NULL, S7K_PAO_HLIFT_C, S7K_PAO_HLIFT_S, S7K_PAO_HLIFT_A);

	therm_cov_status = 1;
	therm_cov_proc = 0;
	
	// camera parameters
	SetCameraOffset (_V(0.0,0.0,0.0));

	// associate a mesh for the visual
	mesh[0] = AddMesh("Soyuz_7k\\T_PAO");
	mesh[1] = AddMesh("Soyuz_7k\\OK_thermal1");
	mesh[2] = AddMesh("Soyuz_7k\\OK_thermal2");
	mesh[3] = AddMesh("Soyuz_7k\\OK_thermal3");
	mesh[4] = AddMesh("Soyuz_7k\\OK_thermal4");
	mesh[5] = AddMesh("Soyuz_7k\\OK_thermal5");
	mesh[6] = AddMesh("Soyuz_7k\\OK_thermal6");
	mesh[7] = AddMesh("Soyuz_7k\\OK_thermal7");
	mesh[8] = AddMesh("Soyuz_7k\\OK_thermal8");
	
	for (int i = 0; i < 9; i++) {
		SetMeshVisibilityMode(mesh[i], MESHVIS_ALWAYS);
	}

	ShiftCG(_V(0, 0, -2.5));


}

void Soyuz7k_T_PAO::clbkPreStep(double SimT, double SimDT, double MJD) {

	// thermal covers animation
	if (therm_cov_status == 1)
	{
		double da = SimDT * THERM_SPEED;
		therm_cov_proc = therm_cov_proc + da;
		if (therm_cov_proc > 1) {
			therm_cov_proc = 1;
			therm_cov_status = 2;
		}
		SetAnimation(anim_therm_cov, therm_cov_proc);
	}
	else if (therm_cov_status == 2)
	{
		SetAnimation(anim_therm_cov, 1.0);
	}
	else if (therm_cov_status == 3)
	{
		double da = SimDT * THERM_SPEED;
		therm_cov_proc = therm_cov_proc - da;
		if (therm_cov_proc < 0) {
			therm_cov_proc = 0;
			therm_cov_status = 0;
		}
		SetAnimation(anim_therm_cov, therm_cov_proc);
	}
	else SetAnimation(anim_therm_cov, 0.0);

}


// ==============================================================
// Airfoil lift/drag functions
// ==============================================================

void Soyuz7k_T_PAO::vlift (VESSEL *v, double aoa, double M, double Re,
	void *context, double *cl, double *cm, double *cd)
{
	static const double clp[] = {  // lift coefficient from -pi to pi in 10deg steps
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
	};
	static const double aoa_step = 10.0*RAD;
	double a, fidx, saoa = sin(aoa);
	a = modf((aoa+PI)/aoa_step, &fidx);
	int idx = (int)(fidx+0.5);
	*cl = clp[idx]*(1.0-a) + clp[idx+1]*a;     // linear interpolation
	*cm = 0.0; //-0.03*sin(aoa-0.1);
	*cd = 0.6 + 0.4*saoa*saoa;                // profile drag
	*cd += oapiGetInducedDrag (*cl, 1.0, 0.1); // induced drag
	*cd += oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);  // wave drag
}

void Soyuz7k_T_PAO::hlift (VESSEL *v, double aoa, double M, double Re,
	void *context, double *cl, double *cm, double *cd)
{
	static const double clp[] = {  // lift coefficient from -pi to pi in 45deg steps
		0,0,0,0,0,0,0,0,0,0
	};
	static const double aoa_step = 45.0*RAD;
	double a, fidx, saoa = sin(aoa);;
	a = modf((aoa+PI)/aoa_step, &fidx);
	int idx = (int)(fidx+0.5);
	*cl = clp[idx]*(1.0-a) + clp[idx+1]*a;     // linear interpolation
	*cm = 0.0;
	*cd = 0.6 + 0.4 * saoa * saoa;
	*cd += oapiGetInducedDrag (*cl, 1.0, 0.1); // induced drag
	*cd += oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);  // wave drag
}


// --------------------------------------------------------------
// Vessel initialisation
// --------------------------------------------------------------
DLLCLBK VESSEL *ovcInit (OBJHANDLE hvessel, int flightmodel)
{
	return new Soyuz7k_T_PAO (hvessel, flightmodel);
}

// --------------------------------------------------------------
// Vessel cleanup
// --------------------------------------------------------------
DLLCLBK void ovcExit (VESSEL *vessel)
{
	if (vessel) delete (Soyuz7k_T_PAO*)vessel;
}