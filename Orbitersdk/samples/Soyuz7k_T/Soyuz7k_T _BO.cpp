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
const double  S7K_BO_SIZE			= 2.5;				// mean radius [m]
const VECTOR3 S7K_BO_CS			= {5.6,5.3,4.9};		// x,y,z cross sections [m^2]
const VECTOR3 S7K_BO_PMI			= {7.0,6.9,0.8};	// principal moments of inertia (mass-normalised) [m^2]
const VECTOR3 S7K_BO_RD			= {0.025,0.025,0.02};	// {0.05,0.1,0.05};  // rotation drag coefficients
const double  S7K_BO_EMPTYMASS		= 1350.0;           // empty vessel mass [kg]
const VECTOR3 S7K_BO_COP			= {0,0,0};			// centre of pressure for airfoils [m]
const double  S7K_BO_VLIFT_C		= 2.1;				// chord length [m]
const double  S7K_BO_VLIFT_S		= 2.0;				// wing area [m^2]
const double  S7K_BO_VLIFT_A		= 2.5;				// wing aspect ratio
const double  S7K_BO_HLIFT_C		= 2.1;				// chord length [m]
const double  S7K_BO_HLIFT_S		= 1.5;				// wing area [m^2]
const double  S7K_BO_HLIFT_A		= 2.0;				// wing aspect ratio



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
// Soyuz7k_T_BO class interface
// ==============================================================

class Soyuz7k_T_BO: public VESSEL3 {
public:
	Soyuz7k_T_BO (OBJHANDLE hVessel, int flightmodel);
	~Soyuz7k_T_BO ();
	void clbkSetClassCaps (FILEHANDLE cfg);

private:
	static void vlift (VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd);
	static void hlift (VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd);
	
};

Soyuz7k_T_BO::Soyuz7k_T_BO (OBJHANDLE hVessel, int flightmodel)
: VESSEL3 (hVessel, flightmodel)
{
}

Soyuz7k_T_BO::~Soyuz7k_T_BO ()
{
}



// ==============================================================
// Overloaded callback functions
// ==============================================================


// --------------------------------------------------------------
// Set the capabilities of the vessel class
// --------------------------------------------------------------
void Soyuz7k_T_BO::clbkSetClassCaps (FILEHANDLE cfg)
{
	// physical vessel parameters
	SetSize (S7K_BO_SIZE);
	SetEmptyMass (S7K_BO_EMPTYMASS);
	SetPMI (S7K_BO_PMI);
	SetCrossSections (S7K_BO_CS);
	SetRotDrag (S7K_BO_RD);
	SetTouchdownPoints (tdvtx, ntdvtx);

	// airfoil definitions
	CreateAirfoil3 (LIFT_VERTICAL,   S7K_BO_COP, vlift, NULL, S7K_BO_VLIFT_C, S7K_BO_VLIFT_S, S7K_BO_VLIFT_A);
	CreateAirfoil3 (LIFT_HORIZONTAL, S7K_BO_COP, hlift, NULL, S7K_BO_HLIFT_C, S7K_BO_HLIFT_S, S7K_BO_HLIFT_A);

	
	// camera parameters
	SetCameraOffset (_V(0.0,0.0,0.0));

	// associate a mesh for the visual
	UINT mesh;
	
	if (!_strnicmp(GetName(), "noCM_", 5)) mesh = AddMesh("Soyuz_7k\\T_BO_noCM");
	else mesh = AddMesh("Soyuz_7k\\T_BO");
	SetMeshVisibilityMode(mesh, MESHVIS_ALWAYS);

	ShiftCG(_V(0, 0, 2.5));

}



// ==============================================================
// Airfoil lift/drag functions
// ==============================================================

void Soyuz7k_T_BO::vlift (VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd)
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
	*cd = 0.5 + 0.01*saoa*saoa;                // profile drag
	*cd += oapiGetInducedDrag (*cl, 1.0, 0.1); // induced drag
	*cd += oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);  // wave drag
}

void Soyuz7k_T_BO::hlift (VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd)
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
	*cd = 0.5 + 0.01 * saoa * saoa;
	*cd += oapiGetInducedDrag (*cl, 1.0, 0.1); // induced drag
	*cd += oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);  // wave drag
}




// --------------------------------------------------------------
// Vessel initialisation
// --------------------------------------------------------------
DLLCLBK VESSEL *ovcInit (OBJHANDLE hvessel, int flightmodel)
{
	return new Soyuz7k_T_BO (hvessel, flightmodel);
}

// --------------------------------------------------------------
// Vessel cleanup
// --------------------------------------------------------------
DLLCLBK void ovcExit (VESSEL *vessel)
{
	if (vessel) delete (Soyuz7k_T_BO*)vessel;
}
