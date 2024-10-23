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
const double  S7K_HT_SIZE = 1.1;					// mean radius [m]
const VECTOR3 S7K_HT_CS = {0.1,0.1,12.5};			// x,y,z cross sections [m^2]
const VECTOR3 S7K_HT_PMI = {0.7,0.2,2.4};			// principal moments of inertia (mass-normalised) [m^2]
const VECTOR3 S7K_HT_RD = {0.025,0.025,0.02};		// {0.05,0.1,0.05};  // rotation drag coefficients
const double  S7K_HT_EMPTYMASS = 500.0;				// empty vessel mass [kg]
const VECTOR3 S7K_HT_COP = {0,0,0};					// centre of pressure for airfoils [m]
const double  S7K_HT_VLIFT_C = 2.0;					// chord length [m]
const double  S7K_HT_VLIFT_S = 2.0;					// wing area [m^2]
const double  S7K_HT_VLIFT_A = 2.5;					// wing aspect ratio
const double  S7K_HT_HLIFT_C = 2.0;					// chord length [m]
const double  S7K_HT_HLIFT_S = 1.5;					// wing area [m^2]
const double  S7K_HT_HLIFT_A = 2.0;					// wing aspect ratio


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


// ==============================================================
// Soyuz7k_T_heatshield class interface
// ==============================================================

class Soyuz7k_T_heatshield: public VESSEL3 {
public:
	Soyuz7k_T_heatshield (OBJHANDLE hVessel, int flightmodel);
	~Soyuz7k_T_heatshield ();
	void clbkSetClassCaps (FILEHANDLE cfg);

private:
	static void vlift (VESSEL *v, double aoa, double M, double Re,
		void *context, double *cl, double *cm, double *cd);
	static void hlift (VESSEL *v, double aoa, double M, double Re,
		void *context, double *cl, double *cm, double *cd);
	
};

Soyuz7k_T_heatshield::Soyuz7k_T_heatshield (OBJHANDLE hVessel, int flightmodel)
: VESSEL3 (hVessel, flightmodel)
{
}

Soyuz7k_T_heatshield::~Soyuz7k_T_heatshield ()
{
}



// ==============================================================
// Overloaded callback functions
// ==============================================================

// --------------------------------------------------------------
// Set the capabilities of the vessel class
// --------------------------------------------------------------
void Soyuz7k_T_heatshield::clbkSetClassCaps (FILEHANDLE cfg)
{
	
	
	// physical vessel parameters
	SetSize (S7K_HT_SIZE);
	SetEmptyMass (S7K_HT_EMPTYMASS);
	SetPMI (S7K_HT_PMI);
	SetCrossSections (S7K_HT_CS);
	SetRotDrag (S7K_HT_RD);
	SetTouchdownPoints (tdvtx, ntdvtx);

	// airfoil definitions
	CreateAirfoil3 (LIFT_VERTICAL,   S7K_HT_COP, vlift, NULL, S7K_HT_VLIFT_C, S7K_HT_VLIFT_S, S7K_HT_VLIFT_A);
	CreateAirfoil3 (LIFT_HORIZONTAL, S7K_HT_COP, hlift, NULL, S7K_HT_HLIFT_C, S7K_HT_HLIFT_S, S7K_HT_HLIFT_A);

	
	// camera parameters
	SetCameraOffset (_V(0.0,0.0,0.0));

	// associate a mesh for the visual
	UINT mesh;

	mesh = AddMesh("Soyuz_7k\\OK_heatshield");
	SetMeshVisibilityMode(mesh, MESHVIS_ALWAYS);

	//ShiftCG(_V(0, -1.1, 0));


}



// ==============================================================
// Airfoil lift/drag functions
// ==============================================================

void Soyuz7k_T_heatshield::vlift (VESSEL *v, double aoa, double M, double Re,
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

void Soyuz7k_T_heatshield::hlift (VESSEL *v, double aoa, double M, double Re,
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




// --------------------------------------------------------------
// Vessel initialisation
// --------------------------------------------------------------
DLLCLBK VESSEL *ovcInit (OBJHANDLE hvessel, int flightmodel)
{
	return new Soyuz7k_T_heatshield (hvessel, flightmodel);
}

// --------------------------------------------------------------
// Vessel cleanup
// --------------------------------------------------------------
DLLCLBK void ovcExit (VESSEL *vessel)
{
	if (vessel) delete (Soyuz7k_T_heatshield*)vessel;
}
