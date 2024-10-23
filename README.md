![1666046986903](https://github.com/user-attachments/assets/ab4c6ad6-695e-43e8-becd-6a62ce478c6b)

# Soyuz 7K-T

This is a simulation of the second generation Soyuz spacecraft for Orbiter Space Flight Simulator 2016 (http://orbit.medphys.ucl.ac.uk/). First launched uncrewed as Kosmos 496 in 1972, and crewed as Soyuz 12 in 1973, the 7K-T was introduced in the wake of the Soyuz 11 incident, for increased safety and focusing on the space station ferry role. This is a very different ship to handle compared to the modern Soyuz (versions TM and newer), with mostly analog electronics and avionics, based on 1960's technology, and a much more limited orbital capability. The Salyut 4 space station is also included as the mission target of Soyuz.

Soyuz 7K-T is a fully custom Orbiter addon, meant to simulate the spacecraft at the system level, as realistically as my time and skill allow. Development is on-going, and this is a work in progress version. As such, systems are still missing and certain details will look unfinished, and bugs are to be expected. However, a full simulation of a flight to Salyut 4 and back is possible with, I hope, a decent level of immersion. 

Features (see manual for full details):

+ Descent Module Virtual Cockpit, with the main and lateral KSU control panels modelled, with partial functionality, and a very basic interior model;
+ By default, the VC is in Russian, but an English option is available;
+ Full flight sequence simulated: launch, rendezvous and docking, module separation, re-entry and landing under parachute;
+ "Igla" approach and docking autopilot, precursor to the more modern Kurs system, with interaction between Soyuz and Salyut;
+ Realistic SSVP docking assembly simulation, with probe, latches and hooks;
+ Orbital and Solar Orientation and Inertial Attitude Hold autopilots;
+ Ion, Infrared and Sun sensors for orientation;
+ Integrating Accelerometer for manual burns;
+ DPO and DO manoeuvring systems, running on independent propellant supplies;
+ Main and backup orbital corrective engines (SKDU);
+ Vzor optical periscope for manual orientation and approach viewing;
+ Automated SKDU burn and descent programs;
+ SUS descent control, with ballistic re-entry automatic guidance;
+ Automated drogue and main parachute sequence and landing;
+ A Flight Guide and other documentation are included to accompany the scenarios.

![imagem](https://github.com/user-attachments/assets/2bf63adb-b87d-4d3d-83f8-740a12cbd5fa)


# Installation

For launch scenarios only, BEFORE installing Soyuz7k_T:

1. In this order, Igel's Historical Spaceports 1.5, Soyuz 1.2 and O2016 improvements pack: https://www.orbithangar.com/showAddon.php?id=0bb66462-d9cc-4f68-95f3-482bc319a5c7

2. DO NOT install Soyuz 1.3, third-party payloads are not working, pending update by author.

Non-launch scenarios have no further dependencies.

Built for Orbiter 2016 exclusively, latest O2016 D3D9 build required for best visual and feature match. The built-in graphics engine is no longer supported. Vzor periscope will not work without D3D9.
https://www.orbiter-forum.com/resources/d3d9-for-orbiter-2016.5493/

Sounds require XRSound. OrbiterSound is not supported, as XRSound will be included with future Orbiter releases.

Interior panels in Russian by default. To use the English virtual cockpit textures, copy and replace the textures from "Textures\Sirius_7k\Eng" to the main "Textures\Sirius_7k" folder.

Please read the included Manual and Flight Guide for a summary and explanation of the currently implemented features and how to use them on a typical flight.

## Common Issues

- If on launching a scenario no spacecraft are visible, install the x86 Microsoft Visual C++ Redistributable packages for Visual Studio 2015, 2017, 2019, and 2022:
https://learn.microsoft.com/en-US/cpp/windows/latest-supported-vc-redist?view=msvc-170

- Make sure to enable local light sources in the Orbiter settings
  
# Credits

- castorp/OctoberSky for the original exterior Soyuz 7k and Salyut meshes and textures.
- Using edited igel meshes for the fairings, to make them blank and the grid fins black, and to account for the Vzor lateral offset on 7K. All credit goes to igel.
- BrianJ's Dragon 2 code for the realistic docking system (soft/hard dock).

All remaining code, interior Soyuz meshes, VC panel textures, by diogom. Source code included in the SDK folder.

# License

Code is open-source under MIT license. Meshes and textures: all rights reserved, not to be redistributed without permission from the authors (diogom and castorp).
