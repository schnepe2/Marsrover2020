// ==============================================================
//                 ORBITER MODULE: MSL_ROVER
//                  Part of the ORBITER SDK
//          Copyright (C) 2002-2004 Martin Schweiger
//                   All rights reserved
//
// MSL.cpp
// Control module for MSL vessel class
//

// ==============================================================
#define ORBITER_MODULE
#include "Orbitersdk.h"
#include "Marsrover2020.h"
#include "MSLmesh.h"

//FOR DIALGUE
//#include "ultramathnew.h"
#include "DlgCtrl.h"
#include "resource.h"
#include <stdio.h>
#include <fstream>
#define LOADBMP(id) (LoadBitmap (g_Param.hDLL, MAKEINTRESOURCE (id)))
BOOL CALLBACK MSL_ROVER_DlgProc(HWND, UINT, WPARAM, LPARAM);
BOOL CALLBACK RMS_DlgProc(HWND, UINT, WPARAM, LPARAM);

// ==============================================================
// Global (class-wide) parameters
// ==============================================================

typedef struct {
	HINSTANCE hDLL;
	SURFHANDLE tkbk_label;
	HFONT font[1];
} GDIParams;

GDIParams g_Param;
//
VISHANDLE MainExternalMeshVisual = 0;
static const int ntdvtx = 3;
static TOUCHDOWNVTX tdvtx[ntdvtx] = {
	{ _V(0, 0.00078, 1.5), 26150.8, 18409.6, 3.2, 0.8 },
	{ _V(-2, 0.00078, -2.6), 26150.8, 18409.6, 3.2, 0.4 },
	{ _V(2, 0.00078, -2.6), 26150.8, 18409.6, 3.2, 0.4 }//,
	//	{ _V(-2, -2.164, 1.5), 1e6, 1e5, 3.2, 0 },
	//	{ _V(2, -2.164, 1.5), 1e6, 1e5, 3.2, 0 },
	//	{ _V(-2, 1.6, -2.6), 1e6, 1e5, 3.2, 0 },
	//	{ _V(2, 1.6, -2.6), 1e6, 1e5, 3.2, 0 },
	//	{ _V(-2, 1.6, 1.5), 1e6, 1e5, 3.2, 0 },
	//	{ _V(2, 1.6, 1.5), 1e6, 1e5, 3.2, 0 }
};

// Constructor
MSL_ROVER::MSL_ROVER(OBJHANDLE hObj, int fmodel) : VESSEL4(hObj, fmodel)
{
	LR2 = NULL;
	LIFT_SPEED = .05;
	HGA_status = DOOR_UP;
	HGA1_proc = 0;
	HGA3_proc = 0;
	HGA2_proc = 0;
	FRONTWHEELleft_proc = 0;
	CAMMAST_proc = 1.0;
	middleshaftleft_proc = 0.0;
	rearshaftleft_proc = 0;
	FRONTshaftleft_proc = 0;
	FRONTWHEELRIGHT_proc = 0;
	FRONTWHEELFRAMELEFT_proc = 0;
	CAM_Y_proc = 0.4870;
	CAM_Z_proc = 0.3317;
	UNPACK = false;
	PACK = false;
	CAMUP = false;
	CAMDOWN = false;
	center_arm = false;
	i2 = 0;
	i3 = 0;
	i5 = 0;
	ARM0_proc = 0;
	ARM1_proc = 0.4631;
	ARM2_proc = .0970;
	ARM3_proc = 0.6846;
	ARM4_proc = 0;
	DRIVEMODE = 1;
	COPTERDEPLOY = 0;
	currentSpeed = 0;
	eAccDir = NEUTRAL;
	TURNDir = STRAIGHT;
	FORWARDgear = 0;
	REVERSEgear = 0;
	neutralgear = 0;
	LASTGEAR = 0;
	TurnANGLE = 0;
	TURN_proc = 0;
	coptercover = 0;
	copterdep = 0;
	COPTERDEPLOY_proc = 0;
	COPTER_status = DOOR_UP;
	Height_From_Ground = 0.00;//2.1666
	//Height_From_Ground = 3.25;//2.1666
	//Height_From_Ground = 0;//2.1666
	//Height_From_Ground = 2.1666;
	DefineAnimations();

	//camera
	arm_cameratip3[0] = _V(.512, 2.1347, .542);//-0.5, -2.1,-.15  x=y  y=z z=x
	arm_cameratip3[1] = (_V(.512, 2.1347, .542) + _V(0, 1, 0));//1.5, -2.1, -.15

	arm_cameratip3[2] = (_V(.512, 2.1347, .542) + _V(0.438900189937643, 0, 0.898535821919583));//1.5, -2.1, -.15
}

void MSL_ROVER::clbkSetClassCaps(FILEHANDLE cfg)
{
	SetSize(6.08);
	SetEmptyMass(MASS);
	SetCW(0.9, 0.9, 2, 1.4);
	SetWingAspect(0.1);
	SetWingEffectiveness(0.1);
	SetCrossSections(_V(232.84, 1220.32, 166.36));
	SetRotDrag(_V(0.1, 0.1, 0.1));
	if (GetFlightModel() >= 1) {
		SetPitchMomentScale(1e-4);
		SetBankMomentScale(1e-4);
	}
	SetPMI(_V(163.54, 208.04, 76.03));
	SetTrimScale(0.05);
	SetCameraOffset(_V(0, .7, 3.121));
	//	SetTouchdownPoints(_V(0, -2.116, 20), _V(-15, -2.116, -20), _V(15, -2.116, -20));;
	//	SetTouchdownPoints(tdvtx, ntdvtx);
	double ro = Passo;
	TOUCHDOWNVTX td[4];

	double x_target = -0.5;
	double stiffness = (-1) * (MASS * 9.80655) / (3 * x_target);
	double damping = 0.9 * (2 * sqrt(MASS * stiffness));
	for (int i = 0; i < 4; i++) {
		td[i].damping = damping;
		td[i].mu = 3;
		td[i].mu_lng = 3;
		td[i].stiffness = stiffness;
	}
	td[0].pos.x = cos(30 * RAD) * ro;
	td[0].pos.y = -Height_From_Ground;
	td[0].pos.z = -sin(30 * RAD) * ro;
	td[1].pos.x = 0;
	td[1].pos.y = -Height_From_Ground;
	td[1].pos.z = 1 * ro;
	td[2].pos.x = -cos(30 * RAD) * ro;
	td[2].pos.y = -Height_From_Ground;
	td[2].pos.z = -sin(30 * RAD) * ro;
	td[3].pos.x = 0;
	td[3].pos.y = 15 * ro;
	td[3].pos.z = 0;

	SetTouchdownPoints(td, 4);

	LR2 = CreateAttachment(true, _V(0, 1.25, -.55), _V(0, 1, 0), _V(0, 0, 1), "SKYCRANE", false);

	EnableTransponder(true);
	mesh_MSL = AddMesh(oapiLoadMeshGlobal("Marsrover2020/Rover2020"));
	SetMeshVisibilityMode(mesh_MSL, MESHVIS_ALWAYS); //Main ship mesh

	COPTER = AddMesh(oapiLoadMeshGlobal("Marsrover2020/CopterStored"));// mesh0
	SetMeshVisibilityMode(COPTER, MESHVIS_ALWAYS);

	COPTERcase = AddMesh(oapiLoadMeshGlobal("Marsrover2020/CopterCase"));// mesh0
	SetMeshVisibilityMode(COPTERcase, MESHVIS_ALWAYS);
	//new
	THRUSTER_HANDLE attitudeDummy[6]; // 6, as we have pitch, yaw, bank, in both directions
	PROPELLANT_HANDLE smallDummyPropellant = CreatePropellantResource(0.01); // small, non-zero propellant mass

	// the position and direction vectors don't matter, as we have no thrust anyway.
	attitudeDummy[0] = CreateThruster(_V(0, 0, 0), _V(0, 1, 0), 0, smallDummyPropellant, 0, 0);
	attitudeDummy[1] = CreateThruster(_V(0, 0, 0), _V(0, -1, 0), 0, smallDummyPropellant, 0, 0);
	attitudeDummy[2] = CreateThruster(_V(0, 0, 0), _V(1, 0, 0), 0, smallDummyPropellant, 0, 0);
	attitudeDummy[3] = CreateThruster(_V(0, 0, 0), _V(-1, 0, 0), 0, smallDummyPropellant, 0, 0);
	attitudeDummy[4] = CreateThruster(_V(0, 0, 0), _V(0, 1, 0), 0, smallDummyPropellant, 0, 0);
	attitudeDummy[5] = CreateThruster(_V(0, 0, 0), _V(0, -1, 0), 0, smallDummyPropellant, 0, 0);

	// assign to the default thruster groups. Note that you could also instead assign to the translational groups (THGROUP_ATT_LEFT, THGROUP_ATT_RIGHT, ...). You decide
	CreateThrusterGroup(&attitudeDummy[0], 1, THGROUP_ATT_PITCHUP);
	CreateThrusterGroup(&attitudeDummy[1], 1, THGROUP_ATT_PITCHDOWN);
	CreateThrusterGroup(&attitudeDummy[2], 1, THGROUP_ATT_YAWLEFT);
	CreateThrusterGroup(&attitudeDummy[3], 1, THGROUP_ATT_YAWRIGHT);
	CreateThrusterGroup(&attitudeDummy[4], 1, THGROUP_ATT_BANKLEFT);
	CreateThrusterGroup(&attitudeDummy[5], 1, THGROUP_ATT_BANKRIGHT);
}

void MSL_ROVER::clbkPreStep(double simt, double simdt, double mjd)
{
	//	rt = oapiGetSize(GetSurfaceRef());
	//earth_circ = rt * 2 * PI;
	//each_deg = earth_circ / 360;
	//grav_acc = GGRAV * oapiGetMass(GetSurfaceRef()) / (rt * rt);
	//memset(&vs2, 0, sizeof(vs2));
	//vs2.version = 2;
	//GetStatusEx(&vs2);
	//MoveAround();
	rt = oapiGetSize(GetSurfaceRef());
	earth_circ = rt * 2 * PI;
	each_deg = earth_circ / 360;
	grav_acc = GGRAV * oapiGetMass(GetSurfaceRef()) / (rt * rt);
	memset(&vs2, 0, sizeof(vs2));
	vs2.version = 2;
	GetStatusEx(&vs2);
	//vs2.vrot.x = Height_From_Ground;
//	CurrentSpeed = .5;

	MoveAround();
}

void MSL_ROVER::clbkPostStep(double simt, double simdt, double mjd)
{
	//camera
	VECTOR3 xp3 = arm_cameratip3[1] - arm_cameratip3[0];
	normalise(xp3);
	VECTOR3 xr3 = arm_cameratip3[2] - arm_cameratip3[0];
	normalise(xr3);
	//SetAttachmentParams(MASTCAM, arm_cameratip3[0], xp3, xr3);

	SetCameraDefaultDirection(xp3);
	SetCameraOffset(arm_cameratip3[0]);

	// kuddel: You should not call this every frame!
	//         Just when the state of COPTERDEPLOY changes.
	if (COPTERDEPLOY == 1) {
		SetMeshVisibilityMode(COPTER, MESHVIS_NEVER);    //COPTERDEPLOYED
	}
//new
	if (GetThrusterGroupLevel(THGROUP_ATT_PITCHUP) != 0.0) {
		eAccDir = FORWARD;
	} else if (GetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN) != 0.0) {
		eAccDir = REVERSE;
	}

	if (GetThrusterGroupLevel(THGROUP_ATT_YAWLEFT) != 0.0) {
		TURNDir = LEFT;
	} else if (GetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT) != 0.0) {
		TURNDir = RIGHT;
	} else {
		TURNDir = STRAIGHT;
	}

	if (GetThrusterGroupLevel(THGROUP_ATT_BANKLEFT) != 0.0) {
		ROTATELEFT();
	} else if (GetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT) != 0.0) {
		ROTATERIGHT();
	}
	//
	double direction = 0;

	switch (eAccDir) {
		case NEUTRAL:
			if (abs(currentSpeed) < 0.001) { // deadband
				direction = 0;
			} else {
				direction = (currentSpeed < 0) ? +1.0 : -1.0; // "accelerate" towards zero (decelerate ;) )
			}
			break;
		case FORWARD:
			direction = +1.0;
			break;
		case REVERSE:
			direction = -1.0;
			break;
	}
	// Calculate new speed
	currentSpeed += direction * simdt * ROVER_ACCELERATION_FWD_REV;
	// limit speed to +- max
	currentSpeed = max(-MAXSPEED, min(currentSpeed, MAXSPEED));

	// current speed to wheelspin (can become negative or greater than 1)
	wheelspin += simdt * currentSpeed / 3.5; // The "3.5" has unit [m] and is wheel-circumference dependent (and how much MoveAround() moves around ;) )

	// kuddel: 'wheelspin' can only be positive in the range [0-1]!
	//         As it's an animation-state!
	//
	while (wheelspin > 1.0) {
		wheelspin -= 1.0;    // flip over at op range
	}
	while (wheelspin < 0.0) {
		wheelspin += 1.0;    // flip over at bottom range
	}

	// DEBUG helper for external view
	//sprintf(oapiDebugString(), "dt:%04d[ms] a:%d, v:%.2f w:%.2f, D:%d", int(round(simdt*1e3)), eAccDir, currentSpeed, wheelspin, DRIVEMODE);
	i3 = GetControlSurfaceLevel(AIRCTRL_RUDDER) * .15;  //changes heading

	SetAnimation(anim_FRONTWHEELLEFT, wheelspin);
	SetAnimation(anim_MIDDLEWHEELLEFT, wheelspin);
	SetAnimation(anim_REARWHEELLEFT, wheelspin);
	SetAnimation(anim_FRONTWHEELRIGHT, wheelspin);
	SetAnimation(anim_MIDDLEWHEELRIGHT, wheelspin);
	SetAnimation(anim_REARWHEELRIGHT, wheelspin);
	//SetAnimation(anim_FRONTWHEELRIGHTREV, wheelspin);
	//		main_on = 1;
	//		retro_on = 0;

	// kuddel: What's DRIVEMODE 0 ?
	//         And what's DRIVEMODE 1?
	//         You shoud use an enum and name them propperly ;)

	//if (DRIVEMODE == 0) {
	//	i3 = GetControlSurfaceLevel(AIRCTRL_RUDDER) * .15;  //changes heading

	//	if ((FORWARDgear == 1) && (pos > 0)) {
	//		AddForce(_V(1800, 0, 0), _V(-1, 0, 0));
	//		AddForce(_V(1800, 0, 0), _V(1, 0, 0));
	//	}

	//	if ((FORWARDgear == 1) && (pos < 0)) {
	//		AddForce(_V(-1800, 0, 0), _V(1, 0, 0));
	//		AddForce(_V(-1800, 0, 0), _V(-1, 0, 0));
	//	}

	//	if ((REVERSEgear == 1) && (pos > 0)) {
	//		AddForce(_V(-1800, 0, 0), _V(1, 0, 0));
	//		AddForce(_V(-1800, 0, 0), _V(-1, 0, 0));
	//	}
	//	if ((REVERSEgear == 1) && (pos < 0)) {
	//		AddForce(_V(1800, 0, 0), _V(-1, 0, 0));
	//		AddForce(_V(-1800, 0, 0), _V(1, 0, 0));
	//	}
	//}

//	if (DRIVEMODE == 1)
	{
		if (TURN_proc > 1) {
			TURN_proc = (TURN_proc - 1);
		} else if (TURN_proc < 0) {
			TURN_proc = (TURN_proc + 1);
		}
		if (TURN_check == TURN_DOWN) {
			TURN_proc = (TURN_proc + db);
		} else if (TURN_check == TURN_UP) {
			TURN_proc = (TURN_proc - db);
		}
		//else if (TURN_check == TURN_STOP)TURN_proc = TURN_proc;
		double ax14 = TURN_proc - rot;
		if (ax14 > 1) {
			ax14 -= 1;
		} else if (ax14 < 0) {
			ax14 += 1;
		}
		double ax36 = TURN_proc + rot;
		if (ax36 > 1) {
			ax36 -= 1;
		} else if (ax36 < 0) {
			ax36 += 1;
		}

		SetAnimation(anim_FRONTWHEELFRAMERIGHT, ax14);
		//SetAnimation(anim_axle2, TURN_proc);
		SetAnimation(anim_REARWHEELFRAMERIGHT, ax36);
		SetAnimation(anim_FRONTWHEELFRAME, ax14);
		//SetAnimation(anim_axle5, TURN_proc);
		SetAnimation(anim_REARWHEELFRAME, ax36);
	}
	TurnANGLE = int(TURN_proc * 360);
	//stow arm
	if (center_arm) {
		double t0 = oapiGetSimTime();
		double dt = t0 - center_arm_t;       // time step
		double da = ARM_OPERATING_SPEED2 * dt;
		//ROTATE CAM HEAD

		//ROTATE CAM MAST
		if (da && ARM0_proc) {     // zero wrist pitch
			if (da >= ARM0_proc) { // finished
				ARM0_proc = 0.0, da -= ARM0_proc;
			} else {
				ARM0_proc -= da, da = 0;
			}
			SetAnimationArm(anim_ARM0, ARM0_proc);
		}
		if (da && ARM1_proc != 0.5) {             // zero wrist pitch
			if (da >= fabs(ARM1_proc - 0.4631)) { // finished
				ARM1_proc = 0.5, da -= fabs(ARM1_proc - 0.5);
			} else {
				ARM1_proc -= (ARM1_proc > 0.5 ? da : -da), da = 0;
			}
			SetAnimationArm(anim_ARM1, ARM1_proc);
		}
		if (da && ARM2_proc != 0.5) {          // zero wrist pitch
			if (da >= fabs(ARM2_proc - 0.5)) { // finished
				ARM2_proc = 0.5, da -= fabs(ARM2_proc - 0.5);
			} else {
				ARM2_proc -= (ARM2_proc > 0.5 ? da : -da), da = 0;
			}
			SetAnimationArm(anim_ARM2, ARM2_proc);
		}
		if (da && ARM3_proc != 0.5) {          // zero wrist pitch
			if (da >= fabs(ARM3_proc - 0.5)) { // finished
				ARM3_proc = 0.5, da -= fabs(ARM3_proc - 0.5);
			} else {
				ARM3_proc -= (ARM3_proc > 0.5 ? da : -da), da = 0;
			}
			SetAnimationArm(anim_ARM3, ARM3_proc);
		}
		if (da && ARM4_proc != 0.5) {          // zero wrist pitch
			if (da >= fabs(ARM4_proc - 0.5)) { // finished
				ARM4_proc = 0.5, da -= fabs(ARM4_proc - 0.5);
			} else {
				ARM4_proc -= (ARM4_proc > 0.5 ? da : -da), da = 0;
			}
			SetAnimationArm(anim_ARM4, ARM4_proc);
		}
		center_arm_t = t0;
		if (da) {
			center_arm = false; // finished stowing
		}
	}
	//DEPLOYCOPTER
	if (COPTER_status >= DOOR_RAISING) {
		double da = simdt * LIFT_SPEED;
		if (COPTER_status == DOOR_RAISING) {
			if (COPTERDEPLOY_proc > 0.0) {
				COPTERDEPLOY_proc = max(0.0, COPTERDEPLOY_proc - da);
			} else {
				COPTER_status = DOOR_UP;
			}
		} else {
			if (COPTERDEPLOY_proc < 1.0) {
				COPTERDEPLOY_proc = min(1.0, COPTERDEPLOY_proc + da);
			} else {
				COPTER_status = DOOR_DOWN;
			}
		}
		if (COPTERDEPLOY_proc >= 1.0) {
			SPAWNCOPTER();
		}
		SetAnimation(anim_copterdeploy, COPTERDEPLOY_proc);
	}
	//sprintf(oapiDebugString(), " anim %2.2f ", COPTERDEPLOY_proc);

	if (UNPACK) {
		double t1 = oapiGetSimTime();
		double dt1 = t1 - center_arm_t2;       // time step
		double dF = ARM_OPERATING_SPEED2 * dt1;

		//ROTATE CAM HEAD

		//sprintf(oapiDebugString(), "animdf %2.2f anim %2.2f ", dF,CAMMAST_proc);

		//ROTATE CAM MAST

		{
			if (dF && middleshaftleft_proc) {             // zero elbow pitch
				if (dF >= middleshaftleft_proc) {         // finished
					middleshaftleft_proc = 0.0, dF -= middleshaftleft_proc;
				} else {
					middleshaftleft_proc -= dF, dF = 0;
				}
				SetAnimationArm(anim_MIDDLESHAFT, middleshaftleft_proc);
				SetAnimationArm(anim_MIDDLESHAFTRIGHT, middleshaftleft_proc);
			}

			if (dF && rearshaftleft_proc) {             // zero elbow pitch
				if (dF >= rearshaftleft_proc) {         // finished
					rearshaftleft_proc = 0.0, dF -= rearshaftleft_proc;
				} else {
					rearshaftleft_proc -= dF, dF = 0;
				}
				SetAnimationArm(anim_REARWHEELSHAFT, rearshaftleft_proc);
				SetAnimationArm(anim_REARWHEELSHAFTRIGHT, rearshaftleft_proc);
			}

			if (dF && FRONTshaftleft_proc) {             // zero elbow pitch
				if (dF >= FRONTshaftleft_proc) {         // finished
					FRONTshaftleft_proc = 0.0, dF -= FRONTshaftleft_proc;
				} else {
					FRONTshaftleft_proc -= dF, dF = 0;
				}
				SetAnimationArm(anim_FRONTWHEELSHAFTRIGHT, FRONTshaftleft_proc);
				SetAnimationArm(anim_FRONTWHEELSHAFT, FRONTshaftleft_proc);
			}

			if (dF && (FRONTWHEELRIGHT_proc)) {
				// zero wrist pitch
				if (dF >= FRONTWHEELRIGHT_proc) { // finished
					FRONTWHEELRIGHT_proc = 0.0, dF -= FRONTWHEELRIGHT_proc;
				} else {
					FRONTWHEELRIGHT_proc -= dF, dF = 0;
				}
				SetAnimationArm(anim_FRONTWHEELFRAMERIGHT, FRONTWHEELRIGHT_proc);
				//SetAnimationArm(anim_FRONTWHEELFRAME, FRONTWHEELRIGHT_proc);
			}
			if (dF && (FRONTWHEELFRAMELEFT_proc)) {
				// zero wrist pitch
				if (dF >= FRONTWHEELFRAMELEFT_proc) { // finished
					FRONTWHEELFRAMELEFT_proc = 0.0, dF -= FRONTWHEELFRAMELEFT_proc;
				} else {
					FRONTWHEELFRAMELEFT_proc -= dF, dF = 0;
				}
				//SetAnimationArm(anim_FRONTWHEELFRAMERIGHT, FRONTWHEELRIGHT_proc);
				SetAnimationArm(anim_FRONTWHEELFRAME, FRONTWHEELFRAMELEFT_proc);
			}
			center_arm_t2 = t1;
			if (dF) {
				UNPACK = false; // finished stowing
			}
		}
	}
	//pack wheels
	if (PACK) {
		double t2 = oapiGetSimTime();
		double dt2 = t2 - center_arm_t3;       // time step
		double dF1 = ARM_OPERATING_SPEED2 * dt2;
		//double dG1 = simdt * .01;
		//7
		//ROTATE CAM HEAD

		//sprintf(oapiDebugString(), "animdf %2.2f anim %2.2f ", dF,CAMMAST_proc);

		//ROTATE CAM MAST

		{
			//	sprintf(oapiDebugString(), "animdt1 %2.2f animdf %2.2f anim %2.2f ", dt1, dF, rearshaftleft_proc);

			if (dF1 && (middleshaftleft_proc != 1.0)) {             // zero elbow pitch
				if (dF1 >= fabs(middleshaftleft_proc - 1.0)) {       // finished
					middleshaftleft_proc = 1.0, dF1 -= fabs(middleshaftleft_proc - 1.0);
				} else {
					middleshaftleft_proc -= (middleshaftleft_proc > 1.0 ? dF1 : -dF1), dF1 = 0;
				}

				SetAnimationArm(anim_MIDDLESHAFT, middleshaftleft_proc);
				SetAnimationArm(anim_MIDDLESHAFTRIGHT, middleshaftleft_proc);
			}

			if (dF1 && (rearshaftleft_proc != 0.25)) {             // zero elbow pitch
				if (dF1 >= fabs(rearshaftleft_proc - 0.25)) {       // finished
					rearshaftleft_proc = 0.25, dF1 -= fabs(rearshaftleft_proc - .25);
				} else {
					rearshaftleft_proc -= (rearshaftleft_proc > .25 ? dF1 : -dF1), dF1 = 0;
				}
				SetAnimationArm(anim_REARWHEELSHAFT, rearshaftleft_proc);
				SetAnimationArm(anim_REARWHEELSHAFTRIGHT, rearshaftleft_proc);
			}

			if (dF1 && (FRONTshaftleft_proc != .4)) {             // zero elbow pitch
				if (dF1 >= fabs(FRONTshaftleft_proc - .4)) {       // finished
					FRONTshaftleft_proc = .4, dF1 -= fabs(FRONTshaftleft_proc - .4);
				} else {
					FRONTshaftleft_proc -= (FRONTshaftleft_proc > .4 ? dF1 : -dF1), dF1 = 0;
				}
				SetAnimationArm(anim_FRONTWHEELSHAFTRIGHT, FRONTshaftleft_proc);
				SetAnimationArm(anim_FRONTWHEELSHAFT, FRONTshaftleft_proc);
			}

			if (dF1 && (FRONTWHEELRIGHT_proc != 0.6)) {
				// zero wrist pitch
				if (dF1 >= fabs(FRONTWHEELRIGHT_proc - 0.6)) { // finished
					FRONTWHEELRIGHT_proc = 0.6, dF1 -= fabs(FRONTWHEELRIGHT_proc - 0.6);
				} else {
					FRONTWHEELRIGHT_proc -= (FRONTWHEELRIGHT_proc > 0.6 ? dF1 : -dF1), dF1 = 0;
				}
				SetAnimationArm(anim_FRONTWHEELFRAMERIGHT, FRONTWHEELRIGHT_proc);
			}
			if (dF1 && (FRONTWHEELFRAMELEFT_proc != 0.3)) {
				// zero wrist pitch
				if (dF1 >= fabs(FRONTWHEELFRAMELEFT_proc - 0.3)) { // finished
					FRONTWHEELFRAMELEFT_proc = 0.3, dF1 -= fabs(FRONTWHEELFRAMELEFT_proc - 0.3);
				} else {
					FRONTWHEELFRAMELEFT_proc -= (FRONTWHEELFRAMELEFT_proc > 0.3 ? dF1 : -dF1), dF1 = 0;
				}
				SetAnimationArm(anim_FRONTWHEELFRAME, FRONTWHEELFRAMELEFT_proc);
				//SetAnimationArm(anim_FRONTWHEELFRAME, FRONTWHEELRIGHT_proc);
			}
			center_arm_t3 = t2;
			if (dF1) {
				PACK = false; // finished stowing
			}
		}
	}
	//move camera
	if (CAMUP) {
		double t3 = oapiGetSimTime();
		double dt3 = t3 - center_arm_t4;       // time step
		double dF2 = ARM_OPERATING_SPEED2 * dt3;

		//ROTATE CAM MAST

		{
			//	sprintf(oapiDebugString(), "animdt1 %2.2f animdf %2.2f anim %2.2f ", dt1, dF, rearshaftleft_proc);

			if (dF2 && (CAMMAST_proc != 1.0)) {             // zero elbow pitch
				if (dF2 >= fabs(CAMMAST_proc - 1.0)) {       // finished
					CAMMAST_proc = 1.0, dF2 -= fabs(CAMMAST_proc - 1.0);
				} else {
					CAMMAST_proc -= (CAMMAST_proc > 1.0 ? dF2 : -dF2), dF2 = 0;
				}

				SetAnimationArm(anim_CAM, CAMMAST_proc);
			}

			if (dF2 && (CAM_Z_proc != 0.5)) {
				// zero wrist pitch
				if (dF2 >= fabs(CAM_Z_proc - .5)) { // finished
					CAM_Z_proc = .5, dF2 -= fabs(CAM_Z_proc - .5);
				} else {
					CAM_Z_proc -= (CAM_Z_proc > .5 ? dF2 : -dF2), dF2 = 0;
				}
				SetAnimationArm(anim_CAM_Z, CAM_Z_proc);
				//SetAnimationArm(anim_FRONTWHEELFRAME, FRONTWHEELRIGHT_proc);
			}

			if (dF2 && (CAM_Y_proc != 0.5)) {
				// zero wrist pitch
				if (dF2 >= fabs(CAM_Y_proc - .5)) { // finished
					CAM_Y_proc = .5, dF2 -= fabs(CAM_Z_proc - .5);
				} else {
					CAM_Y_proc -= (CAM_Y_proc > .5 ? dF2 : -dF2), dF2 = 0;
				}
				SetAnimationArm(anim_CAM_Y, CAM_Y_proc);
				//SetAnimationArm(anim_FRONTWHEELFRAME, FRONTWHEELRIGHT_proc);
			}

			center_arm_t4 = t3;
			if (dF2) {
				CAMUP = false; // finished stowing
			}
		}
	}

	if (CAMDOWN) {
		double t4 = oapiGetSimTime();
		double dt4 = t4 - center_arm_t5;       // time step
		double dF3 = ARM_OPERATING_SPEED2 * dt4;
		//double dG1 = simdt * .01;
		//7
		//ROTATE CAM HEAD

		//sprintf(oapiDebugString(), "animdf %2.2f anim %2.2f ", dF,CAMMAST_proc);

		//ROTATE CAM MAST

		{
			//	sprintf(oapiDebugString(), "animdt1 %2.2f animdf %2.2f anim %2.2f ", dt1, dF, rearshaftleft_proc);

			if (dF3 && (CAM_Z_proc != 0.5)) {
				// zero wrist pitch
				if (dF3 >= fabs(CAM_Z_proc - .5)) { // finished
					CAM_Z_proc = .5, dF3 -= fabs(CAM_Z_proc - .8344);
				} else {
					CAM_Z_proc -= (CAM_Z_proc > .5 ? dF3 : -dF3), dF3 = 0;
				}
				SetAnimationArm(anim_CAM_Z, CAM_Z_proc);
				//SetAnimationArm(anim_FRONTWHEELFRAME, FRONTWHEELRIGHT_proc);
			}

			if (dF3 && (CAM_Y_proc != 0.5)) {
				// zero wrist pitch
				if (dF3 >= fabs(CAM_Y_proc - .5)) { // finished
					CAM_Y_proc = .5, dF3 -= fabs(CAM_Y_proc - .5);
				} else {
					CAM_Y_proc -= (CAM_Y_proc > .5 ? dF3 : -dF3), dF3 = 0;
				}
				SetAnimationArm(anim_CAM_Y, CAM_Y_proc);
				//SetAnimationArm(anim_FRONTWHEELFRAME, FRONTWHEELRIGHT_proc);
			}

			if (dF3 && CAMMAST_proc) {//finished
				if (dF3 >= CAMMAST_proc) { // finished
					CAMMAST_proc = 0.0, dF3 -= CAMMAST_proc;
				} else {
					CAMMAST_proc -= dF3, dF3 = 0;
				}
				SetAnimationArm(anim_CAM, CAMMAST_proc);
			}

			center_arm_t5 = t4;
			if (dF3) {
				CAMDOWN = false; // finished stowing
			}
		}
	}
}

void MSL_ROVER::DefineAnimations(void)
{
	ANIMATIONCOMPONENT_HANDLE parent, parent1, parent2, parentFRTFRAME, parentFRT, parentcam, parentRLFTFRAME, parentRRGTFRAME, parent1R, parent2R, parentHEAD, COPTERPARENT;
	// , COPTERPARENTLEG, COPTERPARENT1;

	//CAMERA
	static UINT RMSMASTGrp[13] = { GRP_HEADBASE1, GRP_HEADBASE2, GRP_HEADBASE3, GRP_HEADBASE4, GRP_HEADBASE5, GRP_HEADBASE6, GRP_HEADBASE7, GRP_HEADBASE8, GRP_HEADBASE9, GRP_HEADBASE10, GRP_HEADBASE11, GRP_HEADBASE12, GRP_HEADBASE13 }; //MAST DOWN
	rms1_anim[3] = new MGROUP_ROTATE(0, RMSMASTGrp
	                                 , 13,
	                                 _V(.5560269, 1.243, .7159711), _V(-.5, 0, .866), (float)(-90 * RAD)); // -2 .. +145
	anim_CAM = CreateAnimation(1.0);
	parent = AddAnimationComponent(anim_CAM, 0, 1, rms1_anim[3]);

	static UINT CAMGrp1[7] = { GRP_HDPIVOT6, GRP_HDPIVOT5, GRP_HDPIVOT4, GRP_HDPIVOT3, GRP_HDPIVOT2, GRP_HDPIVOT1, GRP_HDPIVOT1A }; //CAMERA ROTATE Z AXIS MAST TOP AND REEL
	rms1_anim[4] = new MGROUP_ROTATE(0, CAMGrp1, 7, _V(.5579104, 1.83665, .7139302), _V(0, 1, 0), (float)(-360 * RAD)); // -2 .. +145
	anim_CAM_Z = CreateAnimation(0.5);
	parent = AddAnimationComponent(anim_CAM_Z, 0, 1, rms1_anim[4], parent);

	static UINT CAMGrp2[16] = { GRP_HEADTILT9, GRP_HEADTILT8, GRP_HEADTILT7, GRP_HEADTILT6, GRP_HEADTILT5, GRP_HEADTILT4, GRP_HEADTILT3, GRP_HEADTILT2, GRP_HEADTILT1, GRP_HEADTILT10, GRP_HEADTILT12, GRP_HEADTILT13, GRP_HEADTILT15, GRP_HEADTILT16, GRP_HEADTILT17, GRP_HEADTILT14 }; //CAMERA ROTATE Y AXIS
	rms1_anim[5] = new MGROUP_ROTATE(0, CAMGrp2, 16, _V(.575, 1.979, .6842173), _V(-.5, 0, .866), (float)(-170 * RAD)); // -2 .. +145
	anim_CAM_Y = CreateAnimation(0.5);
	parentcam = AddAnimationComponent(anim_CAM_Y, 0, 1, rms1_anim[5], parent);
	//camera
	rms1_anim[6] = new MGROUP_ROTATE(LOCALVERTEXLIST, MAKEGROUPARRAY(arm_cameratip3), 3, _V(.512, 2.1347, .542), _V(-1, 0, 0), (float)(894 * RAD)); // -447 .. +447

	static UINT RMSShoulderBASEGrp[4] = { GRP_PIVOT1, GRP_PIVOT1C, GRP_PIVOT1A, GRP_PIVOT1D }; //arm base
	rms_anim[0] = new MGROUP_ROTATE(0, RMSShoulderBASEGrp, 4, _V(-.452, 0.98813, 1.173), _V(0, 1, 0), (float)(-180 * RAD)); // -2 .. +145
	anim_ARM0 = CreateAnimation(0.0);
	parentHEAD = AddAnimationComponent(anim_ARM0, 0, 1, rms_anim[0]);

	static UINT RMSShoulderYawGrp[10] = { GRP_ARM1A, GRP_ARM1B, GRP_ARM1C, GRP_ARM1D, GRP_ARM1E, GRP_ARM1F, GRP_ARM1G, GRP_ARM1H, GRP_ARM1I, GRP_ARM1J }; //1st arm
	rms_anim[1] = new MGROUP_ROTATE(0, RMSShoulderYawGrp
	                                , 10,
	                                _V(-0.287, .822, 1.232387), _V(0, 0, -1), (float)(-360 * RAD)); // -2 .. +145
	anim_ARM1 = CreateAnimation(0.5);
	parent = AddAnimationComponent(anim_ARM1, 0, 1, rms_anim[1], parentHEAD);

	static UINT RMSShoulderPitchGrp[8] = { GRP_ARM2A, GRP_ARM2B, GRP_ARM2C, GRP_ARM2D, GRP_ARM2E, GRP_ARM2F, GRP_ARM2G, GRP_ARM2H };//2nd arm
	rms_anim[2] = new MGROUP_ROTATE(0, RMSShoulderPitchGrp, 8, _V(0.502, 1.052, 1.782743), _V(0, 0, -1), (float)(-360 * RAD)); // -180 .. +180
	anim_ARM2 = CreateAnimation(0.5);
	parent = AddAnimationComponent(anim_ARM2, 0, 1, rms_anim[2], parent);

	static UINT RMSWristPitchGrp[4] = { GRP_DRILLHEADPIVOT2, GRP_DRILLHEADPIVOT1, GRP_DRILLHEADPIVOT3, GRP_DRILLHEADPIVOT4 }; //3rd arm
	rms_anim[3] = new MGROUP_ROTATE(0, RMSWristPitchGrp, 4, _V(-0.253, 1.075, 1.074), _V(0, 0, -1), (float)(360 * RAD)); // -180 .. +180
	anim_ARM3 = CreateAnimation(0.5);
	parent = AddAnimationComponent(anim_ARM3, 0, 1, rms_anim[3], parent);

	static UINT RMSWristYaw1Grp[15] = { GRP_DRILLHEAD1, GRP_DRILLHEAD10, GRP_DRILLHEAD11, GRP_DRILLHEAD12, GRP_DRILLHEAD13, GRP_DRILLHEAD14, GRP_DRILLHEAD15, GRP_DRILLHEAD2, GRP_DRILLHEAD3, GRP_DRILLHEAD4, GRP_DRILLHEAD5, GRP_DRILLHEAD6, GRP_DRILLHEAD7, GRP_DRILLHEAD8, GRP_DRILLHEAD9 }; //sample head
	rms_anim[4] = new MGROUP_ROTATE(0, RMSWristYaw1Grp, 15,
	                                _V(-.428, .964, .996), _V(0, 1, 0), (float)(360 * RAD)); // -180 .. +180
	anim_ARM4 = CreateAnimation(0.5);
	parent = AddAnimationComponent(anim_ARM4, 0, 1, rms_anim[4], parent);

	//LEFT SIDE
	static UINT MIDDLESHAFTGrp1[6] = { GRP_leftmiddlewheel4A, GRP_leftmiddlewheel4, GRP_MIDDLELEFTLEG1, GRP_MIDDLELEFTLEG2, GRP_MIDDLELEFTLEG4, GRP_leftmiddlewheel6 }; //LEFTMIDDLESHAFT
	rms2_anim[0] = new MGROUP_ROTATE(0, MIDDLESHAFTGrp1, 6, _V(0, .8980631, .2060914), _V(1, 0, 0), (float)(70 * RAD)); // -2 .. +145
	anim_MIDDLESHAFT = CreateAnimation(0.0);
	parent1 = AddAnimationComponent(anim_MIDDLESHAFT, 0, 1, rms2_anim[0]);

	static UINT REARWHEELSHAFTGrp1[8] = { GRP_leftwheel1, GRP_leftwheel2, GRP_MIDDLELEFTAXLEA, GRP_REARLEFTLEG1, GRP_REARLEFTLEG2, GRP_REARLEFTLEG3, GRP_leftmiddlewheel3, GRP_leftmiddlewheel5 }; //LEFTREARSHAFT
	rms2_anim[1] = new MGROUP_ROTATE(0, REARWHEELSHAFTGrp1, 8, _V(0, .6651183, -.5350971), _V(1, 0, 0), (float)(-90 * RAD)); // -2 .. +145
	anim_REARWHEELSHAFT = CreateAnimation(0.0);
	parent2 = AddAnimationComponent(anim_REARWHEELSHAFT, 0, 1, rms2_anim[1], parent1);

	static UINT REARWHEELFRAMEGrp1[4] = { GRP_REARLEFTAXLEB, GRP_REARLEFTAXLEA, GRP_REARLEFTAXLEC, GRP_FRONTRIGHTAXLE5 }; //LEFTREARFRAME
	rms2_anim[2] = new MGROUP_ROTATE(0, REARWHEELFRAMEGrp1, 4, _V(-1.063, -1.116, -1.165), _V(0, 1, 0), (float)(360 * RAD)); // -2 .. +145
	anim_REARWHEELFRAME = CreateAnimation(0.0);
	parentRLFTFRAME = AddAnimationComponent(anim_REARWHEELFRAME, 0, 1, rms2_anim[2], parent2);

	static UINT REARWHEELGrp1[3] = { GRP_REARLEFTWHEELB, GRP_REARLEFTWHEELA, GRP_REARLEFTWHEELC }; //LEFTREARWHEEL
	rms2_anim[3] = new MGROUP_ROTATE(0, REARWHEELGrp1, 3, _V(-1, .265, -1.164), _V(1, 0, 0), (float)(360 * RAD)); // -2 .. +145
	anim_REARWHEELLEFT = CreateAnimation(0.5);
	AddAnimationComponent(anim_REARWHEELLEFT, 0, 1, rms2_anim[3], parentRLFTFRAME);

	static UINT MIDDLEWHEELGrp1[3] = { GRP_MIDDLELEFTWHEELB, GRP_MIDDLELEFTWHEELA, GRP_MIDDLELEFTWHEELC }; //LEFTMIDDLEWHEEL
	rms2_anim[4] = new MGROUP_ROTATE(0, MIDDLEWHEELGrp1, 3, _V(-1, .265, -.091), _V(1, 0, 0), (float)(360 * RAD)); // -2 .. +145
	anim_MIDDLEWHEELLEFT = CreateAnimation(0.5);
	AddAnimationComponent(anim_MIDDLEWHEELLEFT, 0, 1, rms2_anim[4], parent2);

	static UINT FRONTWHEELSHAFTGrp1[4] = { GRP_FRONTLEFTLEG1, GRP_FRONTLEFTLEG2, GRP_FRONTLEFTLEG3, GRP_FRONTLEFTLEG4 }; //LEFTFRONTSHAFT
	rms2_anim[5] = new MGROUP_ROTATE(0, FRONTWHEELSHAFTGrp1, 4, _V(0, .8980631, .2060914), _V(1, 0, 0), (float)(-110 * RAD)); // -2 .. +145
	anim_FRONTWHEELSHAFT = CreateAnimation(0.0);
	parentFRT = AddAnimationComponent(anim_FRONTWHEELSHAFT, 0, 1, rms2_anim[5]);

	static UINT FRONTWHEELFRAMEGrp1[3] = { GRP_FRONTLEFTAXLEA, GRP_FRONTLEFTAXLEB, GRP_FRONTLEFTAXLEC }; //LEFTFRONTWHEELFRAME
	rms2_anim[6] = new MGROUP_ROTATE(0, FRONTWHEELFRAMEGrp1, 3, _V(-1.063, -0.65085, 1.094), _V(0, 1, 0), (float)(360 * RAD)); // -2 .. +145
	anim_FRONTWHEELFRAME = CreateAnimation(0.0);
	parentFRTFRAME = AddAnimationComponent(anim_FRONTWHEELFRAME, 0, 1, rms2_anim[6], parentFRT);

	static UINT FRONTWHEELLEFTGrp1[3] = { GRP_FRONTLEFTWHEELB, GRP_FRONTLEFTWHEELA, GRP_FRONTLEFTWHEELC }; //LEFTFRONTWHEEL
	rms2_anim[7] = new MGROUP_ROTATE(0, FRONTWHEELLEFTGrp1, 3, _V(-1, .265, 1.094), _V(1, 0, 0), (float)(360 * RAD)); // -2 .. +145
	anim_FRONTWHEELLEFT = CreateAnimation(0.5);
	AddAnimationComponent(anim_FRONTWHEELLEFT, 0, 1, rms2_anim[7], parentFRTFRAME);

	//RIGHTWHEEL
	static UINT MIDDLESHAFTGrp2[6] = { GRP_rightmiddlewheel4A, GRP_rightmiddlewheel4, GRP_MIDDLERIGHTLEG3, GRP_MIDDLERIGHTLEG2, GRP_MIDDLERIGHTLEG4, GRP_rightmiddlewheel6 }; //CAMERA ROTATE Z AXIS MAST TOP AND REEL
	rms3_anim[0] = new MGROUP_ROTATE(0, MIDDLESHAFTGrp2, 6, _V(0, .8980631, .2060914), _V(1, 0, 0), (float)(70 * RAD)); // -2 .. +145
	anim_MIDDLESHAFTRIGHT = CreateAnimation(0.0);
	parent1R = AddAnimationComponent(anim_MIDDLESHAFTRIGHT, 0, 1, rms3_anim[0]);

	static UINT REARWHEELSHAFTGrp2[8] = { GRP_rightwheel1, GRP_rightwheel2, GRP_MIDDLERIGHTAXLEA, GRP_REARRIGHTLEG1, GRP_REARRIGHTLEG2, GRP_REARRIGHTLEG3, GRP_rightmiddlewheel3, GRP_rightmiddlewheel5 }; //CAMERA ROTATE Y AXIS
	rms3_anim[1] = new MGROUP_ROTATE(0, REARWHEELSHAFTGrp2, 8, _V(0, .6651183, -.5350971), _V(1, 0, 0), (float)(-90 * RAD));
	anim_REARWHEELSHAFTRIGHT = CreateAnimation(0.0);
	parent2R = AddAnimationComponent(anim_REARWHEELSHAFTRIGHT, 0, 1, rms3_anim[1], parent1R);

	static UINT REARWHEELFRAMEGrp2[4] = { GRP_REARRIGHTAXLEB, GRP_REARRIGHTAXLEA, GRP_REARRIGHTAXLEC, GRP_FRONTLEFTAXLE5 }; //CAMERA ROTATE Y AXIS
	rms3_anim[2] = new MGROUP_ROTATE(0, REARWHEELFRAMEGrp2, 4, _V(1.063, -1.116, -1.165), _V(0, 1, 0), (float)(360 * RAD)); // -2 .. +145
	anim_REARWHEELFRAMERIGHT = CreateAnimation(0.0);
	parentRRGTFRAME = AddAnimationComponent(anim_REARWHEELFRAMERIGHT, 0, 1, rms3_anim[2], parent2);

	static UINT REARWHEELGrp2[3] = { GRP_REARRIGHTWHEELB, GRP_REARRIGHTWHEELA, GRP_REARRIGHTWHEELC }; //CAMERA ROTATE Y AXIS
	rms3_anim[3] = new MGROUP_ROTATE(0, REARWHEELGrp2, 3, _V(1, .265, -1.164), _V(1, 0, 0), (float)(360 * RAD)); // -2 .. +145
	anim_REARWHEELRIGHT = CreateAnimation(0.5);
	AddAnimationComponent(anim_REARWHEELRIGHT, 0, 1, rms3_anim[3], parentRRGTFRAME);

	static UINT MIDDLEWHEELGrp2[3] = { GRP_MIDDLERIGHTWHEELB, GRP_MIDDLERIGHTWHEELA, GRP_MIDDLERIGHTWHEELC }; //CAMERA ROTATE Y AXIS
	rms3_anim[4] = new MGROUP_ROTATE(0, MIDDLEWHEELGrp2, 3, _V(1, .265, -.091), _V(1, 0, 0), (float)(360 * RAD)); // -2 .. +145
	anim_MIDDLEWHEELRIGHT = CreateAnimation(0.5);
	AddAnimationComponent(anim_MIDDLEWHEELRIGHT, 0, 1, rms3_anim[4], parent2R);

	static UINT FRONTWHEELSHAFTGrp2[4] = { GRP_FRONTRIGHTLEG2, GRP_FRONTRIGHTLEG3, GRP_FRONTRIGHTLEG4, GRP_FRONTRIGHTLEG11 }; //CAMERA ROTATE Y AXIS
	rms3_anim[5] = new MGROUP_ROTATE(0, FRONTWHEELSHAFTGrp2, 4, _V(0, .8980631, .2060914), _V(1, 0, 0), (float)(-110 * RAD)); // -2 .. +145
	anim_FRONTWHEELSHAFTRIGHT = CreateAnimation(0.0);
	parentFRT = AddAnimationComponent(anim_FRONTWHEELSHAFTRIGHT, 0, 1, rms3_anim[5]);

	static UINT FRONTWHEELFRAMEGrp3A[3] = { GRP_FRONTRIGHTAXLEA, GRP_FRONTRIGHTAXLEB, GRP_FRONTRIGHTAXLEC }; //CAMERA ROTATE Y AXIS
	rms3_anim[6] = new MGROUP_ROTATE(0, FRONTWHEELFRAMEGrp3A, 3, _V(1.063, -0.65085, 1.094), _V(0, 1, 0), (float)(360 * RAD)); // -2 .. +145
	anim_FRONTWHEELFRAMERIGHT = CreateAnimation(0.0);
	parentFRTFRAME = AddAnimationComponent(anim_FRONTWHEELFRAMERIGHT, 0, 1, rms3_anim[6], parentFRT);

	static UINT FRONTWHEELLEFTGrp2[3] = { GRP_FRONTRIGHTWHEELB, GRP_FRONTRIGHTWHEELA, GRP_FRONTRIGHTWHEELC }; //CAMERA ROTATE Y AXIS
	rms3_anim[7] = new MGROUP_ROTATE(0, FRONTWHEELLEFTGrp2, 3, _V(1, .265, 1.094), _V(1, 0, 0), (float)(360 * RAD)); // -2 .. +145
	anim_FRONTWHEELRIGHT = CreateAnimation(0.5);
	AddAnimationComponent(anim_FRONTWHEELRIGHT, 0, 1, rms3_anim[7], parentFRTFRAME);

	//HGA
	static UINT RMSShoulderBASEGrp1[6] = { GRP_HGAROT1, GRP_HGAROT2, GRP_HGAROT3, GRP_HGAROT4, GRP_HGAROT5, GRP_HGAROT6 }; //hga base
	rms1_anim[0] = new MGROUP_ROTATE(0, RMSShoulderBASEGrp1, 6,
	                                 _V(-.4740302, 1.1853, -.459788), _V(0, 1, 0), (float)(-360 * RAD)); // -180 .. +180
	anim_HGA = CreateAnimation(0.0);
	parent = AddAnimationComponent(anim_HGA, 0, 1, rms1_anim[0]);

	static UINT RMSShoulderYawGrp2[3] = { GRP_HGABASE, GRP_HGABASE1, GRP_HGABASE3 }; //hga tilt
	rms1_anim[1] = new MGROUP_ROTATE(0, RMSShoulderYawGrp2
	                                 , 3,
	                                 _V(-.5245481, 1.382571, -0.560585), _V(.4226, 0, .906), (float)(-360 * RAD)); // -90 .. +90
	anim_HGA2 = CreateAnimation(0.0);
	parent = AddAnimationComponent(anim_HGA2, 0, 1, rms1_anim[1], parent);

	static UINT RMSShoulderYawGrp1[3] = { GRP_HGAPIVOTSTOP, GRP_HGASTOP3, GRP_HGASTOP2 }; //hga rotate
	rms1_anim[2] = new MGROUP_ROTATE(0, RMSShoulderYawGrp1, 3, _V(.4448647, 1.1326, -.3959622), _V(1, 0, 0), (float)(-40 * RAD)); // -180 .. +180
	anim_HGA1 = CreateAnimation(0.0);
	AddAnimationComponent(anim_HGA1, 0, 1, rms1_anim[2], parent);

	anim_copterdeploy = CreateAnimation(0.0);
	static UINT COPTERARM1[1] = { GRP_copter3 };//COPTER RELEASE
	copter_anim[0] = new MGROUP_ROTATE(0, COPTERARM1, 1, _V(.421, .661, -.277), _V(0, 0, 1), (float)(90 * RAD));

	static UINT COPTERARM2[2] = { GRP_copter1, GRP_copter2, }; //COPTER HOLDER
	copter_anim[1] = new MGROUP_ROTATE(0, COPTERARM2, 2, _V(-.0187794, .7054401, -0.23251), _V(0, 0, 1), (float)(-90 * RAD));

	static UINT COPTERBODY[47] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 45, 46, 31, 43, 44 }; //COPTERMESH
	copter_anim[2] = new MGROUP_ROTATE(1, COPTERBODY, 47, _V(-.0187794, .7054401, -0.23251), _V(0, 0, 1), (float)(-90 * RAD));

	//static UINT COPTERBODYLEG1[3] = { 31,43,44 };//COPTERMESH
	//copter_anim[3] = new MGROUP_ROTATE(1, COPTERBODYLEG1, 3, _V(-1 , 0.55, - .165), _V(1, 0, 0), (float)(90 * RAD));

	AddAnimationComponent(anim_copterdeploy, 0, .5, copter_anim[0]);//COPTER RELEASE
	COPTERPARENT = AddAnimationComponent(anim_copterdeploy, .5, .9, copter_anim[1]);//COPTER HOLDER
	COPTERPARENT = AddAnimationComponent(anim_copterdeploy, .5, 1.0, copter_anim[2]);// , &COPTERPARENT);//COPTERBODY
	//COPTERPARENTLEG=AddAnimationComponent(anim_copterdeploy, .9, 1, copter_anim[3], &COPTERPARENT);//COPTERLEG
}

//,41,42,45,46
int MSL_ROVER::clbkConsumeBufferedKey(DWORD key, bool down, char* kstate)
{
	// only process keydown events
	if (!down) {
		return 0;
	}

	if (key == OAPI_KEY_SPACE) {
		// open RMS control dialog
		oapiOpenDialogEx(g_Param.hDLL, IDD_RMS, RMS_DlgProc, DLG_CAPTIONCLOSE, this);
		return 1;
	}
	//if (key == OAPI_KEY_G)
	//{
	//	if (DRIVEMODE == 0) DRIVEMODE = 1;
	//	else DRIVEMODE = 0;
	//	return 1;
	//}

	if (key == OAPI_KEY_K) {
		PACK = !PACK;
		if (PACK) {
			center_arm_t3 = oapiGetSimTime();
		}
		return 1;
	}
	if (key == OAPI_KEY_J) {
		UNPACK = !UNPACK;
		if (UNPACK) {
			center_arm_t2 = oapiGetSimTime();
		}
		return 1;
	}
	if (key == OAPI_KEY_1) {
		SetMeshVisibilityMode(LASER, MESHVIS_ALWAYS);

		return 1;
	}
	if (key == OAPI_KEY_2) {
		SetMeshVisibilityMode(LASER, MESHVIS_NEVER);

		return 1;
	}
	if (key == OAPI_KEY_3) {
		COPTERCASE();

		return 1;
	}
	if (key == OAPI_KEY_4) {
		RevertCOPTER();

		return 1;
	}

	return 0;
}

int MSL_ROVER::clbkConsumeDirectKey(char* kstate)
{
	//new
	/*
	if (KEYDOWN(kstate, OAPI_KEY_SUBTRACT)) {
		eAccDir = REVERSE;
		return 0;
	}
	if (KEYDOWN(kstate, OAPI_KEY_ADD)) {
		eAccDir = FORWARD;
		return 0;
	}

	if (KEYDOWN(kstate, OAPI_KEY_NUMPAD4)) { // turns wheels left
		ROTATELEFT();
		RESETKEY(kstate, OAPI_KEY_NUMPAD4);
		return 0;
	}
	if (KEYDOWN(kstate, OAPI_KEY_NUMPAD6)) { // turns wheels left
		ROTATERIGHT();
		RESETKEY(kstate, OAPI_KEY_NUMPAD6);
		return 0;
	}

	else {
		eAccDir = NEUTRAL;
		return 0;
	}
	if (KEYDOWN(kstate, OAPI_KEY_NUMPAD1)) { // turns wheels left
		TURNDir = LEFT;
		RESETKEY(kstate, OAPI_KEY_NUMPAD1);
		return 0;
	}
	if (KEYDOWN(kstate, OAPI_KEY_NUMPAD3)) { // turns wheels left
		TURNDir = RIGHT;;
		RESETKEY(kstate, OAPI_KEY_NUMPAD3);
		return 0;
	}
	else {
		TURNDir = STRAIGHT;
		return 0;
	}
	*/
	int keyConsumed = 0;
	if (KEYDOWN(kstate, OAPI_KEY_ADD)) {
		eAccDir = FORWARD;
		keyConsumed = 1;
	} else if (KEYDOWN(kstate, OAPI_KEY_SUBTRACT)) { // note the "else", 'cause these two are exclusive
		eAccDir = REVERSE;
		keyConsumed = 1;
	} else {
		eAccDir = NEUTRAL;
	}
	//change heading
	if (KEYDOWN(kstate, OAPI_KEY_NUMPAD1)) { // turns wheels left
		TURNDir = LEFT;
		RESETKEY(kstate, OAPI_KEY_NUMPAD1);
		keyConsumed = 1;
	} else if (KEYDOWN(kstate, OAPI_KEY_NUMPAD3)) { // turns wheels left
		TURNDir = RIGHT;;
		RESETKEY(kstate, OAPI_KEY_NUMPAD3);
		keyConsumed = 1;
	} else {
		TURNDir = STRAIGHT;
	}

	//rotate 4 wheels
	if (KEYDOWN(kstate, OAPI_KEY_NUMPAD4)) {
		ROTATELEFT();
		RESETKEY(kstate, OAPI_KEY_NUMPAD4);
		keyConsumed = 1;
	}
	if (KEYDOWN(kstate, OAPI_KEY_NUMPAD6)) {
		ROTATERIGHT();
		RESETKEY(kstate, OAPI_KEY_NUMPAD6);
		keyConsumed = 1;
	}
	return keyConsumed; // one single return "path"
}

// ==============================================================
// Message callback function for RMS control dialog box
// ==============================================================

void MSL_ROVER::SetAnimationArm(UINT anim, double state)
{
	SetAnimation(anim, state);
}

DLLCLBK VESSEL* ovcInit(OBJHANDLE hvessel, int flightmodel)
{
	return new MSL_ROVER(hvessel, flightmodel);
}

DLLCLBK void ovcExit(VESSEL* vessel)
{
	if (vessel) {
		delete (MSL_ROVER*)vessel;
	}
}

DLLCLBK void InitModule(HINSTANCE hModule)
{
	g_Param.hDLL = hModule;
	oapiRegisterCustomControls(hModule);
	g_Param.tkbk_label = oapiCreateSurface(LOADBMP(IDB_TKBKLABEL));

	// allocate GDI resources
	g_Param.font[0] = CreateFont(-11, 0, 0, 0, 400, 0, 0, 0, 0, 0, 0, 0, 0, "Arial");
}

DLLCLBK void ExitModule(HINSTANCE hModule)
{
	oapiUnregisterCustomControls(hModule);
	oapiDestroySurface(g_Param.tkbk_label);

	// deallocate GDI resources
	DeleteObject(g_Param.font[0]);
}

// --------------------------------------------------------------
// Keyboard interface handler (buffered key events)
// --------------------------------------------------------------

// ====================================================================
// clbkVisualCreated used to display UMMU initialisation message
// because oapiDebugString() doesn't work in clbkSetClassCap
// ====================================================================
void MSL_ROVER::clbkVisualCreated(VISHANDLE vis, int refcount)
{
	MainExternalMeshVisual = GetMesh(vis, 0);
}
// ==============================================================
// Visual destroyed
// ==============================================================
void MSL_ROVER::clbkVisualDestroyed(VISHANDLE vis, int refcount)
{
	MainExternalMeshVisual = 0;
}

#pragma warning (disable : 6031) // C6031 : "Return value ignored: 'scanf'."
void MSL_ROVER::clbkLoadStateEx(FILEHANDLE scn, void* status)
{
	char* line;
	while (oapiReadScenario_nextline(scn, line)) {
		if (!_strnicmp(line, "ARM0", 4)) {
			sscanf(line + 4, "%lf", &ARM0_proc);
		}

		if (!_strnicmp(line, "ARM1", 4)) {
			sscanf(line + 4, "%lf", &ARM1_proc);
		}

		if (!_strnicmp(line, "ARM2", 4)) {
			sscanf(line + 4, "%lf", &ARM2_proc);
		}
		if (!_strnicmp(line, "ARM3", 4)) {
			sscanf(line + 4, "%lf", &ARM3_proc);
		}
		if (!_strnicmp(line, "ARM4", 4)) {
			sscanf(line + 4, "%lf", &ARM4_proc);
		}
		if (!_strnicmp(line, "HGA3", 4)) {
			sscanf(line + 4, "%lf", &HGA3_proc);
		}
		if (!_strnicmp(line, "HGA1", 4)) {
			sscanf(line + 4, "%lf", &HGA1_proc);
		}
		if (!_strnicmp(line, "HGA2", 4)) {
			sscanf(line + 4, "%lf", &HGA2_proc);
		}
		if (!_strnicmp(line, "CAMY", 4)) {
			sscanf(line + 4, "%lf", &CAM_Y_proc);
		}
		if (!_strnicmp(line, "CAMZ", 4)) {
			sscanf(line + 4, "%lf", &CAM_Z_proc);
		}
		if (!_strnicmp(line, "CAMMAST", 7)) {
			sscanf(line + 7, "%lf", &CAMMAST_proc);
		}

		if (!_strnicmp(line, "MIDDLESHAFT", 11)) {
			sscanf(line + 11, "%lf", &middleshaftleft_proc);
		}
		if (!_strnicmp(line, "REARSHAFT", 9)) {
			sscanf(line + 9, "%lf", &rearshaftleft_proc);
		}
		if (!_strnicmp(line, "FRONTSHAFT", 10)) {
			sscanf(line + 10, "%lf", &FRONTshaftleft_proc);
		}
		if (!_strnicmp(line, "FRONTRIGHTWHEEL", 15)) {
			sscanf(line + 15, "%lf", &FRONTWHEELRIGHT_proc);
		}
		if (!_strnicmp(line, "FRONTLEFTWHEEL", 14)) {
			sscanf(line + 14, "%lf", &FRONTWHEELFRAMELEFT_proc);
		}
		if (!_strnicmp(line, "COPTERCOVER", 11)) {
			sscanf(line + 11, "%d", &coptercover);
		}
		if (!_strnicmp(line, "COPTERDEPLOY", 12)) {
			sscanf(line + 12, "%d %lf", &COPTER_status, &COPTERDEPLOY_proc);
		}
		UpdateMesh();
		SetAnimationArm(anim_copterdeploy, COPTERDEPLOY_proc);
		// ORBITER, unrecognised option - pass to Orbiter's generic parser
		ParseScenarioLineEx(line, status);
		if (coptercover == 1) {
			SetMeshVisibilityMode(COPTERcase, MESHVIS_NEVER);
		} else {
			SetMeshVisibilityMode(COPTERcase, MESHVIS_ALWAYS);
		}
	}
}
#pragma warning (default : 6031) // C6031 : "Return value ignored: 'scanf'."

void MSL_ROVER::UpdateMesh()
{
	// update animation states
	SetAnimationArm(anim_ARM0, ARM0_proc);
	SetAnimationArm(anim_ARM1, ARM1_proc);
	SetAnimationArm(anim_ARM2, ARM2_proc);
	SetAnimationArm(anim_ARM3, ARM3_proc);
	SetAnimationArm(anim_ARM4, ARM4_proc);
	SetAnimationArm(anim_HGA, HGA3_proc);
	SetAnimationArm(anim_HGA1, HGA1_proc);
	SetAnimationArm(anim_HGA2, HGA2_proc);
	SetAnimationArm(anim_CAM, CAMMAST_proc);
	SetAnimationArm(anim_CAM_Y, CAM_Y_proc);
	SetAnimationArm(anim_CAM_Z, CAM_Z_proc);
	SetAnimationArm(anim_MIDDLESHAFT, middleshaftleft_proc);
	SetAnimationArm(anim_FRONTWHEELSHAFT, FRONTshaftleft_proc);
	SetAnimationArm(anim_FRONTWHEELFRAMERIGHT, FRONTWHEELRIGHT_proc);
	SetAnimationArm(anim_REARWHEELSHAFT, rearshaftleft_proc);
	SetAnimationArm(anim_MIDDLESHAFTRIGHT, middleshaftleft_proc);
	SetAnimationArm(anim_FRONTWHEELSHAFTRIGHT, FRONTshaftleft_proc);
	SetAnimationArm(anim_FRONTWHEELFRAME, FRONTWHEELFRAMELEFT_proc);
	SetAnimationArm(anim_REARWHEELSHAFTRIGHT, rearshaftleft_proc);
}

void MSL_ROVER::clbkSaveState(FILEHANDLE scn)
{
	char cbuf[256];
	// ORBITER, default vessel parameters
	SaveDefaultState(scn);

	// default vessel parameters
	sprintf(cbuf, "%0.4f", ARM0_proc);
	oapiWriteScenario_string(scn, "ARM0", cbuf);

	sprintf(cbuf, "%0.4f", ARM1_proc);
	oapiWriteScenario_string(scn, "ARM1", cbuf);

	sprintf(cbuf, "%0.4f", ARM2_proc);
	oapiWriteScenario_string(scn, "ARM2", cbuf);
	sprintf(cbuf, "%0.4f", ARM3_proc);
	oapiWriteScenario_string(scn, "ARM3", cbuf);
	sprintf(cbuf, "%0.4f", ARM4_proc);
	oapiWriteScenario_string(scn, "ARM4", cbuf);

	sprintf(cbuf, "%0.4f", HGA3_proc);
	oapiWriteScenario_string(scn, "HGA3", cbuf);
	sprintf(cbuf, "%0.4f", HGA1_proc);
	oapiWriteScenario_string(scn, "HGA1", cbuf);
	sprintf(cbuf, "%0.4f", HGA2_proc);
	oapiWriteScenario_string(scn, "HGA2", cbuf);

	sprintf(cbuf, "%0.4f", CAM_Y_proc);
	oapiWriteScenario_string(scn, "CAMY", cbuf);

	sprintf(cbuf, "%0.4f", CAM_Z_proc);
	oapiWriteScenario_string(scn, "CAMZ", cbuf);

	sprintf(cbuf, "%0.4f", CAMMAST_proc);
	oapiWriteScenario_string(scn, "CAMMAST", cbuf);

	sprintf(cbuf, "%0.4f", middleshaftleft_proc);
	oapiWriteScenario_string(scn, "MIDDLESHAFT", cbuf);

	sprintf(cbuf, "%0.4f", rearshaftleft_proc);
	oapiWriteScenario_string(scn, "REARSHAFT", cbuf);

	sprintf(cbuf, "%0.4f", FRONTshaftleft_proc);
	oapiWriteScenario_string(scn, "FRONTSHAFT", cbuf);

	sprintf(cbuf, "%0.4f", FRONTWHEELRIGHT_proc);
	oapiWriteScenario_string(scn, "FRONTRIGHTWHEEL", cbuf);

	sprintf(cbuf, "%0.4f", FRONTWHEELFRAMELEFT_proc);
	oapiWriteScenario_string(scn, "FRONTLEFTWHEEL", cbuf);

	sprintf(cbuf, "%d", coptercover);
	oapiWriteScenario_string(scn, "COPTERCOVER", cbuf);

	sprintf(cbuf, "%d %0.4f", COPTER_status, COPTERDEPLOY_proc);
	oapiWriteScenario_string(scn, "COPTERDEPLOY", cbuf);
}

BOOL CALLBACK MSL_ROVER_DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	MSL_ROVER* sts = (uMsg == WM_INITDIALOG ? (MSL_ROVER*)lParam : (MSL_ROVER*)oapiGetDialogContext(hWnd));
	// pointer to vessel instance was passed as dialog context

	switch (uMsg) {
		case WM_COMMAND:
			switch (LOWORD(wParam)) {
				case IDCANCEL:
					oapiCloseDialog(hWnd);
					return TRUE;
				//case IDC_PLBAYOP:
				//	sts->plop->OpenDialog();
				//	break;
				case IDC_RMSOP:
					oapiOpenDialogEx(g_Param.hDLL, IDD_RMS, RMS_DlgProc, DLG_CAPTIONCLOSE, sts);
					break;
			}
			break;
	}
	return oapiDefDialogProc(hWnd, uMsg, wParam, lParam);
}

// ==============================================================
// Message callback function for RMS control dialog box
// ==============================================================

BOOL CALLBACK RMS_DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	MSL_ROVER* sts = (uMsg == WM_INITDIALOG ? (MSL_ROVER*)lParam : (MSL_ROVER*)oapiGetDialogContext(hWnd));
	// pointer to vessel instance was passed as dialog context

	//const double step = 0.05 * RAD;
	static double t0;
	double t1;
	HICON hIcon;
	HWND hDlg;

	switch (uMsg) {
		case WM_INITDIALOG:
			hIcon = LoadIcon(g_Param.hDLL, MAKEINTRESOURCE(IDI_UP));
			SendDlgItemMessage(hWnd, IDC_WRIST_PITCHUP, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_SHOULDER_PITCHUP, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_ELBOW_PITCHUP, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_SHOULDER_PITCHUP2, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_BASELEFT4, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);

			hIcon = LoadIcon(g_Param.hDLL, MAKEINTRESOURCE(IDI_DOWN));
			SendDlgItemMessage(hWnd, IDC_WRIST_PITCHDOWN, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_SHOULDER_PITCHDOWN, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_ELBOW_PITCHDOWN, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_SHOULDER_PITCHDOWN2, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_BASELEFT5, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_SHOULDER_PITCHDOWN3, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);

			hIcon = LoadIcon(g_Param.hDLL, MAKEINTRESOURCE(IDI_LEFT));
			SendDlgItemMessage(hWnd, IDC_WRIST_YAWRIGHT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			//	SendDlgItemMessage(hWnd, IDC_SHOULDER_YAWLEFT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_BASELEFT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_BASELEFT2, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_BASELEFT3, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);

			//	SendDlgItemMessage(hWnd, IDC_ELBOW_PITCHDOWN, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			hIcon = LoadIcon(g_Param.hDLL, MAKEINTRESOURCE(IDI_RIGHT));
			SendDlgItemMessage(hWnd, IDC_WRIST_YAWLEFT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			//	SendDlgItemMessage(hWnd, IDC_SHOULDER_YAWRIGHT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_BASERIGHT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_BASERIGHT2, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_BASERIGHT3, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);

			//	SendDlgItemMessage(hWnd, IDC_ELBOW_PITCHUP, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);

			hIcon = LoadIcon(g_Param.hDLL, MAKEINTRESOURCE(IDI_RRIGHT));
			SendDlgItemMessage(hWnd, IDC_WRIST_ROLLRIGHT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			hIcon = LoadIcon(g_Param.hDLL, MAKEINTRESOURCE(IDI_RLEFT));
			SendDlgItemMessage(hWnd, IDC_WRIST_ROLLLEFT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hIcon);
			SendDlgItemMessage(hWnd, IDC_SHOWGRAPPLE, BM_SETCHECK, oapiGetShowGrapplePoints() ? BST_CHECKED : BST_UNCHECKED, 0);

			//SetWindowText(GetDlgItem(hWnd, IDC_GRAPPLE), sts->SatGrappled() ? "Release" : "Grapple");
			//EnableWindow(GetDlgItem(hWnd, IDC_STOW), sts->SatGrappled() ? FALSE : TRUE);
			//SetWindowText(GetDlgItem(hWnd, IDC_PAYLOAD), sts->SatStowed() ? "Purge" : "Arrest");
			//EnableWindow(GetDlgItem(hWnd, IDC_PAYLOAD), sts->SatStowed() || sts->CanArrest() ? TRUE : FALSE);
			SetTimer(hWnd, 1, 50, NULL);
			t0 = oapiGetSimTime();
			return FALSE;
		case WM_DESTROY:
			KillTimer(hWnd, 1);
			return 0;
		case WM_TIMER:
			if (wParam == 1) {
				t1 = oapiGetSimTime();

				hDlg = oapiFindDialog(g_Param.hDLL, IDD_RMS);
				if (hDlg) {
					if (sts->HGA1_proc >= 1) {
						SetWindowText(GetDlgItem(hDlg, IDC_SHOULDER_PITCHUP3), "UNLOCK");
					}

					if (sts->HGA1_proc <= 0) {
						SetWindowText(GetDlgItem(hDlg, IDC_SHOULDER_PITCHUP3), "LOCK");
					}
					if ((sts->HGA1_proc > 0) && (sts->HGA1_proc < 1)) {
						SetWindowText(GetDlgItem(hDlg, IDC_SHOULDER_PITCHUP3), "////////");
					}
				}
				//ARM1_proc = 1.0 + fmod(ARM1_proc, 1.0);
				if (SendDlgItemMessage(hWnd, IDC_SHOULDER_PITCHDOWN, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->ARM1_proc = min(1.1, sts->ARM1_proc + (t1 - t0) * ARM_OPERATING_SPEED);
					sts->SetAnimationArm(sts->anim_ARM1, sts->ARM1_proc);
					if (sts->ARM1_proc > 1) {
						sts->ARM1_proc = 0;
					}
				} else if (SendDlgItemMessage(hWnd, IDC_SHOULDER_PITCHUP, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->ARM1_proc = max(-.1, sts->ARM1_proc - (t1 - t0) * ARM_OPERATING_SPEED);
					sts->SetAnimationArm(sts->anim_ARM1, sts->ARM1_proc);
					if (sts->ARM1_proc < 0) {
						sts->ARM1_proc = 1;
					}
				} else if (SendDlgItemMessage(hWnd, IDC_ELBOW_PITCHUP, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->ARM2_proc = max(0.0, sts->ARM2_proc - (t1 - t0) * ARM_OPERATING_SPEED);
					sts->SetAnimationArm(sts->anim_ARM2, sts->ARM2_proc);
				} else if (SendDlgItemMessage(hWnd, IDC_ELBOW_PITCHDOWN, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->ARM2_proc = min(1.0, sts->ARM2_proc + (t1 - t0) * ARM_OPERATING_SPEED);
					sts->SetAnimationArm(sts->anim_ARM2, sts->ARM2_proc);
				}

				else if (SendDlgItemMessage(hWnd, IDC_WRIST_PITCHUP, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->ARM3_proc = min(1.1, sts->ARM3_proc + (t1 - t0) * ARM_OPERATING_SPEED);
					if (sts->ARM3_proc > 1) {
						sts->ARM3_proc = 0;
					}
					sts->SetAnimationArm(sts->anim_ARM3, sts->ARM3_proc);
				} else if (SendDlgItemMessage(hWnd, IDC_WRIST_PITCHDOWN, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->ARM3_proc = max(-.1, sts->ARM3_proc - (t1 - t0) * ARM_OPERATING_SPEED);
					if (sts->ARM3_proc < 0) {
						sts->ARM3_proc = 1;
					}
					sts->SetAnimationArm(sts->anim_ARM3, sts->ARM3_proc);
				}

				else if (SendDlgItemMessage(hWnd, IDC_WRIST_ROLLLEFT, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->ARM4_proc = min(1.1, sts->ARM4_proc + (t1 - t0) * ARM_OPERATING_SPEED);
					if (sts->ARM4_proc > 1) {
						sts->ARM4_proc = 0;
					}
					sts->SetAnimationArm(sts->anim_ARM4, sts->ARM4_proc);
				} else if (SendDlgItemMessage(hWnd, IDC_WRIST_ROLLRIGHT, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->ARM4_proc = max(-.1, sts->ARM4_proc - (t1 - t0) * ARM_OPERATING_SPEED);
					if (sts->ARM4_proc < 0) {
						sts->ARM4_proc = 1;
					}
					sts->SetAnimationArm(sts->anim_ARM4, sts->ARM4_proc);
				} else if (SendDlgItemMessage(hWnd, IDC_BASERIGHT, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->ARM0_proc = min(1.0, sts->ARM0_proc + (t1 - t0) * ARM_OPERATING_SPEED);
					sts->SetAnimationArm(sts->anim_ARM0, sts->ARM0_proc);
				} else if (SendDlgItemMessage(hWnd, IDC_BASELEFT, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->ARM0_proc = max(0.0, sts->ARM0_proc - (t1 - t0) * ARM_OPERATING_SPEED);
					sts->SetAnimationArm(sts->anim_ARM0, sts->ARM0_proc);
				} else if (SendDlgItemMessage(hWnd, IDC_BASERIGHT2, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->HGA3_proc = min(1.1, sts->HGA3_proc + (t1 - t0) * ARM_OPERATING_SPEED);
					if (sts->HGA3_proc > 1) {
						sts->HGA3_proc = 0;
					}
					sts->SetAnimationArm(sts->anim_HGA, sts->HGA3_proc);
				} else if (SendDlgItemMessage(hWnd, IDC_BASELEFT2, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->HGA3_proc = max(-.1, sts->HGA3_proc - (t1 - t0) * ARM_OPERATING_SPEED);
					if (sts->HGA3_proc < 0) {
						sts->HGA3_proc = 1;
					}
					sts->SetAnimationArm(sts->anim_HGA, sts->HGA3_proc);
				} else if (SendDlgItemMessage(hWnd, IDC_SHOULDER_PITCHDOWN2, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->HGA2_proc = min(1.1, sts->HGA2_proc + (t1 - t0) * ARM_OPERATING_SPEED);
					sts->SetAnimationArm(sts->anim_HGA2, sts->HGA2_proc);
					if (sts->HGA2_proc > 1) {
						sts->HGA2_proc = 0;
					}
				} else if (SendDlgItemMessage(hWnd, IDC_SHOULDER_PITCHUP2, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->HGA2_proc = max(-.1, sts->HGA2_proc - (t1 - t0) * ARM_OPERATING_SPEED);
					sts->SetAnimationArm(sts->anim_HGA2, sts->HGA2_proc);
					if (sts->HGA2_proc < 0) {
						sts->HGA2_proc = 1;
					}
				} else if (SendDlgItemMessage(hWnd, IDC_SHOULDER_PITCHUP3, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					hDlg = oapiFindDialog(g_Param.hDLL, IDD_RMS);
					if (hDlg) {
						sts->RevertHGA();
					}
				} else if (SendDlgItemMessage(hWnd, IDC_BASERIGHT3, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->CAM_Z_proc = min(1.1, sts->CAM_Z_proc + (t1 - t0) * ARM_OPERATING_SPEED);
					if (sts->CAM_Z_proc > 1) {
						sts->CAM_Z_proc = 0;
					}
					sts->SetAnimationArm(sts->anim_CAM_Z, sts->CAM_Z_proc);
				} else if (SendDlgItemMessage(hWnd, IDC_BASELEFT3, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->CAM_Z_proc = max(-.1, sts->CAM_Z_proc - (t1 - t0) * ARM_OPERATING_SPEED);
					if (sts->CAM_Z_proc < 0) {
						sts->CAM_Z_proc = 1;
					}
					sts->SetAnimationArm(sts->anim_CAM_Z, sts->CAM_Z_proc);
				} else if (SendDlgItemMessage(hWnd, IDC_BASELEFT4, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->CAM_Y_proc = min(1.0, sts->CAM_Y_proc + (t1 - t0) * ARM_OPERATING_SPEED);

					sts->SetAnimationArm(sts->anim_CAM_Y, sts->CAM_Y_proc);
				} else if (SendDlgItemMessage(hWnd, IDC_BASELEFT5, BM_GETSTATE, 0, 0) & BST_PUSHED) {
					sts->CAM_Y_proc = max(0, sts->CAM_Y_proc - (t1 - t0) * ARM_OPERATING_SPEED);

					sts->SetAnimationArm(sts->anim_CAM_Y, sts->CAM_Y_proc);
				}

				t0 = t1;
			}
			if (!sts->center_arm) {
				EnableWindow(GetDlgItem(hWnd, IDC_GRAPPLE), TRUE);
			}
			break;
		case WM_COMMAND:
			switch (LOWORD(wParam)) {
				case IDCANCEL:
					oapiCloseDialog(hWnd);
					return TRUE;

				case IDC_GRAPPLE:
					//sts->ToggleGrapple();
					return 0;
				case IDC_PAYLOAD:
					//	sts->ToggleArrest();
					return 0;

				case IDC_STOW:
					sts->center_arm = !sts->center_arm;
					if (sts->center_arm) {
						sts->center_arm_t = oapiGetSimTime();
						//EnableWindow(GetDlgItem(hWnd, IDC_GRAPPLE), FALSE);
					}
					return 0;

				case IDC_STOW3:
					sts->CAMUP = !sts->CAMUP;
					if (sts->CAMUP) {
						sts->center_arm_t4 = oapiGetSimTime();
						//EnableWindow(GetDlgItem(hWnd, IDC_GRAPPLE), FALSE);
					}
					return 0;
				case IDC_STOW2:
					sts->CAMDOWN = !sts->CAMDOWN;
					if (sts->CAMDOWN) {
						sts->center_arm_t5 = oapiGetSimTime();
						//EnableWindow(GetDlgItem(hWnd, IDC_GRAPPLE), FALSE);
					}
					return 0;
				case IDC_SHOWGRAPPLE:
					//oapiSetShowGrapplePoints(SendDlgItemMessage(hWnd, IDC_SHOWGRAPPLE, BM_GETCHECK, 0, 0) == BST_CHECKED ? true : false);
					return 0;
			}
			break;
	}
	return oapiDefDialogProc(hWnd, uMsg, wParam, lParam);
}

void MSL_ROVER::RevertWHEEL(void)
{
	WHEEL_status = ((WHEEL_status == WHEEL_UP || WHEEL_status == WHEEL_RAISING) ?
	                WHEEL_LOWERING : WHEEL_RAISING);
}

void MSL_ROVER::RevertHGA(void)
{
	HGA_status = ((HGA_status == DOOR_UP || HGA_status == DOOR_RAISING) ?
	              DOOR_LOWERING : DOOR_RAISING);
}

void MSL_ROVER::RevertCOPTER(void)
{
	COPTER_status = ((COPTER_status == DOOR_UP || COPTER_status == DOOR_RAISING) ?
	                 DOOR_LOWERING : DOOR_RAISING);
}

void MSL_ROVER::ROTATELEFT(void)
{
	double simdt = oapiGetSimStep();
	double db = simdt * .05;
	TURN_proc -= db;
	if (TURN_proc < 0) {
		TURN_proc += 1;
	}
}

void MSL_ROVER::ROTATERIGHT(void)
{
	double simdt = oapiGetSimStep();
	double db = simdt * .05;
	(TURN_proc += db);
	if (TURN_proc > 1) {
		TURN_proc = (TURN_proc - 1);
	}
}

MATRIX3 MSL_ROVER::RotationMatrix(VECTOR3 angles, bool xyz = FALSE)
{
	MATRIX3 m;
	MATRIX3 RM_X, RM_Y, RM_Z;
	RM_X = _M(1, 0, 0, 0, cos(angles.x), -sin(angles.x), 0, sin(angles.x), cos(angles.x));
	RM_Y = _M(cos(angles.y), 0, sin(angles.y), 0, 1, 0, -sin(angles.y), 0, cos(angles.y));
	RM_Z = _M(cos(angles.z), -sin(angles.z), 0, sin(angles.z), cos(angles.z), 0, 0, 0, 1);
	if (!xyz) {
		m = mul(RM_Z, mul(RM_Y, RM_X));
	} else {
		m = mul(RM_X, mul(RM_Y, RM_Z));
	}
	return m;
}

void MSL_ROVER::MoveAround()
{
	memset(&vs2, 0, sizeof(vs2));
	vs2.version = 2;
	GetStatusEx(&vs2);
	double sinTurn = sin(2 * PI * TURN_proc);
	double cosTurn = cos(2 * PI * TURN_proc);

	/*if (i3 < 0) {//LEFT TURN
		d_hdg = d_hdg + .0000001;//rate of steering change of heading
		vs2.surf_hdg -= d_hdg;
		if (vs2.surf_hdg < 0) { vs2.surf_hdg += 2 * PI; }
		rot += 0.0005;//animation axle wheel turn rate
		if (rot > .125) rot = .125;
	}
	if (i3 > 0) {//RIGHT TURN
		rot -= 0.0005;//animation axle wheel turn rate
		if (rot < -.125) rot = -.125;
		d_hdg = d_hdg + .0000001;//rate of steering change of heading
		vs2.surf_hdg += d_hdg;
		if (vs2.surf_hdg > 2 * PI) { vs2.surf_hdg -= 2 * PI; }
	}
	if (i3 == 0) {// straight so straighten wheels
		if (abs(rot) <= 0.005) rot = 0;
		else if (rot < 0) rot += 0.0005;
		else if (rot > 0) rot -= 0.0005;
		d_hdg = 0;
	}

	sprintf(oapiDebugString(), "turn %d  SURFHDG %2.2f SURFHDGchange %2.2f  ", i3, vs2.surf_hdg, d_hdg);
	//sprintf(oapiDebugString(), "turn %2.2f cos %2.2f sin %2.2f heading %2.2f rudder %2.2f", TURN_proc, cosTurn, sinTurn, vs2.surf_hdg,i3);

	if (TURN_proc == 0) {//wheels are straight so move with heading
		d_lat = (currentSpeed * oapiGetSimStep() * cos(vs2.surf_hdg) / each_deg);
		d_lng = (currentSpeed * oapiGetSimStep() * sin(vs2.surf_hdg) / each_deg);
	}
	else {//wheels are not straight so move no matter the heading move to the left forward
		d_lat = (currentSpeed * oapiGetSimStep() * (cosTurn * 1) / each_deg);
		d_lng = (currentSpeed * oapiGetSimStep() * (sinTurn * 1) / each_deg);
	}
	vs2.surf_lat += d_lat * RAD;
	vs2.surf_lng += d_lng * RAD;

	*/
	if (eAccDir == FORWARD) { //MOVING FORWARD
		if (TURNDir == LEFT) {//LEFT TURN
			//sprintf(oapiDebugString(), "turn %2.2f cos %2.2f sin %2.2f", TURN_proc, cosTurn, sinTurn);
			d_hdg = d_hdg + .0000001;//rate of steering change of heading
			vs2.surf_hdg -= d_hdg;

			if (vs2.surf_hdg < 0) {
				vs2.surf_hdg += 2 * PI;
			}
			rot += 0.0005;//animation axle wheel turn rate
			if (rot > .125) {
				rot = .125;
			}
		}
		if (TURNDir == RIGHT) {//RIGHT TURN
			rot -= 0.0005;//animation axle wheel turn rate
			if (rot < -.125) {
				rot = -.125;
			}
			d_hdg = d_hdg + .0000001;//rate of steering change of heading
			vs2.surf_hdg += d_hdg;
			//sprintf(oapiDebugString(), "turn %d SURFHDG %2.2f", i3,  vs2.surf_hdg);
			if (vs2.surf_hdg > 2 * PI) {
				vs2.surf_hdg -= 2 * PI;
			}
		}
		if (TURNDir == NEUTRAL) {// straight so straighten wheels
			// sprintf (oapiDebugString (), "turn %2.2f cos %2.2f sin %2.2f", TURN_proc, cosTurn, sinTurn );
			if (abs(rot) <= 0.005) {
				rot = 0;
			} else if (rot < 0) {
				rot += 0.0005;
			} else if (rot > 0) {
				rot -= 0.0005;
			}
			d_hdg = 0;
		}
		if (TURN_proc == 0) {//wheels are straight so move with heading
			d_lat = (currentSpeed * oapiGetSimStep() * cos(vs2.surf_hdg) / each_deg);
			d_lng = (currentSpeed * oapiGetSimStep() * sin(vs2.surf_hdg) / each_deg);
		} else { //wheels are not straight so move no matter the heading move to the left forward
			d_lat = (currentSpeed * oapiGetSimStep() * (cosTurn * 1) / each_deg);
			d_lng = (currentSpeed * oapiGetSimStep() * (sinTurn * 1) / each_deg);
		}
		vs2.surf_lat += d_lat * RAD;
		vs2.surf_lng += d_lng * RAD;
	} else if (eAccDir == REVERSE) {
		if (i3 > 0) {
			d_hdg = d_hdg + .00001;//rate of steering change of heading
			vs2.surf_hdg -= d_hdg;

			if (vs2.surf_hdg < 0) {
				vs2.surf_hdg += 2 * PI;
			}
			rot += 0.005;//animation axle wheel turn rate
			if (rot > .125) {
				rot = .125;
			}
		}
		if (i3 < 0) {
			rot -= 0.005;//animation axle wheel turn rate
			if (rot < -.125) {
				rot = -.125;
			}

			d_hdg = d_hdg + .00001;//rate of steering change of heading
			vs2.surf_hdg += d_hdg;

			if (vs2.surf_hdg > 2 * PI) {
				vs2.surf_hdg -= 2 * PI;
			}
		}
		if (i3 == 0) {// straight so straighten wheels
			// sprintf (oapiDebugString (), "turn %2.2f cos %2.2f sin %2.2f", TURN_proc, cosTurn, sinTurn );
			if (abs(rot) <= 0.005) {
				rot = 0;
			} else if (rot < 0) {
				rot += 0.005;
			} else if (rot > 0) {
				rot -= 0.005;
			}
			d_hdg = 0;
		}
		if (TURN_proc == 0) {//wheels are straight so move with heading
			d_lat = (currentSpeed * oapiGetSimStep() * cos(vs2.surf_hdg) / each_deg);
			d_lng = (currentSpeed * oapiGetSimStep() * sin(vs2.surf_hdg) / each_deg);
		} else { //wheels are not straight so move no matter the heading move to the left forward
			d_lat = (currentSpeed * oapiGetSimStep() * (cosTurn * 1) / each_deg);
			d_lng = (currentSpeed * oapiGetSimStep() * (sinTurn * 1) / each_deg);
		}
		//sprintf(oapiDebugString(), "turn %2.2f cos %2.2f sin %2.2f lat %2.2f lng %2.2f", TURN_proc, cosTurn, sinTurn, d_lat, d_lng);

		vs2.surf_lat += d_lat * RAD;
		vs2.surf_lng += d_lng * RAD;
	} else if (eAccDir == NEUTRAL) {
		if (TURN_proc == 0) {//wheels are straight so move with heading
			d_lat = (currentSpeed * oapiGetSimStep() * cos(vs2.surf_hdg) / each_deg);
			d_lng = (currentSpeed * oapiGetSimStep() * sin(vs2.surf_hdg) / each_deg);
		} else { //wheels are not straight so move no matter the heading move to the left forward
			d_lat = (currentSpeed * oapiGetSimStep() * (cosTurn * 1) / each_deg);
			d_lng = (currentSpeed * oapiGetSimStep() * (sinTurn * 1) / each_deg);
		}
		vs2.surf_lat += d_lat * RAD;
		vs2.surf_lng += d_lng * RAD;
	}

	lng = vs2.surf_lng;
	lat = vs2.surf_lat;
	hdg = vs2.surf_hdg;

	Front_Axle_Pos = _V(0, 1, 2);

	VECTOR3 ant_dx_pos = Front_Axle_Pos;
	double ant_dx_dlat = ((ant_dx_pos.z * cos(hdg) - ant_dx_pos.x * sin(hdg)) / each_deg) * RAD;
	double ant_dx_dlng = ((ant_dx_pos.z * sin(hdg) + ant_dx_pos.x * cos(hdg)) / each_deg) * RAD;
	double elev_ant_dx = oapiSurfaceElevation(GetSurfaceRef(), lng + ant_dx_dlng, lat + ant_dx_dlat);

	VECTOR3 ant_sx_pos = _V(Front_Axle_Pos.x - 1, Front_Axle_Pos.y, Front_Axle_Pos.z);
	double ant_sx_dlat = ((ant_sx_pos.z * cos(hdg) - ant_sx_pos.x * sin(hdg)) / each_deg) * RAD;
	double ant_sx_dlng = ((ant_sx_pos.z * sin(hdg) + ant_sx_pos.x * cos(hdg)) / each_deg) * RAD;
	double elev_ant_sx = oapiSurfaceElevation(GetSurfaceRef(), lng + ant_sx_dlng, lat + ant_sx_dlat);
	//sprintf(oapiDebugString(), "roll %2.2f pitch %2.2f ", roll_angle, pitch_angle);
	Rear_Axle_Pos = _V(0, 1, -2);
	VECTOR3 post_pos = Rear_Axle_Pos;
	double post_dlat = ((+post_pos.z * cos(hdg)) / each_deg) * RAD;
	double post_dlng = ((+post_pos.z * sin(hdg)) / each_deg) * RAD;
	double elev_post = oapiSurfaceElevation(GetSurfaceRef(), lng + post_dlng, lat + post_dlat);

	{
		//roll_angle = -CurrentSterzo * 2 * Max_Steering_Angle;
		roll_angle = atan2(elev_ant_dx - elev_ant_sx, ant_dx_pos.x - ant_sx_pos.x);
		pitch_angle = atan2(-elev_post + ((elev_ant_dx + elev_ant_sx) * 0.5), ant_dx_pos.z - post_pos.z);
	}
	{
		MATRIX3 rot1 = RotationMatrix(_V(0 * RAD, (90 * RAD - lng), 0 * RAD), TRUE);
		MATRIX3 rot2 = RotationMatrix(_V(-lat + 0 * RAD, 0, 0 * RAD), TRUE);
		MATRIX3 rot3 = RotationMatrix(_V(0, 0, 180 * RAD + hdg), TRUE);

		//MATRIX3 rot4 = RotationMatrix(_V(90 * RAD, 0, 0), TRUE);
		MATRIX3 rot4 = RotationMatrix(_V(90 * RAD - pitch_angle, 0, roll_angle), TRUE);

		//		MATRIX3 rot_pitch = RotationMatrix(_V(pitch_angle, 0, 0));
		//	MATRIX3 rot_roll = RotationMatrix(_V(0, 0, roll_angle));

		MATRIX3 RotMatrix_Def = mul(rot1, mul(rot2, mul(rot3, rot4)));

		vs2.arot.x = atan2(RotMatrix_Def.m23, RotMatrix_Def.m33);
		vs2.arot.y = -asin(RotMatrix_Def.m13);
		vs2.arot.z = atan2(RotMatrix_Def.m12, RotMatrix_Def.m11);
		vs2.vrot.x = Height_From_Ground;
	}
	DefSetStateEx(&vs2);
	return;
}

bool MSL_ROVER::clbkDrawHUD(int mode, const HUDPAINTSPEC* hps, oapi::Sketchpad* skp)
{
	// draw the default HUD
	VESSEL3::clbkDrawHUD(mode, hps, skp);
	int s = hps->H;
	double d = (s * 0.00130208);
	int sw = ((hps->W));
	int lw = (int)(16 * sw / 1024);

	int roxl = 0;
	int royl = 0;
	//int roy2 = 0;

	double ds = s;
	double dsw = sw;
	double sc_ratio = ds / dsw;

	if (sc_ratio < 0.7284) {
		roxl = (lw * 10);
		royl = (int)(-88 * d);
		//roy2 = (int)(-70 * d);
	}
	//int wd = (int)(152 * d);
	//int wc = (int)(168 * d);
	//int w0 = (int)(184 * d);
	//int w1 = (int)(200 * d);
	//int w2 = (int)(216 * d);
	//int w3 = (int)(232 * d);
	//int w4 = (int)(248 * d);
	//int w5 = (int)(264 * d);
	//int w6 = (int)(280 * d);
	//int w7 = (int)(296 * d);
	//int w8 = (int)(312 * d);
	int w9 = (int)(328 * d);
	int w10 = (int)(344 * d);
	//int w11 = (int)(360 * d);

	{
		skp->SetTextColor(0x0066FF66);
		char abuf[256];
		sprintf(abuf, "Current Ground Speed %0.2f%s", abs(currentSpeed),
		        (eAccDir == NEUTRAL && currentSpeed == 0) ? "" : (
		            signbit(currentSpeed) ? " REV"
		            : " FWD")
		       );
		skp->Text((10 + roxl), (w9 + royl), abuf, strlen(abuf));
	}

	{
		skp->SetTextColor(0x0066FF66);
		char abuf[256];
		sprintf(abuf, "Turn Angle: %d", (TurnANGLE));
		skp->Text((10 + roxl), (w10 + royl), abuf, strlen(abuf));
	}
	return true;
}

void MSL_ROVER::COPTERCASE(void)
{
	if (coptercover == 0) {
		VESSELSTATUS2 vs;
		memset(&vs, 0, sizeof(vs));
		vs.version = 2;
		GetStatusEx(&vs);
		OBJHANDLE COPTER_COVER;
		vs.status = 1;

		COPTER_COVER = oapiCreateVesselEx("COPTER_COVER", "COPTERCOVER", &vs); // do we need apostrophes "" ?
		VESSEL2* COPTER_COVER_vessel = (VESSEL2*)oapiGetVesselInterface(COPTER_COVER); // vessel interface if needed, delete line if not

		SetMeshVisibilityMode(COPTERcase, MESHVIS_NEVER);
		coptercover = 1;
	}
}

void MSL_ROVER::SPAWNCOPTER(void)
{
	VESSELSTATUS2 vs;
	memset(&vs, 0, sizeof(vs));
	vs.version = 2;
	GetStatusEx(&vs);
	OBJHANDLE COPTER_VESSEL;
	vs.status = 1; // Landed

	COPTER_VESSEL = oapiCreateVesselEx("MARSCOPTER", "MarsCopter", &vs); // do we need apostrophes "" ?
	VESSEL2* COPTER_vessel = (VESSEL2*)oapiGetVesselInterface(COPTER_VESSEL); // vessel interface if needed, delete line if not
	SetMeshVisibilityMode(COPTER, MESHVIS_NEVER);//do not show stored copter

	RevertCOPTER();  // copter holding gear goes back

	copterdep = 1;//copter is deployed
}
