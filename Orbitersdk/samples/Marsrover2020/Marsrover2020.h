// ==============================================================
//                 ORBITER MODULE: EAGLEHANGAR2
//                  Part of the ORBITER SDK
//          Copyright (C) 2002-2004 Martin Schweiger
//                   All rights reserved
//
// EAGLEHANGAR2.cpp
// Control module for EAGLEHANGAR2 vessel class
//
// Notes:
// This is an example for a "minimal" vessel implementation which
// only overloads the clbkSetClassCaps method to define vessel
// capabilities and otherwise uses the default VESSEL class
// behaviour.
// ==============================================================

// ==============================================================
// Some vessel parameters
// ==============================================================

// Interface for derived vessel class: QJBStar
// ==========================================================
const double ARM_OPERATING_SPEED = 0.05;
const double ARM_OPERATING_SPEED2 = 0.05;
const double FUELMASS = 10;
const double ISP = 5e7;
const double MAXMAINTH = 600;
const double MAXRETROTH = 600;
const double RCSISP = 5e7;
const double MAXRCSTH = 50;
const double MASS = 900;

// kuddel: I would place these into the .cpp file
//         It doesn't matter here but in larger projects, a change
//         in a header file usually results in lots of (cpp-)modules
//         to be re-build although only one really needed to be.
//
const double MAXSPEED = 2.78;                  // m/s
const double ROVER_ACCELERATION_FWD_REV = 0.5; // m/s^2
const double ROVER_ACCELERATION_NEUTRAL = 0.1; // m/s^2

class MSL_ROVER : public VESSEL4
{
public:
	MSL_ROVER(OBJHANDLE hObj, int fmodel);

	friend BOOL CALLBACK RMS_DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	int clbkConsumeDirectKey(char* kstate);
	enum DOORStatus { DOOR_UP, DOOR_DOWN, DOOR_RAISING, DOOR_LOWERING } HGA_status, COPTER_status;

	void clbkPreStep(double simt, double simdt, double mjd);
	bool clbkDrawHUD(int mode, const HUDPAINTSPEC* hps, oapi::Sketchpad* skp);

	void clbkSetClassCaps(FILEHANDLE cfg);
	void clbkVisualCreated(VISHANDLE vis, int refcount);
	void DefineAnimations(void);
	void clbkVisualDestroyed(VISHANDLE vis, int refcount);
	void clbkPostStep(double simtt, double simdt, double mjd);
	void clbkLoadStateEx(FILEHANDLE scn, void* status);
	void clbkSaveState(FILEHANDLE scn);
	void SetAnimationArm(UINT anim, double state);
	int clbkConsumeBufferedKey(DWORD key, bool down, char* kstate);
	enum WHEELStatus { WHEEL_UP, WHEEL_DOWN, WHEEL_RAISING, WHEEL_LOWERING } WHEEL_status;
	THRUSTER_HANDLE th_main[1], th_retro[1], th_yaw_l[2], th_yaw_r[2];
	THRUSTER_HANDLE thg_main, thg_retro, thg_yaw_l, thg_yaw_r;
	//void UpdateMesh();
	enum TURN_check { TURN_UP, TURN_STOP, TURN_DOWN } TURN_check;
	enum { NEUTRAL, FORWARD, REVERSE } eAccDir;
	enum { STRAIGHT, LEFT, RIGHT } TURNDir;
	ATTACHMENTHANDLE  LR2;
	void UpdateMesh();
	void RevertWHEEL(void);
	void ROTATERIGHT();
	void ROTATELEFT();
	void RevertHGA(void);
	void RevertCOPTER(void);
	void COPTERCASE();
	void SPAWNCOPTER();
	void MoveAround();
	//camera
	VECTOR3  arm_cameratip3[3];

	VESSELSTATUS2 vs2;
	char name[255];

	double currentSpeed;
	int  TurnANGLE, TURN_STATE, copterdep, coptercover;
	double d_lat, d_lng;
	double Long, Lat, Heading;
	double AngSpeed;
	double d_hdg;
	double lng, lat, hdg;
	double roll_angle;
	double pitch_angle;
	double CurrentSterzo;
	MATRIX3 RotationMatrix(VECTOR3 angles, bool xyz);
	VECTOR3 Rear_Axle_Pos, Front_Axle_Pos;
	MGROUP_TRANSFORM* copter_anim[20];

	UINT anim_ARM1, anim_ARM2, anim_ARM3, anim_ARM4, anim_ARM5, anim_ARM6, anim_ARM6a, anim_ARM7, anim_ARM0, anim_HGA, anim_HGA1, anim_HGA2, anim_CAM, anim_CAM_Z, anim_CAM_Y, anim_CAM1;
	UINT anim_MIDDLESHAFT, anim_REARWHEELSHAFT, anim_MIDDLEWHEELLEFT, anim_MIDDLEWHEELSHAFTLEFT, anim_REARWHEELLEFT, anim_REARWHEELFRAME, anim_FRONTWHEELLEFT, anim_FRONTWHEELFRAME, anim_FRONTWHEELSHAFT, anim_FRONTWHEELFRAME2;

	UINT anim_laser, anim_FRONTWHEELRIGHTREV, anim_MIDDLESHAFTRIGHT, anim_REARWHEELSHAFTRIGHT, anim_MIDDLEWHEELRIGHT, anim_MIDDLEWHEELSHAFTRIGHT, anim_REARWHEELFRAMERIGHT, anim_REARWHEELRIGHT, anim_FRONTWHEELSHAFTRIGHT, anim_FRONTWHEELFRAMERIGHT, anim_FRONTWHEELFRAMERIGHT2, anim_FRONTWHEELRIGHT;
	MGROUP_TRANSFORM* rms_anim[8], *rms1_anim[8], *rms2_anim[11], *rms3_anim[11];
	UINT anim_copterdeploy;
	double LIFT_SPEED, ARM1_proc, ARM2_proc, ARM3_proc, ARM4_proc, ARM0_proc, HGA3_proc, HGA2_proc, CAMMAST_proc, CAM_Z_proc, CAM_Y_proc, middleshaftleft_proc, rearshaftleft_proc, FRONTshaftleft_proc, FRONTWHEELleft_proc, FRONTWHEELRIGHT_proc;
	bool center_arm, UNPACK, PACK, CAMUP, CAMDOWN;
	bool center_arm1;
	bool center_arm2;
	double i3, i2l, i2, var, BASE0_proc, rot1, pos, rot;
	double center_arm_t, center_arm_t1, center_arm_t2, center_arm_t3, rot2, wheelspin, i5, pos2, FRONTWHEELFRAMELEFT_proc, center_arm_t4, center_arm_t5, COPTERDEPLOY_proc;
	double db, simdt, apd, TURN_proc, HGA1_proc;

	int  DRIVEMODE, COPTERDEPLOY, FORWARDgear, REVERSEgear, neutralgear, cameragear, LASTGEAR;;
	double Max_Steering_Angle, Height_From_Ground, Passo;

	double rt, earth_circ, each_deg, grav_acc;
	UINT LASER, mesh_MSL, COPTER, COPTERcase;
	//double CalculateCameraRotationAngle(VECTOR3& dir, const VECTOR3& rot);
};
