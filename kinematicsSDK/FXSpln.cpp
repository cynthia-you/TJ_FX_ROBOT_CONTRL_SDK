#include "FXSpln.h"
#include "AxisPln.h"

extern "C" FXSpln AxisPln_Create()
{
	return new CAxisPln();
}

extern "C" void AxisPln_OnMovL(FXSpln spln, long RobotSerial, double start_pos[6], double end_pos[6], double ref_joints[7], double vel, double acc, double jerk, char* path)
{
	CAxisPln* obj = static_cast<CAxisPln*>(spln);
	if (obj)
	{
		obj->OnMovL(RobotSerial,ref_joints,start_pos, end_pos, vel, acc, jerk, path);
	}
}

//void AxisPln_OnMovL_KeepJ(FXSpln spln, long RobotSerial, double start_pos[6], double end_pos[6], double vel, char* path);
extern "C" void AxisPln_OnMovL_KeepJ(FXSpln spln, long RobotSerial, double start_pos[6], double end_pos[6], double vel, char* path)
{
	CAxisPln* obj = static_cast<CAxisPln*>(spln);
	if (obj)
	{
		obj->OnMovL_KeepJ(RobotSerial, start_pos, end_pos, vel, path);
	}
}


extern "C" void AxisPln_OnMovJ(FXSpln spln, long RobotSerial, double start_joint[7], double end_joint[7], double vel, double acc, double jerk, char* path)
{
	CAxisPln* obj = static_cast<CAxisPln*>(spln);
	if (obj)
	{
		obj->OnMovJ(RobotSerial,start_joint, end_joint, vel, acc, jerk, path);
	}
}

extern "C" void AxisPln_Destroy(FXSpln spln)
{
	CAxisPln* obj = static_cast<CAxisPln*>(spln);
	if (obj)
	{
		delete obj;
	}
}
