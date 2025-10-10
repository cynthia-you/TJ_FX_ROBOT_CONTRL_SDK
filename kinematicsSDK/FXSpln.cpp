#include "FXSpln.h"
#include "AxisPln.h"

extern "C" FXSpln AxisPln_Create()
{
	return new CAxisPln();
}

extern "C" void AxisPln_OnMovL(FXSpln spln, double start_pos[6], double end_pos[6], double ref_joints[7], double vel, double acc, double jerk, char* path)
{
	CAxisPln* obj = static_cast<CAxisPln*>(spln);
	if (obj)
	{
		obj->OnMovL(ref_joints,start_pos, end_pos, vel, acc, jerk, path);
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
