#include "LoadIdenPub.h"
#include <cstdio>


int main()
{
	printf("[FxRobot - FX_Robot_Iden_LoadDyn]\n");

	LoadDynamicPara DynPara;
//	FX_BOOL ISCCS = FX_TRUE;

	OnCalLoadDyn(  &DynPara,   false, "./LoadData/");

	return 0;
}
