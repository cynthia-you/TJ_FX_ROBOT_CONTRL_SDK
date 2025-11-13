#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>

int main()
{

    FX_DOUBLE ret_m = 0;
    Vect3 ret_mr = { 0 };
    Vect6 ret_I = { 0 };
    char ip[] = "LoadData_ccs_right/LoadData";
    char* ipath = ip;

    if (FX_Robot_Iden_LoadDyn(1, ipath, &ret_m, ret_mr, ret_I) !=0)
    {
        printf("Robot Tool Dynamics Parameter Identification Error\n");
        //typedef enum {
        //	LOAD_IDEN_NoErr = 0, // No error
        //	LOAD_IDEN_CalErr = 1, // Calculation error, 计算错误，需重新采集数据计算
        //	LOAD_IDEN_OpenSmpDateFieErr = 2, //  Open sample file error 打开采集数据文件错误，须检查采样文件
        //	LOAD_IDEN_OpenCfgFileErr = 3, // Open config file error 配置文件被修改
        //	LOAD_IDEN_DataSmpErr = 4 // Data sample error 采集时间不够，缺少有效数据
        //}LoadIdenErrCode;

    }
    else
    {
        printf("Robot Tool Dynamics Parameter Identification Success\n");
        printf("identy results [mass,mx,my,mz,ixx,iyy,izz,ixy,ixz,iyz]=[%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf] \n",ret_m,ret_mr[0],ret_mr[1],ret_mr[2],ret_I[0],ret_I[1],ret_I[2],ret_I[3],ret_I[4],ret_I[5]);
        double tool_dynamics_in_marvin[6]={0};
        tool_dynamics_in_marvin[0]=ret_m;
        tool_dynamics_in_marvin[1]=ret_mr[0];
        tool_dynamics_in_marvin[2]=ret_mr[1];
        tool_dynamics_in_marvin[3]=ret_mr[2];
        tool_dynamics_in_marvin[4]=ret_I[0];
        tool_dynamics_in_marvin[5]=ret_I[3];
        tool_dynamics_in_marvin[6]=ret_I[4];
        tool_dynamics_in_marvin[7]=ret_I[1];
        tool_dynamics_in_marvin[8]=ret_I[5];
        tool_dynamics_in_marvin[9]=ret_I[2];
        printf("order of tool dynamics in MARVIN:[mass,mx,my,mz,ixx,ixy,ixz,iyy,iyz,izz]=[%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf] \n",
        tool_dynamics_in_marvin[0],tool_dynamics_in_marvin[1],tool_dynamics_in_marvin[2],tool_dynamics_in_marvin[3],
        tool_dynamics_in_marvin[4],tool_dynamics_in_marvin[5],tool_dynamics_in_marvin[6],tool_dynamics_in_marvin[7],
        tool_dynamics_in_marvin[8],tool_dynamics_in_marvin[9]);


        return -1;
    }
    printf("------------------------------\n");

	return 0;
}
