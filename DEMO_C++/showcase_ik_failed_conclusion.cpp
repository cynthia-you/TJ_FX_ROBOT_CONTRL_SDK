#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>

void KineFailedDemo()
{
    FX_INT32L i = 0;
    FX_INT32L j = 0;

   ////////////////////////导入运动学参数
    FX_INT32L TYPE[2];
    FX_DOUBLE GRV[2][3];
    FX_DOUBLE DH[2][8][4];
    FX_DOUBLE PNVA[2][7][4];
    FX_DOUBLE BD[2][4][3];

    FX_DOUBLE Mass[2][7];
    FX_DOUBLE MCP[2][7][3];
    FX_DOUBLE I[2][7][6];
    if (LOADMvCfg((char*)"ccs_m6.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == FX_TRUE)
    {
        printf("Robot Load CFG Success\n");
    }
    else
    {
        printf("Robot Load CFG Error\n");
    }
    printf("------------------------------\n");

    ////////////////////////初始化运动学参数
    if (FX_Robot_Init_Type(0, TYPE[0]) == FX_FALSE)
    {
        printf("Robot Init Type Error\n");
    }
    else
    {
        printf("Robot Init Type Success\n");
    }

    if (FX_Robot_Init_Kine(0, DH[0]) == FX_FALSE)
    {
        printf("Robot Init DH Parameters Error\n");
    }
    else
    {
        printf("Robot Init DH Parameters Success\n");
    }

    if (FX_Robot_Init_Lmt(0, PNVA[0], BD[0]) == FX_FALSE)
    {
        printf("Robot Init Limit Parameters Error\n");
    }
    else
    {
        printf("Robot Init Limit Parameters Success\n");
    }
    printf("------------------------------\n");

    FX_DOUBLE jv[7] = { 10, 10, 10, 0, 10, 10, 10};
    Matrix4 kine_pg;
    if (FX_Robot_Kine_FK(0, jv, kine_pg) == FX_FALSE)
    {
        printf("Robot Forward Kinematics Error\n");
    }
    else
    {
        printf("Robot Forward Kinematics Success\n");
    }
    printf("------------------------------\n");

    ////////////////////////逆向解失败情况1: 四关节为0
    FX_InvKineSolvePara sp;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sp.m_Input_IK_TargetTCP[i][j] = kine_pg[i][j];
        }
    }

    for (i = 0; i < 7; i++)
    {
        sp.m_Input_IK_RefJoint[i] = jv[i];
    }
    if (FX_Robot_Kine_IK(0, &sp) == FX_FALSE)
    {
        printf("Robot Inverse Kinamatics Error\n");
    }
    else
    {
        printf("Robot Inverse Kinamatics Success\n");
    }
    printf("------------------------------\n");


    ////////////////////////#逆向解失败情况2: 超可达空间
    Vect6 xyzabc={1000,500,300,0,0,0};
    Matrix4 mat_result;
    FX_XYZABC2Matrix4DEG(xyzabc,mat_result);
    FX_DOUBLE jv1[7]={10, 10, 10, 10, 10, 10, 10};

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sp.m_Input_IK_TargetTCP[i][j] = mat_result[i][j];
        }
    }

    for (i = 0; i < 7; i++)
    {
        sp.m_Input_IK_RefJoint[i] = jv1[i];
    }
    if (FX_Robot_Kine_IK(0, &sp) == FX_FALSE)
    {
        printf("Robot Inverse Kinamatics Error\n");
    }
    else
    {
        printf("Robot Inverse Kinamatics Success\n");
    }

}

int main()
{
    KineFailedDemo();
}
