#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>

void RobotKineDemo()
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


    ////////////////////////计算正运动学
    FX_DOUBLE jv[7] = { 10, 10,10,10,10, 10,10 };
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



    ////////////////////////直线规划（MOVL）
    //特别提示:直线规划前,需要将起始关节位置调正解接口,将数据更新到起始关节.
    Vect6 start = { -187.0254426109816, -66.71264281828329, 832.0536847286807, 4.174345954026256, -29.162364453381763, 32.135018044129566};//调用FX_Matrix42XYZABCDEG得到的XYZABC
    Vect6 end = { -177.0254426109816, -66.71264281828329, 832.0536847286807, 4.174345954026256, -29.162364453381763, 32.135018044129566};// 在X方向移动10毫米

    char op[] = "test.txt";
    char* path = op;

    if (FX_Robot_PLN_MOVL(0, start, end, jv, 100, 100, path) == FX_FALSE)
    {
        printf("Robot MOVL Error\n");
    }
    else
    {
        printf("Robot MOVL Success\n");
    }
    printf("------------------------------\n");

}

int main()
{
    RobotKineDemo();
    // testpln();
}
