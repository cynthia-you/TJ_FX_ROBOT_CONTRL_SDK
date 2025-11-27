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


    ////////////////////////test MOVL J
    FX_DOUBLE start_j[7] = { 10,20,30,40,50,10,20 };
    FX_DOUBLE stop_j[7] = { 20,30,40,50,60,70,80 };

    /* FX_DOUBLE angle1[7] = { -146.830,7.110,6.660,-94.920,2.690,5.11,-47.390 };
     FX_DOUBLE angle2[7] = { -146.043,-16.088,0.562,-68.108,1.848,5.167,-61.515 };*/


    FX_DOUBLE angle1[7] = { -5.918, -35.767, 49.494, -68.112, -90.699, 49.211, -23.995 };
    FX_DOUBLE angle2[7] = { -26.908 ,-91.109, 74.502 ,-88.083, -93.599 ,17.151, -13.602 };

    char op[] = "testkeepj.txt";
    char* path = op;

    FX_Robot_PLN_MOVL_KeepJ(0, angle1, angle2, 20, path);

}

int main()
{
    RobotKineDemo();
}
