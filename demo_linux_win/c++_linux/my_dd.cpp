#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>

void RobotKineDemo()
{
    FX_INT32L i = 0;
    FX_INT32L j = 0;

   ////////////////////////1.导入运动学参数
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

    ////////////////////////2. 初始化运动学参数
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

    ////////////////////////3.工具设置
    Matrix4 tool;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            if (i == j)
            {
                tool[i][j] = 1;
            }
            else
            {
                tool[i][j] = 0;
            }

        }

    }

    if (FX_Robot_Tool_Set(0, tool) == FX_FALSE)
    {
        printf("Robot Set Tool Error\n");
    }
    else
    {
        printf("Robot Set Tool Success\n");
    }

    if (FX_Robot_Tool_Rmv(0) == FX_FALSE)
    {
        printf("Robot Remove Tool Error\n");
    }
    else
    {
        printf("Robot Remove Tool Success\n");
    }


    printf("------------------------------\n");

    ////////////////////////4. 计算正运动学
    FX_DOUBLE jv[7] = { 10, 20, 30, 40, 50, 60, 70 };
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


    ////////////////////////5. 4*4位置姿态矩阵 转 xyzabc
    Vect6 xyzabc={0};

    if (FX_Matrix42XYZABCDEG(kine_pg, xyzabc) == FX_FALSE)
    {
        printf("matrix to xyzabc failed.");
    }
    else
    {
        printf("matrix to xyzabc Success\n");
        printf("xyzabc=[%lf,%lf,%lf,%lf,%lf,%lf]\n",xyzabc[0],xyzabc[1],xyzabc[2],xyzabc[3],xyzabc[4],xyzabc[5]);
    }
    printf("------------------------------\n");

    ////////////////////////6. 计算雅可比矩阵
    FX_Jacobi jcb;
    if (FX_Robot_Kine_Jacb(0, jv, &jcb) == FX_FALSE)
    {
        printf("Robot Jacobian Matrix Error\n");
    }
    else
    {
        printf("Robot Jacobian Matrix Success\n");
    }
    printf("------------------------------\n");


    ////////////////////////7. 计算逆运动学
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
    //FX_M44Copy(kine_pg, sp.m_Input_IK_TargetTCP);
    //FX_Vect7Copy(jv, sp.m_Input_IK_RefJoint);
    if (FX_Robot_Kine_IK(0, &sp) == FX_FALSE)
    {
        printf("Robot Inverse Kinamatics Error\n");
    }
    else
    {
        printf("Robot Inverse Kinamatics Success\n");
    }
    printf("------------------------------\n");

    ////////////////////////8.计算末端位姿不变、改变零空间（臂角方向）的逆运动学
    sp.m_Input_IK_ZSPType = 0;
    sp.m_Input_ZSP_Angle -= 1;
    if (FX_Robot_Kine_IK_NSP(0, &sp) == FX_FALSE)
    {
        printf("Robot Null-Space Inverse Kinamatics Error\n");
    }
    else
    {
        printf("Robot Null-Space Inverse Kinamatics Success\n");
    }
    printf("------------------------------\n");



    ////////////////////////9. ֱ直线规划（MOVL）
    //特别提示:直线规划前,需要将起始关节位置调正解接口,将数据更新到起始关节.
    Vect6 start = { -261.62825703744505, -201.62460924482926, 664.7097505619455, -26.448945032606105, -41.30548147663203, 176.4130453346306};//调用FX_Matrix42XYZABCDEG得到的XYZABC
    Vect6 end = { -251.62825703744505, -201.62460924482926, 664.7097505619455, -26.448945032606105, -41.30548147663203, 176.4130453346306};// 在X方向移动10毫米

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

    ////////////////////////10. 工具动力学参数辨识
    FX_DOUBLE ret_m = 0;
    Vect3 ret_mr = { 0 };
    Vect6 ret_I = { 0 };

    char ip[] = "./LoadData";
    char* ipath = ip;

    if (FX_Robot_Iden_LoadDyn(1007, ipath, &ret_m, ret_mr, ret_I) == FX_FALSE)
    {
        printf("Robot Tool Dynamics Parameter Identification Error\n");
    }
    else
    {
        printf("mass=%lf\n",ret_m);
        printf("Robot Tool Dynamics Parameter Identification Success\n");
    }
    printf("------------------------------\n");


}

int main()
{
    RobotKineDemo();
    // testpln();
}
