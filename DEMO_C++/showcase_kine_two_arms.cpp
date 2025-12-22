#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>

void KineTwoArmsDemo()
{
    
    // 打印数组的lambda - 保留两位小数
    auto print_array = [](auto* arr, size_t n, const char* name = "", int precision = 2) {
        if (name[0] != '\0') printf("%s=", name);
        printf("[");
        for (size_t i = 0; i < n; ++i) {
            printf("%.*lf%s", precision, arr[i], i < n-1 ? "," : "");
        }
        printf("]\n");
    };

    // 打印矩阵的lambda - 保留两位小数
    auto print_matrix = [](auto* mat, size_t rows, size_t cols, const char* name = "", int precision = 2) {
        if (name[0] != '\0') printf("%s=\n", name);
        for (size_t i = 0; i < rows; ++i) {
            printf("%s[", i == 0 ? "[" : " ");
            for (size_t j = 0; j < cols; ++j) {
                printf("%.*lf%s", precision, mat[i][j], j < cols-1 ? "," : "");
            }
            printf("]%s\n", i < rows-1 ? "," : "]");
        }
    };


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
    printf("A arm\n");
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

    printf("B arm\n");
    if (FX_Robot_Init_Type(1, TYPE[1]) == FX_FALSE)
    {
        printf("Robot Init Type Error\n");
    }
    else
    {
        printf("Robot Init Type Success\n");
    }

    if (FX_Robot_Init_Kine(1, DH[1]) == FX_FALSE)
    {
        printf("Robot Init DH Parameters Error\n");
    }
    else
    {
        printf("Robot Init DH Parameters Success\n");
    }

    if (FX_Robot_Init_Lmt(1, PNVA[1], BD[1]) == FX_FALSE)
    {
        printf("Robot Init Limit Parameters Error\n");
    }
    else
    {
        printf("Robot Init Limit Parameters Success\n");
    }
    printf("------------------------------\n");


    ////////////////////////计算正运动学
    printf("A arm\n");
    FX_DOUBLE jv[7] = { 10, 20, 30, 40, 50, 10,10 };
    Matrix4 kine_pg;
    if (FX_Robot_Kine_FK(0, jv, kine_pg) == FX_FALSE)
    {
        printf("Robot Forward Kinematics Error\n");
    }
    else
    {
        printf("Robot Forward Kinematics Success\n");
        print_matrix(kine_pg,4,4,"kine_pg");
    }

    printf("B arm\n");
    FX_DOUBLE jv1[7] = { 10, 20, 30, 40, 50, 10,0 };
    Matrix4 kine_pg1;
    if (FX_Robot_Kine_FK(1, jv1, kine_pg1) == FX_FALSE)
    {
        printf("Robot Forward Kinematics Error\n");
    }
    else
    {
        printf("Robot Forward Kinematics Success\n");
        print_matrix(kine_pg1,4,4,"kine_pg1");
    }
    printf("------------------------------\n");


     ////////////////////////计算逆运动学
    printf("A arm\n");
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
        print_array(sp.m_Output_RetJoint,7,"ik result under reference joints");
    }


    printf("B arm\n");
    FX_InvKineSolvePara sp1;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            sp1.m_Input_IK_TargetTCP[i][j] = kine_pg1[i][j];
        }
    }

    for (i = 0; i < 7; i++)
    {
        sp1.m_Input_IK_RefJoint[i] = jv1[i];
    }

    if (FX_Robot_Kine_IK(0, &sp1) == FX_FALSE)
    {
        printf("Robot Inverse Kinamatics Error\n");
    }
    else
    {
        printf("Robot Inverse Kinamatics Success\n");
        print_array(sp1.m_Output_RetJoint,7,"ik result under reference joints");
}
    printf("------------------------------\n");



}

int main()
{
    KineTwoArmsDemo();
}
