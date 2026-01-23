
#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>

void RobotKineDemo()
{
    auto print_array = [](auto* arr, size_t n, const char* name = "", int precision = 2) {
        if (name[0] != '\0') printf("%s=", name);
        printf("[");
        for (size_t i = 0; i < n; ++i) {
            printf("%.*lf%s", precision, arr[i], i < n-1 ? "," : "");
        }
        printf("]\n");
    };

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

     ///////////////////////0 关闭打印日志
     bool log_switch=false;
     FX_LOG_SWITCH(log_switch);
    
   ////////////////////////1.导入运动学参数
    FX_INT32L TYPE[2];
    FX_DOUBLE GRV[2][3];
    FX_DOUBLE DH[2][8][4];
    FX_DOUBLE PNVA[2][7][4];
    FX_DOUBLE BD[2][4][3];

    FX_DOUBLE Mass[2][7];
    FX_DOUBLE MCP[2][7][3];
    FX_DOUBLE I[2][7][6];


    // ccs 6公斤的机型的有两个版本: 3.1(计算配置文件为ccs_m6_31.MvKDCfg), 4.0(计算配置文件为ccs_m6_40.MvKDCfg)，两个版本的参数不一样请确认版本后选择参数.
    // ccs 3公斤的机型的计算配置文件为ccs_m3.MvKDCfg； 
    // srs机型为srs.MvKDCfg.
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
    FX_DOUBLE jv[7] = { 10, 20, 30, 40, 50, 10,10 };
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
        print_array(xyzabc,6,"xyzabc");
    }

    Matrix4 mat_result;
    FX_XYZABC2Matrix4DEG(xyzabc,mat_result);
    printf("xyzabc to matrix Success\n");
    print_matrix(mat_result,4,4,"mat_rersukts");
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
    Vect6 start = {0.0};
     for (i = 0; i < 6; i++)
    {
        start[i] = xyzabc[i];
    }

    Vect6 end = {0.0};
    for (i = 0; i < 6; i++)
    {
        end[i] = xyzabc[i];
    }

    end[0]+=10;//末端X方向移动10毫米

    char op[] = "test_movl.txt";
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


    ////////////////////////10. ֱ在线直线规划（MOVLA）并执行点
     CPointSet pset_movla;
     if (FX_Robot_PLN_MOVLA(0, start, end, jv, 100, 100, &pset_movla) == FX_FALSE)
     {
         printf("Robot MOVLA Error\n");
     }
     else
     {
         printf("Robot MOVLA Success\n");
     }
     printf("------------------------------\n");


     ////////////////////////11. ֱ直线规划（MOVL_KeepJ）
     FX_DOUBLE angle1[7] = { -5.918, -35.767, 49.494, -68.112, -90.699, 49.211, -23.995 };
     FX_DOUBLE angle2[7] = { -26.908 ,-91.109, 74.502 ,-88.083, -93.599 ,17.151, -13.602 };

     char op1[] = "test_movl_keepj.txt";
     char* path1 = op1;

     if (FX_Robot_PLN_MOVL_KeepJ(0, angle1, angle2, 100, 100,path1)== FX_FALSE)
     {
         printf("Robot MOVL KeepJ Error\n");
     }
     else
     {
         printf("Robot MOVL KeepJ Success\n");
     }
     printf("------------------------------\n");


     ////////////////////////12. 在线直线规划（MOVL_KeepJA）并执行点
     CPointSet pset_movl_keepja;
     if (FX_Robot_PLN_MOVL_KeepJA(0, angle1, angle2, 100, 100,&pset_movl_keepja)== FX_FALSE)
     {
         printf("Robot MOVL KeepJA Error\n");
     }
     else
     {
         printf("Robot MOVL KeepJA Success\n");
     }
     printf("------------------------------\n");



     ////////////////////////13. 工具动力学参数辨识
     FX_DOUBLE ret_m = 0;
     Vect3 ret_mr = { 0 };
     Vect6 ret_I = { 0 };

     char ip[] = "./LoadData_ccs_right/LoadData";
     char* ipath = ip;

     if (FX_Robot_Iden_LoadDyn(1, ipath, &ret_m, ret_mr, ret_I) != 0)
     {
         printf("Robot Tool Dynamics Parameter Identification Error\n");
     }
     else
     {
         printf("tool dyn info =[%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf]\n",ret_m,
         ret_mr[0],ret_mr[1],ret_mr[2],
         ret_I[0],ret_I[3],ret_I[4],ret_I[1],ret_I[5],ret_I[2]);//ixx,ixy,ixz,iyy,iyz,izz
         printf("Robot Tool Dynamics Parameter Identification Success\n");
     }
     printf("------------------------------\n");


}

int main()
{
    RobotKineDemo();
}
