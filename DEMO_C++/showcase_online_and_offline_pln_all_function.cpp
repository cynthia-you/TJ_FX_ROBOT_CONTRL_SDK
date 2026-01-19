#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>

void RobotPLNDemo()
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

    ///////////////////////0. 关闭打印日志
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

    ////////////////////////3. 计算正运动学获取末端位置姿态矩阵
    FX_DOUBLE jv[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
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


    ////////////////////////4. 4*4位置姿态矩阵 转 xyzabc
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
    printf("------------------------------\n");


    ////////////////////////5. ֱ离线直线规划（MOVL）
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


    ///////////////////////////6. ֱ加载离线直线规划（MOVL）文件
     CPointSet pset_movl;

     char offline_pvt[] = "test.txt";
     char* pvt_file =offline_pvt;

     pset_movl.OnLoadFast(pvt_file);

     int point_num=0;
     point_num=pset_movl.OnGetPointNum();
     printf("[OFFLINE] MOVL number of pvt points:%d\n",point_num);

     for (long tag=0; tag<point_num;tag+=10)//规划文件为500HZ， 下采样为50HZ
     {
         double* pvv=pset_movl.OnGetPoint(tag);
         print_array(pvv,7,"MOVL offline pvt point");
         if (pvv=NULL)
         {
             printf("MOVL offline pln Error\n");
         }
     }
     printf("------------------------------\n");


     ////////////////////////7. ֱ在线直线规划（MOVLA）并执行点
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

     int point_num1=0;
     point_num1=pset_movla.OnGetPointNum();
     printf("[ONLINE] MOVL number of online pvt points:%d\n",point_num1);

     for (long tag=0; tag<point_num1;tag+=10)//规划点位为500HZ， 下采样为50HZ
     {
         double* pvv1=pset_movla.OnGetPoint(tag);
         print_array(pvv1,7,"MOVLA online pvt point");
         if (pvv1=NULL)
         {
             printf("MOVLA online pln Error\n");
         }
     }
     printf("------------------------------\n");

     ////////////////////////8. 离线直线规划（MOVL_KeepJ）
     FX_DOUBLE angle1[7] = { -5.918, -35.767, 49.494, -68.112, -90.699, 49.211, -23.995 };
     FX_DOUBLE angle2[7] = { -26.908 ,-91.109, 74.502 ,-88.083, -93.599 ,17.151, -13.602 };

     char op1[] = "testkeepj.txt";
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

     ////////////////////////9. 加载离线直线规划（MOVL_keepj）文件
     CPointSet pset_movl_keepj;

     char offline_pvt_movl_keepj[] = "testkeepj.txt";
     char* pvt_file1 =offline_pvt_movl_keepj;

     pset_movl_keepj.OnLoadFast(pvt_file1);

     int point_num2=0;
     point_num2=pset_movl_keepj.OnGetPointNum();
     printf("[OFFLINE] MOVL number of pvt points:%d\n",point_num2);

     for (long tag=0; tag<point_num2;tag+=20)
     {
         double* pvv2=pset_movl_keepj.OnGetPoint(tag);
         print_array(pvv2,7,"MOVL_KEEPJ offline pvt point");
         if (pvv2=NULL)
         {
             printf("MOVL_KEEPJ offline pln Error\n");
         }
     }
     printf("------------------------------\n");


     ////////////////////////10. 在线直线规划（MOVL_KeepJA）并执行点
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

     int point_num3=0;
     point_num3=pset_movl_keepja.OnGetPointNum();
     printf("[ONLINE] MOVL_KEEPJA  number of online pvt points:%d\n",point_num3);

     for (long tag=0; tag<point_num3;tag+=20)
     {
         double* pvv3=pset_movl_keepja.OnGetPoint(tag);
         print_array(pvv3,7,"MOVL_KEEPJA online pvt point");
         if (pvv3=NULL)
         {
             printf("MOVL_KEEPJA online pln Error\n");
         }
     }
     printf("------------------------------\n");
}

int main()
{
    RobotPLNDemo();
}
