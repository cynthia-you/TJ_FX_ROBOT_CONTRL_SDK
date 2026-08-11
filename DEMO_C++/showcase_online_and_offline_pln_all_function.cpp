#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>

// =============================================================================
// 示例说明：集中演示直线轨迹的在线、离线及保持关节构型规划接口
//
// 整体流程：
//   阶段一：运动学初始化
//     1. 关闭运动学打印日志。
//     2. 定义并加载运动学参数。
//     3. 初始化机器人类型、DH 参数及运动限制。
//     4. 通过正运动学计算起点末端位姿。
//     5. 将起点位姿矩阵转换为 XYZABC。
//   阶段二：MOVL 离线规划
//     6. 设置直线终点并执行 MOVL 离线规划。
//     7. 加载离线轨迹文件并以 50 Hz 执行。
//   阶段三：MOVLA 在线规划
//     8. 生成 MOVLA 在线轨迹并以 50 Hz 执行。
//   阶段四：保持关节构型的直线规划
//     9. 执行 MOVL_KeepJ 离线规划。
//    10. 加载并执行 MOVL_KeepJ 离线轨迹文件。
//    11. 生成并执行 MOVL_KeepJA 在线轨迹。
// =============================================================================

void RobotPLNDemo()
{
    auto print_array = [](auto *arr, size_t n, const char *name = "", int precision = 2)
    {
        if (name[0] != '\0')
            printf("%s=", name);
        printf("[");
        for (size_t i = 0; i < n; ++i)
        {
            printf("%.*lf%s", precision, arr[i], i < n - 1 ? "," : "");
        }
        printf("]\n");
    };

    auto print_matrix = [](auto *mat, size_t rows, size_t cols, const char *name = "", int precision = 2)
    {
        if (name[0] != '\0')
            printf("%s=\n", name);
        for (size_t i = 0; i < rows; ++i)
        {
            printf("%s[", i == 0 ? "[" : " ");
            for (size_t j = 0; j < cols; ++j)
            {
                printf("%.*lf%s", precision, mat[i][j], j < cols - 1 ? "," : "");
            }
            printf("]%s\n", i < rows - 1 ? "," : "]");
        }
    };

    int i = 0;
    int j = 0;

    // [阶段一｜步骤 1] 关闭运动学打印日志
    bool log_switch = false;
    FX_LOG_SWITCH(log_switch);

    // [阶段一｜步骤 2] 定义并加载运动学参数
    int TYPE[2];
    double GRV[2][3];
    double DH[2][8][4];
    double PNVA[2][7][4];
    double BD[2][4][3];

    double Mass[2][7];
    double MCP[2][7][3];
    double I[2][7][6];

    // 注意：配置文件必须与实际机型、版本及左右臂设置一致。
    // // ccs 6公斤的机型的有两个版本: 3.1(计算配置文件为ccs_m6_31.MvKDCfg), 4.0(计算配置文件为ccs_m6_40.MvKDCfg)，两个版本的参数不一样请确认版本后选择参数.
    // // ccs 3公斤的机型的计算配置文件为ccs_m3.MvKDCfg；
    // // srs机型为srs.MvKDCfg.
    // 同时需要确认 arm_type 对应左臂（0）还是右臂（1）。
    if (LOADMvCfg((char *)"ccs_m6_40.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == true)
    {
        printf("Robot Load CFG Success\n");
    }
    else
    {
        printf("Robot Load CFG Error\n");
    }
    printf("------------------------------\n");

    // [阶段一｜步骤 3] 初始化运动学参数
    if (FX_Robot_Init_Type(0, TYPE[0]) == false)
    {
        printf("Robot Init Type Error\n");
    }
    else
    {
        printf("Robot Init Type Success\n");
    }

    if (FX_Robot_Init_Kine(0, DH[0]) == false)
    {
        printf("Robot Init DH Parameters Error\n");
    }
    else
    {
        printf("Robot Init DH Parameters Success\n");
    }

    if (FX_Robot_Init_Lmt(0, PNVA[0], BD[0]) == false)
    {
        printf("Robot Init Limit Parameters Error\n");
    }
    else
    {
        printf("Robot Init Limit Parameters Success\n");
    }
    printf("------------------------------\n");

    // [阶段一｜步骤 4] 通过正运动学计算起点末端位姿
    double jv[7] = {44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1};
    double kine_pg[4][4] = {0};
    if (FX_Robot_Kine_FK(0, jv, kine_pg) == false)
    {
        printf("Robot Forward Kinematics Error\n");
    }
    else
    {
        printf("Robot Forward Kinematics Success\n");
    }
    printf("------------------------------\n");

    // [阶段一｜步骤 5] 将 4×4 位姿矩阵转换为 XYZABC
    double xyzabc[6] = {0};

    if (FX_Matrix42XYZABCDEG(kine_pg, xyzabc) == false)
    {
        printf("matrix to xyzabc failed.");
    }
    else
    {
        printf("matrix to xyzabc Success\n");
        print_array(xyzabc, 6, "xyzabc");
    }
    printf("------------------------------\n");

    // [阶段二｜步骤 6] 执行 MOVL 离线直线规划
    double start[6] = {0.0};
    for (i = 0; i < 6; i++)
    {
        start[i] = xyzabc[i];
    }

    double end[6] = {0.0};
    for (i = 0; i < 6; i++)
    {
        end[i] = xyzabc[i];
    }

    end[0] += 10; // 末端沿 X 方向移动 10 mm

    char *path = (char *)"test.txt";
    long freq = 500;
    if (FX_Robot_PLN_MOVL(0, start, end, jv, 100, 100, freq, path) == false)
    {
        printf("Robot MOVL Error\n");
    }
    else
    {
        printf("Robot MOVL Success\n");
    }
    printf("------------------------------\n");

    // [阶段二｜步骤 7] 加载并执行 MOVL 离线轨迹文件
    CPointSet pset_movl;

    char *pvt_file = (char *)"test.txt";

    pset_movl.OnLoadFast(pvt_file);

    int point_num = 0;
    point_num = pset_movl.OnGetPointNum();
    printf("[OFFLINE] MOVL number of pvt points:%d\n", point_num);

    for (long tag = 0; tag < point_num; tag += 10) // 500 Hz 轨迹下采样为 50 Hz
    {
        double *pvv = pset_movl.OnGetPoint(tag);
        print_array(pvv, 7, "MOVL offline pvt point");
        if (pvv = NULL)
        {
            printf("MOVL offline pln Error\n");
        }
    }
    printf("------------------------------\n");

    // [阶段三｜步骤 8] 执行 MOVLA 在线直线规划
    CPointSet pset_movla;
    if (FX_Robot_PLN_MOVLA(0, start, end, jv, 100, 100, freq, &pset_movla) == false)
    {
        printf("Robot MOVLA Error\n");
    }
    else
    {
        printf("Robot MOVLA Success\n");
    }
    printf("------------------------------\n");

    int point_num1 = 0;
    point_num1 = pset_movla.OnGetPointNum();
    printf("[ONLINE] MOVL number of online pvt points:%d\n", point_num1);

    for (long tag = 0; tag < point_num1; tag += 10) // 500 Hz 轨迹下采样为 50 Hz
    {
        double *pvv1 = pset_movla.OnGetPoint(tag);
        print_array(pvv1, 7, "MOVLA online pvt point");
        if (pvv1 = NULL)
        {
            printf("MOVLA online pln Error\n");
        }
    }
    printf("------------------------------\n");

    // [阶段四｜步骤 9] 执行 MOVL_KeepJ 离线规划
    double angle1[7] = {-5.918, -35.767, 49.494, -68.112, -90.699, 49.211, -23.995};
    double angle2[7] = {-26.908, -91.109, 74.502, -88.083, -93.599, 17.151, -13.602};

    char *path1 = (char *)"testkeepj.txt";

    if (FX_Robot_PLN_MOVL_KeepJ(0, angle1, angle2, 100, 100, freq, path1) == false)
    {
        printf("Robot MOVL KeepJ Error\n");
    }
    else
    {
        printf("Robot MOVL KeepJ Success\n");
    }
    printf("------------------------------\n");

    // [阶段四｜步骤 10] 加载并执行 MOVL_KeepJ 轨迹文件
    CPointSet pset_movl_keepj;

    char *pvt_file1 = (char *)"testkeepj.txt";

    pset_movl_keepj.OnLoadFast(pvt_file1);

    int point_num2 = 0;
    point_num2 = pset_movl_keepj.OnGetPointNum();
    printf("[OFFLINE] MOVL number of pvt points:%d\n", point_num2);

    for (long tag = 0; tag < point_num2; tag += 20)
    {
        double *pvv2 = pset_movl_keepj.OnGetPoint(tag);
        print_array(pvv2, 7, "MOVL_KEEPJ offline pvt point");
        if (pvv2 = NULL)
        {
            printf("MOVL_KEEPJ offline pln Error\n");
        }
    }
    printf("------------------------------\n");

    // [阶段四｜步骤 11] 执行 MOVL_KeepJA 在线规划
    CPointSet pset_movl_keepja;
    if (FX_Robot_PLN_MOVL_KeepJA(0, angle1, angle2, 100, 100, freq, &pset_movl_keepja) == false)
    {
        printf("Robot MOVL KeepJA Error\n");
    }
    else
    {
        printf("Robot MOVL KeepJA Success\n");
    }
    printf("------------------------------\n");

    int point_num3 = 0;
    point_num3 = pset_movl_keepja.OnGetPointNum();
    printf("[ONLINE] MOVL_KEEPJA  number of online pvt points:%d\n", point_num3);

    for (long tag = 0; tag < point_num3; tag += 20)
    {
        double *pvv3 = pset_movl_keepja.OnGetPoint(tag);
        print_array(pvv3, 7, "MOVL_KEEPJA online pvt point");
        if (pvv3 = NULL)
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
