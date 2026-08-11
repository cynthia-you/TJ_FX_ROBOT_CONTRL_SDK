
#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>

// =============================================================================
// 示例说明：集中演示机器人运动学与轨迹规划相关接口
//
// 整体流程：
//   阶段一：运动学与坐标系初始化
//     1. 设置运动学打印日志。
//     2. 定义并加载运动学参数（需要注意导入文件名称）。
//     3. 初始化运动学参数
//     4. 设置工具坐标系。
//     5. 设置用户坐标系。
//   阶段二：运动学计算
//     6. 计算正运动学。
//     7. 将 4×4 位姿矩阵转换为 XYZABC。
//     8. 计算雅可比矩阵。
//     9. 计算逆运动学。
//    10. 保持末端位姿不变，计算改变臂角的零空间逆解。
//   阶段三：轨迹规划
//    11. 执行 MOVL 离线直线规划。
//    12. 执行 MOVLA 在线直线规划。
//    13. 执行 MOVL_KeepJ 离线直线规划。
//    14. 执行 MOVL_KeepJA 在线直线规划。
//    15. 执行直线优先规划 FX_Robot_PLN_MOV_Target。
//    16. 执行关节空间规划 MOVJ。
//   阶段四：动力学参数辨识
//    15. 执行工具动力学参数辨识并输出质量、质心和惯量。
//
// 注意：配置文件必须与实际机器人型号和版本一致。
// =============================================================================

void RobotKineDemo()
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

    // [阶段一｜步骤 1] 设置运动学打印日志
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

    // 注意：必须根据 CCS 6 kg、CCS 3 kg 或 SRS 的实际版本选择配置文件。
    // // ccs 6公斤的机型的有两个版本: 3.1(计算配置文件为ccs_m6_31.MvKDCfg), 4.0(计算配置文件为ccs_m6_40.MvKDCfg)，两个版本的参数不一样请确认版本后选择参数.
    // // ccs 3公斤的机型的计算配置文件为ccs_m3.MvKDCfg;
    // srs机型为srs.MvKDCfg.
    // 配置错误可能不会立即报错，但会导致运动学计算结果错误。
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

    // [阶段一｜步骤 3] 初始化运动学参数（机器人类型、DH 参数及运动限制等）
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

    // [阶段一｜步骤 4] 设置工具坐标系
    double tool[4][4] = {0};

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

    if (FX_Robot_Tool_Set(0, tool) == false)
    {
        printf("Robot Set Tool Error\n");
    }
    else
    {
        printf("Robot Set Tool Success\n");
    }

    if (FX_Robot_Tool_Rmv(0) == false)
    {
        printf("Robot Remove Tool Error\n");
    }
    else
    {
        printf("Robot Remove Tool Success\n");
    }

    printf("------------------------------\n");

    // [阶段一｜步骤 5] 设置用户坐标系
    double user_frame_[4][4] = {0};

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            if (i == j)
            {
                user_frame_[i][j] = 1;
            }
            else
            {
                user_frame_[i][j] = 0;
            }
        }
    }

    user_frame_[0][3] += 10;
    user_frame_[1][3] += 10;
    user_frame_[2][3] += 10;

    if (FX_Robot_UserFrame_Set(0, user_frame_) == false)
    {
        printf("Robot Set UserFrame Error\n");
    }
    else
    {
        printf("Robot Set UserFrame Success\n");
    }

    if (FX_Robot_UserFrame_Rmv(0) == false)
    {
        printf("Robot Remove UserFrame Error\n");
    }
    else
    {
        printf("Robot Remove UserFrame Success\n");
    }

    printf("------------------------------\n");

    // [阶段二｜步骤 6] 计算正运动学
    double jv[7] = {10, 20, 30, 40, 50, 10, 10};
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

    // [阶段二｜步骤 7] 将 4×4 位姿矩阵转换为 XYZABC
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

    double mat_result[4][4] = {0};
    FX_XYZABC2Matrix4DEG(xyzabc, mat_result);
    printf("xyzabc to matrix Success\n");

    // [阶段二｜步骤 8] 计算雅可比矩阵
    FX_Jacobi jcb;
    if (FX_Robot_Kine_Jacb(0, jv, &jcb) == false)
    {
        printf("Robot Jacobian Matrix Error\n");
    }
    else
    {
        printf("Robot Jacobian Matrix Success\n");
    }
    printf("------------------------------\n");

    // [阶段二｜步骤 9] 计算逆运动学
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

    if (FX_Robot_Kine_IK(0, &sp) == false)
    {
        printf("Robot Inverse Kinamatics Error\n");
    }
    else
    {
        printf("Robot Inverse Kinamatics Success\n");
    }
    printf("------------------------------\n");

    // [阶段二｜步骤 10] 保持末端位姿不变，计算改变臂角的零空间逆解
    sp.m_Input_IK_ZSPType = 0;
    sp.m_Input_ZSP_Angle -= 1;
    if (FX_Robot_Kine_IK_NSP(0, &sp) == false)
    {
        printf("Robot Null-Space Inverse Kinamatics Error\n");
    }
    else
    {
        printf("Robot Null-Space Inverse Kinamatics Success\n");
    }
    printf("------------------------------\n");

    // [阶段三｜步骤 11] 执行离线直线规划 MOVL
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

    char *path = (char *)"test_movl.txt";
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

    // [阶段三｜步骤 12] 执行在线直线规划 MOVLA
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

    // [阶段三｜步骤 13] 执行离线直线规划 MOVL_KeepJ
    double angle1[7] = {-5.918, -35.767, 49.494, -68.112, -90.699, 49.211, -23.995};
    double angle2[7] = {-26.908, -91.109, 74.502, -88.083, -93.599, 17.151, -13.602};

    char *path1 = (char *)"test_movl_keepj.txt";

    if (FX_Robot_PLN_MOVL_KeepJ(0, angle1, angle2, 100, 100, freq, path1) == false)
    {
        printf("Robot MOVL KeepJ Error\n");
    }
    else
    {
        printf("Robot MOVL KeepJ Success\n");
    }
    printf("------------------------------\n");

    // [阶段三｜步骤 14] 执行在线直线规划 MOVL_KeepJA
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

    // [阶段三｜步骤 15] 执行直线优先规划 MOV_Target
    // 选取一段无法以直线优先规划进而使用关节空间规划的路径
    double jv_target[7] = {-90.0, 35.0, 0.0, -105.0, 130.0, 0.0, 0.0};
    if (FX_Robot_Kine_FK(0, jv, kine_pg) == false)
    {
        printf("Forward Kinematics Error\n");
        goto CONTI;
    }

    {
        double start_target[6] = {0.0};
        double end_target[6] = {0.0};
        if (FX_Matrix42XYZABCDEG(kine_pg, start_target) == false)
        {
            printf("matrix to xyzabc failed.");
            goto CONTI;
        }

        double pg_end[4][4] = {
            {0.67886967, -0.71212705, -0.17891636, 11.97283},
            {0.19112802, 0.40665334, -0.89336618, -383.75744},
            {0.70894716, 0.57228328, 0.41217207, 703.32845},
            {0, 0, 0, 1}};

        if (FX_Matrix42XYZABCDEG(pg_end, end_target) == false)
        {
            printf("matrix to xyzabc failed.");
            goto CONTI;
        }

        CPointSet pset_mov_target;
        if (FX_Robot_PLN_MOV_Target(0, start_target, end_target, jv_target, 100, 100, freq, &pset_mov_target) == false)
        {
            printf("Robot MOV Target Error\n");
        }
        else
        {
            printf("Robot MOV Target Success\n");
        }
    }
    printf("------------------------------\n");

CONTI:
    // [阶段三｜步骤 16] 执行关节空间规划 MOVJ
    CPointSet pset_movj;
    if (FX_Robot_PLN_MOVJ(0, angle1, angle2, 100, 100, freq, &pset_movj) == false)
    {
        printf("Robot MOV Joint Error\n");
    }
    else
    {
        printf("Robot MOV Joint Success\n");
    }
    printf("------------------------------\n");

    // [阶段四｜步骤 17] 执行工具动力学参数辨识
    double ret_m = 0;
    double ret_mr[3] = {0};
    double ret_I[6] = {0};

    char *ipath = (char *)"./LoadData_ccs/LoadData";

    if (FX_Robot_Iden_LoadDyn(1, ipath, &ret_m, ret_mr, ret_I) != 0)
    {
        printf("Robot Tool Dynamics Parameter Identification Error\n");
    }
    else
    {
        printf("tool dyn info =[%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf]\n", ret_m,
               ret_mr[0], ret_mr[1], ret_mr[2],
               ret_I[0], ret_I[3], ret_I[4], ret_I[1], ret_I[5], ret_I[2]); // 惯量顺序：ixx、ixy、ixz、iyy、iyz、izz
        printf("Robot Tool Dynamics Parameter Identification Success\n");
    }
    printf("------------------------------\n");
}

int main()
{
    RobotKineDemo();
}
