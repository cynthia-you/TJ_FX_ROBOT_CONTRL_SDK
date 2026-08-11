#include "FxRobot.h"
#include <stdio.h>
#include <stdlib.h>

// =============================================================================
// 示例说明：演示双臂运动学接口的初始化、坐标配置、正解和逆解
//
// 整体流程：
//   阶段一：双臂运动学初始化
//     1. 设置运动学日志打印开关。
//     2. 定义并加载双臂运动学参数。
//     3. 初始化双臂机器人类型、DH 参数及运动限制。
//   阶段二：坐标系配置
//     4. 设置双臂工具坐标系。
//     5. 设置双臂用户坐标系。
//   阶段三：正逆运动学计算
//     6. 根据双臂关节角计算并输出末端位姿。
//     7. 根据目标末端位姿计算并输出双臂关节角。
//
// 注意：配置文件必须与实际机器人型号、版本及左右臂设置一致。
// =============================================================================

void KineTwoArmsDemo()
{
    // 辅助函数：按指定精度打印数组
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

    // 辅助函数：按指定精度打印矩阵
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

    long i = 0;
    long j = 0;

    // [阶段一｜步骤 1] 设置运动学日志打印开关
    int log_tag_ = 0;
    FX_LOG_SWITCH(log_tag_);

    // [阶段一｜步骤 2] 定义并加载双臂运动学参数
    int TYPE[2];
    double GRV[2][3];
    double DH[2][8][4];
    double PNVA[2][7][4];
    double BD[2][4][3];

    double Mass[2][7];
    double MCP[2][7][3];
    double I[2][7][6];

    // 注意：配置文件与实际机型不匹配时，程序可能正常运行但计算结果错误。
    // CCS 6 kg、CCS 3 kg 和 SRS 机型应分别选择对应的 MvKDCfg 文件。
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

    // [阶段一｜步骤 3] 初始化双臂运动学参数
    printf("A arm\n");
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

    printf("B arm\n");
    if (FX_Robot_Init_Type(1, TYPE[1]) == false)
    {
        printf("Robot Init Type Error\n");
    }
    else
    {
        printf("Robot Init Type Success\n");
    }

    if (FX_Robot_Init_Kine(1, DH[1]) == false)
    {
        printf("Robot Init DH Parameters Error\n");
    }
    else
    {
        printf("Robot Init DH Parameters Success\n");
    }

    if (FX_Robot_Init_Lmt(1, PNVA[1], BD[1]) == false)
    {
        printf("Robot Init Limit Parameters Error\n");
    }
    else
    {
        printf("Robot Init Limit Parameters Success\n");
    }
    printf("------------------------------\n");

    // [阶段二｜步骤 4] 设置双臂工具坐标系
    double tool1[4][4] = {{0}};
    double tool2[4][4] = {{0}};
    for (i = 0; i < 4; i++)
    {
        tool1[i][i] = 1;
        tool2[i][i] = 1;
    }
    tool1[0][3] += 100;

    if (FX_Robot_Tool_Set(0, tool1) == false)
    {
        printf("Robot Set Arm0 Tool Error\n");
    }

    if (FX_Robot_Tool_Set(1, tool2) == false)
    {
        printf("Robot Set Arm1 Tool Error\n");
    }

    // [阶段二｜步骤 5] 设置双臂用户坐标系
    double user_frame1_[4][4] = {{0}};
    double user_frame2_[4][4] = {{0}};
    for (i = 0; i < 4; i++)
    {
        user_frame1_[i][i] = 1;
        user_frame2_[i][i] = 1;
    }
    user_frame1_[2][3] -= 1000;

    if (FX_Robot_UserFrame_Set(0, user_frame1_) == false)
    {
        printf("Robot Set Arm0 UserFrame Error\n");
    }

    if (FX_Robot_UserFrame_Set(1, user_frame2_) == false)
    {
        printf("Robot Set Arm1 UserFrame Error\n");
    }

    // [阶段三｜步骤 6] 计算并输出双臂正运动学结果
    printf("A arm\n");
    double jv[7] = {10, 20, 30, 40, 50, 10, 10};
    double kine_pg[4][4];
    if (FX_Robot_Kine_FK(0, jv, kine_pg) == false)
    {
        printf("Robot Forward Kinematics Error\n");
    }
    else
    {
        printf("Robot Forward Kinematics Success\n");
        print_matrix(kine_pg, 4, 4, "kine_pg");
    }

    printf("B arm\n");
    double jv1[7] = {10, 20, 30, 40, 50, 10, 0};
    double kine_pg1[4][4];
    if (FX_Robot_Kine_FK(1, jv1, kine_pg1) == false)
    {
        printf("Robot Forward Kinematics Error\n");
    }
    else
    {
        printf("Robot Forward Kinematics Success\n");
        print_matrix(kine_pg1, 4, 4, "kine_pg1");
    }
    printf("------------------------------\n");

    // [阶段三｜步骤 7] 根据目标位姿计算双臂逆运动学
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

    if (FX_Robot_Kine_IK(0, &sp) == false)
    {
        printf("Robot Inverse Kinamatics Error\n");
    }
    else
    {
        printf("Robot Inverse Kinamatics Success\n");
        print_array(sp.m_Output_RetJoint, 7, "ik result under reference joints");
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

    if (FX_Robot_Kine_IK(1, &sp1) == false)
    {
        printf("Robot Inverse Kinamatics Error\n");
    }
    else
    {
        printf("Robot Inverse Kinamatics Success\n");
        print_array(sp1.m_Output_RetJoint, 7, "ik result under reference joints");
    }
    printf("------------------------------\n");
}

int main()
{
    KineTwoArmsDemo();
}
